import math
import struct
import time
import argparse

import serial


# 기본 설정
PORT = "/dev/ttyUSB0"
BAUDRATE = 460800

# PIMTP 동기화 바이트
SYNC = b"\xAC\x55\x96\x83"

# PVA 메시지 ID
MSG_ID_PVA = 2379
MSG_ID_IMU = 2389

# 문서/ROS 노드에서 확인한 현재 ISRO-P2 PIMTP 기준 offset
PREFERRED_MSG_ID_OFFSET = 12
PREFERRED_PAYLOAD_OFFSET = 28


# 받은 패킷이 정상인지 확인하는 함수
def crc32_pimtp(data):
    crc = 0

    for byte in data:
        crc ^= byte

        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xEDB88320
            else:
                crc >>= 1

    return crc & 0xFFFFFFFF


def read_u16(data, offset):
    return struct.unpack_from("<H", data, offset)[0]


def read_u32(data, offset):
    return struct.unpack_from("<I", data, offset)[0]


def read_u32_be(data, offset):
    return struct.unpack_from(">I", data, offset)[0]


def crc_check(raw, payload_offset, payload_len):
    crc_offset = payload_offset + payload_len
    received_le = read_u32(raw, crc_offset)
    received_be = read_u32_be(raw, crc_offset)

    ranges = (
        ("sync+header+payload", raw[:crc_offset]),
        ("header+payload", raw[len(SYNC) : crc_offset]),
        ("payload", raw[payload_offset:crc_offset]),
    )

    for name, data in ranges:
        calculated = crc32_pimtp(data)
        if calculated == received_le:
            return True, name, received_le, calculated
        if calculated == received_be:
            return True, name + "/crc_be", received_be, calculated

    calculated = crc32_pimtp(raw[:crc_offset])
    return False, "sync+header+payload", received_le, calculated


def find_frame(buffer):
    # PIMTP 패킷 하나를 찾아서 반환한다. IMU 프레임도 소비해야 뒤의 PVA까지 진행된다.

    if len(buffer) < 42:
        return None

    candidates = []

    for msg_id_offset in (12, 14, 16, 18):
        for payload_offset in (24, 28, 32, 34, 36, 38, 40, 42, 44, 48):
            candidates.append((msg_id_offset, msg_id_offset + 2, payload_offset))

    best_frame = None
    best_score = -1

    for msg_id_offset, length_offset, payload_offset in candidates:
        if len(buffer) < payload_offset:
            continue

        msg_id = read_u16(buffer, msg_id_offset)

        if msg_id not in (MSG_ID_PVA, MSG_ID_IMU):
            continue

        payload_len = read_u16(buffer, length_offset)

        if payload_len <= 0 or payload_len > 4096:
            continue

        total_len = payload_offset + payload_len + 4

        if len(buffer) < total_len:
            continue

        raw = bytes(buffer[:total_len])
        payload = raw[payload_offset : payload_offset + payload_len]

        crc_ok, crc_method, received_crc, calculated_crc = crc_check(
            raw, payload_offset, payload_len
        )

        next_sync_ok = (
            len(buffer) == total_len
            or buffer[total_len : total_len + len(SYNC)] == SYNC
        )

        score = 0

        if crc_ok:
            score += 200

        if next_sync_ok:
            score += 50

        if payload_offset == PREFERRED_PAYLOAD_OFFSET:
            score += 10

        if msg_id_offset == PREFERRED_MSG_ID_OFFSET:
            score += 10

        if score > best_score:
            best_score = score
            best_frame = {
                "msg_id": msg_id,
                "payload": payload,
                "raw_len": total_len,
                "crc_ok": crc_ok,
                "crc_method": crc_method,
                "received_crc": received_crc,
                "calculated_crc": calculated_crc,
                "msg_id_offset": msg_id_offset,
                "length_offset": length_offset,
                "payload_offset": payload_offset,
                "payload_len": payload_len,
                "next_sync_ok": next_sync_ok,
            }

    return best_frame


#PVA 값 파싱 PVA payload에서 실제 값을 꺼냄
def parse_pva(payload):
    # PVA 데이터 시작 위치 후보들
    layouts = (
        (8, 36, 60),
        (8, 32, 60),
        (8, 32, 56),
        (0, 32, 56),
        (8, 40, 64),
        (12, 44, 68),
    )

    best = None
    best_score = -1

    for pos_offset, vel_offset, att_offset in layouts:
        need_len = att_offset + 24

        if len(payload) < need_len:
            continue

        try:
            lat, lon, height = struct.unpack_from("<ddd", payload, pos_offset)
            vel_x, vel_y, vel_z = struct.unpack_from("<ddd", payload, vel_offset)
            roll, pitch, yaw = struct.unpack_from("<ddd", payload, att_offset)
        except struct.error:
            continue

        score = score_pva(lat, lon, height, vel_x, vel_y, vel_z, roll, pitch, yaw)

        if score > best_score:
            best_score = score
            best = {
                "lat": lat,
                "lon": lon,
                "height": height,
                "vel_x": vel_x,
                "vel_y": vel_y,
                "vel_z": vel_z,
                "roll": roll,
                "pitch": pitch,
                "yaw": yaw,
            }

    if best is None or best_score < 9:
        return None

    speed_mps = math.sqrt(best["vel_x"] ** 2 + best["vel_y"] ** 2 + best["vel_z"] ** 2)
    best["speed_kmh"] = speed_mps * 3.6

    return best


def score_pva(lat, lon, height, vel_x, vel_y, vel_z, roll, pitch, yaw):
    values = (lat, lon, height, vel_x, vel_y, vel_z, roll, pitch, yaw)

    if not all(math.isfinite(v) for v in values):
        return 0

    score = 0

    if -90.0 <= lat <= 90.0 and abs(lat) > 0.000001:
        score += 2

    if -180.0 <= lon <= 180.0 and abs(lon) > 0.000001:
        score += 2

    if -1000.0 <= height <= 10000.0:
        score += 1

    if all(abs(v) < 150.0 for v in (vel_x, vel_y, vel_z)):
        score += 1

    if -180.0 <= roll <= 180.0:
        score += 1

    if -90.0 <= pitch <= 90.0:
        score += 1

    if -360.0 <= yaw <= 360.0:
        score += 1

    return score


def hex_preview(data, limit=96):
    shown = data[:limit]
    suffix = " ..." if len(data) > limit else ""
    return " ".join(f"{byte:02X}" for byte in shown) + suffix


def print_frame_debug(frame):
    crc_status = "OK" if frame["crc_ok"] else "BAD"
    print(
        "FRAME "
        f"msg={frame['msg_id']} payload_len={frame['payload_len']} "
        f"offsets=(msg:{frame['msg_id_offset']}, len:{frame['length_offset']}, "
        f"payload:{frame['payload_offset']}) "
        f"next_sync={frame['next_sync_ok']} "
        f"crc={crc_status} method={frame['crc_method']} "
        f"recv=0x{frame['received_crc']:08X} calc=0x{frame['calculated_crc']:08X}"
    )
    print(f"PAYLOAD {hex_preview(frame['payload'])}", flush=True)


def print_pva(pva, frame=None, show_crc_status=False):
    prefix = ""
    if frame is not None and show_crc_status:
        crc_status = "CRC OK" if frame["crc_ok"] else "CRC BAD"
        prefix = f"[{crc_status} {frame['crc_method']}] "

    print(
        prefix +
        f"lat={pva['lat']:.9f}, "
        f"lon={pva['lon']:.9f}, "
        f"h={pva['height']:.3f} m, "
        f"speed={pva['speed_kmh']:.2f} km/h, "
        f"roll={pva['roll']:.3f}, "
        f"pitch={pva['pitch']:.3f}, "
        f"yaw={pva['yaw']:.3f}",
        flush=True,
    )


def main():
    parser = argparse.ArgumentParser(description="Read ISRO-P2 PVA data.")
    parser.add_argument("--port", default=PORT)
    parser.add_argument("--baudrate", type=int, default=BAUDRATE)
    parser.add_argument("--timeout", type=float, default=0.05)
    parser.add_argument("--read-size", type=int, default=256, help="한 번에 읽을 시리얼 바이트 수")
    parser.add_argument("--strict-crc", action="store_true", help="CRC OK인 프레임만 출력")
    parser.add_argument("--show-crc-status", action="store_true", help="PVA 출력 앞에 CRC 상태 표시")
    parser.add_argument("--debug", action="store_true", help="프레임 offset/CRC 정보를 같이 출력")
    parser.add_argument("--max-frames", type=int, default=0, help="지정한 개수 출력 후 종료")
    args = parser.parse_args()

    ser = serial.Serial(args.port, args.baudrate, timeout=args.timeout)
    time.sleep(1)

    buffer = bytearray()
    printed = 0

    try:
        while True:
            chunk = ser.read(args.read_size)

            if not chunk:
                continue

            buffer.extend(chunk)

            while True:
                sync_index = buffer.find(SYNC)

                if sync_index < 0:
                    if len(buffer) > 3:
                        del buffer[:-3]
                    break

                if sync_index > 0:
                    del buffer[:sync_index]

                frame = find_frame(buffer)

                if frame is None:
                    break

                del buffer[: frame["raw_len"]]

                if args.debug:
                    print_frame_debug(frame)

                if frame["msg_id"] != MSG_ID_PVA:
                    continue

                if args.strict_crc and not frame["crc_ok"]:
                    continue

                pva = parse_pva(frame["payload"])

                if pva:
                    print_pva(pva, frame, args.show_crc_status)
                    printed += 1

                    if args.max_frames and printed >= args.max_frames:
                        return

    except KeyboardInterrupt:
        pass
    except serial.SerialException as exc:
        print(f"Serial error: {exc}")
        print("Check whether another process is using the same port, or whether the device was disconnected.")

    finally:
        ser.close()


if __name__ == "__main__":
    main()
