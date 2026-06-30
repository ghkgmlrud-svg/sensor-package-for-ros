import argparse
import math
import queue
import struct
import threading
import time

import serial


ISRO_PORT = "/dev/ttyUSB0"
ISRO_BAUDRATE = 460800
ISRO_SYNC = b"\xAC\x55\x96\x83"
ISRO_MSG_ID_PVA = 2379
ISRO_MSG_ID_IMU = 2389
ISRO_PREFERRED_MSG_ID_OFFSET = 12
ISRO_PREFERRED_PAYLOAD_OFFSET = 28

MW_PORT = "/dev/ttyUSB1"
MW_BAUDRATE = 115200


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


def isro_crc_check(raw, payload_offset, payload_len):
    crc_offset = payload_offset + payload_len
    received_le = read_u32(raw, crc_offset)
    received_be = read_u32_be(raw, crc_offset)

    ranges = (
        ("sync+header+payload", raw[:crc_offset]),
        ("header+payload", raw[len(ISRO_SYNC):crc_offset]),
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


def find_isro_frame(buffer):
    if len(buffer) < 42:
        return None

    best_frame = None
    best_score = -1

    for msg_id_offset in (12, 14, 16, 18):
        for payload_offset in (24, 28, 32, 34, 36, 38, 40, 42, 44, 48):
            if len(buffer) < payload_offset:
                continue

            msg_id = read_u16(buffer, msg_id_offset)
            if msg_id not in (ISRO_MSG_ID_PVA, ISRO_MSG_ID_IMU):
                continue

            length_offset = msg_id_offset + 2
            payload_len = read_u16(buffer, length_offset)
            if payload_len <= 0 or payload_len > 4096:
                continue

            total_len = payload_offset + payload_len + 4
            if len(buffer) < total_len:
                continue

            raw = bytes(buffer[:total_len])
            payload = raw[payload_offset:payload_offset + payload_len]
            crc_ok, crc_method, received_crc, calculated_crc = isro_crc_check(
                raw, payload_offset, payload_len
            )
            next_sync_ok = (
                len(buffer) == total_len
                or buffer[total_len:total_len + len(ISRO_SYNC)] == ISRO_SYNC
            )

            score = 0
            if crc_ok:
                score += 200
            if next_sync_ok:
                score += 50
            if payload_offset == ISRO_PREFERRED_PAYLOAD_OFFSET:
                score += 10
            if msg_id_offset == ISRO_PREFERRED_MSG_ID_OFFSET:
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
                }

    return best_frame


def parse_isro_pva(payload):
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
        if len(payload) < att_offset + 24:
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
    if not all(math.isfinite(value) for value in values):
        return 0

    score = 0
    if -90.0 <= lat <= 90.0 and abs(lat) > 0.000001:
        score += 2
    if -180.0 <= lon <= 180.0 and abs(lon) > 0.000001:
        score += 2
    if -1000.0 <= height <= 10000.0:
        score += 1
    if all(abs(value) < 150.0 for value in (vel_x, vel_y, vel_z)):
        score += 1
    if -180.0 <= roll <= 180.0:
        score += 1
    if -90.0 <= pitch <= 90.0:
        score += 1
    if -360.0 <= yaw <= 360.0:
        score += 1
    return score


def format_isro_pva(pva, crc_ok, show_crc):
    crc_text = f"crc={'OK' if crc_ok else 'BAD'} " if show_crc else ""
    return (
        crc_text
        + f"lat={pva['lat']:.9f}, lon={pva['lon']:.9f}, h={pva['height']:.3f} m, "
        f"speed={pva['speed_kmh']:.2f} km/h, "
        f"roll={pva['roll']:.3f}, pitch={pva['pitch']:.3f}, yaw={pva['yaw']:.3f}"
    )


def parse_mw_line(data):
    values = []
    for part in data.replace(",", " ").split():
        try:
            values.append(float(part))
        except ValueError:
            pass
    return values


def format_mw(values, raw):
    if len(values) >= 3:
        return f"roll={values[0]:.2f}, pitch={values[1]:.2f}, yaw={values[2]:.2f}"
    return raw


def format_fused_line(isro_event, mw_event):
    isro_time = isro_event["time"]
    mw_time = mw_event["time"]
    dt = abs(isro_time - mw_time)
    timestamp = time.strftime("%H:%M:%S", time.localtime(max(isro_time, mw_time)))
    millis = int((max(isro_time, mw_time) % 1.0) * 1000)
    isro = isro_event["pva"]
    mw_text = format_mw(mw_event["values"], mw_event["raw"])

    return (
        f"{timestamp}.{millis:03d} dt={dt:.3f}s | "
        f"ISRO lat={isro['lat']:.9f}, lon={isro['lon']:.9f}, "
        f"h={isro['height']:.2f}m, speed={isro['speed_kmh']:.2f}km/h, "
        f"roll={isro['roll']:.2f}, pitch={isro['pitch']:.2f}, yaw={isro['yaw']:.2f} | "
        f"MW {mw_text}"
    )


def read_isro(args, output_queue, stop_event):
    try:
        with serial.Serial(args.isro_port, args.isro_baudrate, timeout=args.isro_timeout) as ser:
            time.sleep(0.5)
            output_queue.put(("SYSTEM", f"ISRO connected: {args.isro_port} @ {args.isro_baudrate}"))
            buffer = bytearray()

            while not stop_event.is_set():
                chunk = ser.read(args.isro_read_size)
                if not chunk:
                    continue

                buffer.extend(chunk)
                while not stop_event.is_set():
                    sync_index = buffer.find(ISRO_SYNC)

                    if sync_index < 0:
                        if len(buffer) > len(ISRO_SYNC) - 1:
                            del buffer[:-(len(ISRO_SYNC) - 1)]
                        break

                    if sync_index > 0:
                        del buffer[:sync_index]

                    frame = find_isro_frame(buffer)
                    if frame is None:
                        break

                    del buffer[:frame["raw_len"]]

                    if frame["msg_id"] != ISRO_MSG_ID_PVA:
                        continue

                    if args.strict_crc and not frame["crc_ok"]:
                        continue

                    pva = parse_isro_pva(frame["payload"])
                    if pva:
                        output_queue.put(
                            (
                                "ISRO",
                                {
                                    "time": time.time(),
                                    "pva": pva,
                                    "crc_ok": frame["crc_ok"],
                                    "text": format_isro_pva(pva, frame["crc_ok"], args.show_crc),
                                },
                            )
                        )
    except serial.SerialException as exc:
        output_queue.put(("ERROR", f"ISRO serial error: {exc}"))
        stop_event.set()


def configure_mw(ser, output_queue):
    ser.reset_input_buffer()
    ser.write(b"ss=4\r\n")
    ser.write(b"sp=100\r\n")
    ser.flush()
    time.sleep(1.0)
    ser.reset_input_buffer()
    output_queue.put(("SYSTEM", "MW configured: ss=4, sp=100"))


def read_mw(args, output_queue, stop_event):
    try:
        with serial.Serial(args.mw_port, args.mw_baudrate, timeout=args.mw_timeout) as ser:
            time.sleep(0.5)
            output_queue.put(("SYSTEM", f"MW connected: {args.mw_port} @ {args.mw_baudrate}"))
            if not args.skip_mw_config:
                configure_mw(ser, output_queue)

            bad_lines = 0
            while not stop_event.is_set():
                line = ser.readline()
                if not line:
                    continue

                printable_count = sum(1 for byte in line if byte in (9, 10, 13) or 32 <= byte <= 126)
                printable_ratio = printable_count / max(len(line), 1)
                data = line.decode("utf-8", errors="ignore").strip()

                if printable_ratio < 0.7:
                    bad_lines += 1
                    if bad_lines == 5:
                        output_queue.put(
                            (
                                "ERROR",
                                "MW data looks like binary/noise. Check port order and baudrate. "
                                "Usually ISRO=/dev/ttyUSB0, MW=/dev/ttyUSB1.",
                            )
                        )
                    continue

                bad_lines = 0
                if data:
                    output_queue.put(
                        (
                            "MW",
                            {
                                "time": time.time(),
                                "raw": data,
                                "values": parse_mw_line(data),
                            },
                        )
                    )
    except serial.SerialException as exc:
        output_queue.put(("ERROR", f"MW serial error: {exc}"))
        stop_event.set()


def print_loop(output_queue, stop_event, max_lines):
    printed = 0
    latest_isro = None
    latest_mw = None

    while not stop_event.is_set():
        try:
            source, payload = output_queue.get(timeout=0.2)
        except queue.Empty:
            continue

        if source in ("SYSTEM", "ERROR"):
            now = time.strftime("%H:%M:%S")
            print(f"{now} [{source}] {payload}", flush=True)
            continue

        if source == "MW":
            latest_mw = payload
            continue

        if source == "ISRO":
            latest_isro = payload
            if latest_mw is None:
                continue

            print(format_fused_line(latest_isro, latest_mw), flush=True)
            printed += 1

        if max_lines and printed >= max_lines:
            stop_event.set()
            break


def main():
    parser = argparse.ArgumentParser(description="Read ISRO-P2 and MW-AHRS-X1 together.")
    parser.add_argument("--isro-port", default=ISRO_PORT)
    parser.add_argument("--isro-baudrate", type=int, default=ISRO_BAUDRATE)
    parser.add_argument("--isro-timeout", type=float, default=0.05)
    parser.add_argument("--isro-read-size", type=int, default=256)
    parser.add_argument("--mw-port", default=MW_PORT)
    parser.add_argument("--mw-baudrate", type=int, default=MW_BAUDRATE)
    parser.add_argument("--mw-timeout", type=float, default=0.05)
    parser.add_argument("--skip-mw-config", action="store_true")
    parser.add_argument("--strict-crc", action="store_true", help="ISRO CRC OK인 PVA만 출력")
    parser.add_argument("--show-crc", action="store_true", help="ISRO 출력에 CRC 상태 표시")
    parser.add_argument("--max-lines", type=int, default=0, help="센서 데이터 줄 수 제한, 0이면 계속 실행")
    args = parser.parse_args()

    if args.isro_port == args.mw_port:
        raise SystemExit(
            "ISRO와 MW 포트가 같습니다. 두 센서는 서로 다른 포트여야 합니다. "
            "예: --isro-port /dev/ttyUSB0 --mw-port /dev/ttyUSB1"
        )

    output_queue = queue.Queue()
    stop_event = threading.Event()

    threads = [
        threading.Thread(target=read_isro, args=(args, output_queue, stop_event), daemon=True),
        threading.Thread(target=read_mw, args=(args, output_queue, stop_event), daemon=True),
    ]

    for thread in threads:
        thread.start()

    try:
        print_loop(output_queue, stop_event, args.max_lines)
    except KeyboardInterrupt:
        print("\nStopped", flush=True)
        stop_event.set()

    for thread in threads:
        thread.join(timeout=1.0)


if __name__ == "__main__":
    main()
