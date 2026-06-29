#!/usr/bin/env python3
"""ISRO-P2 PIMTP serial parser for PVA and IMU logs."""

from __future__ import annotations

import argparse
import math
import struct
import sys
import time
from dataclasses import dataclass
from pathlib import Path

import serial


SYNC = b"\xAC\x55\x96\x83"
MSG_ID_PVA = 2379
MSG_ID_IMU = 2389

DEFAULT_PORT = "/dev/ttyUSB0"
DEFAULT_BAUDRATE = 460800
DEFAULT_CONFIG = Path("/home/pi3/docs/config_0402.txt")


@dataclass
class PimtpFrame:
    msg_id: int
    payload: bytes
    raw: bytes
    msg_id_offset: int
    length_offset: int
    payload_offset: int
    payload_len: int
    crc_valid: bool
    received_crc: int
    calculated_crc: int


@dataclass
class Pva:
    latitude_deg: float
    longitude_deg: float
    height_m: float
    velocity_x_mps: float
    velocity_y_mps: float
    velocity_z_mps: float
    roll_deg: float
    pitch_deg: float
    yaw_deg: float
    layout: str


@dataclass
class Imu:
    gps_time_s: float
    accel_x_mps2: float
    accel_y_mps2: float
    accel_z_mps2: float
    gyro_x_radps: float
    gyro_y_radps: float
    gyro_z_radps: float
    status: int


def crc32_pimtp(data: bytes) -> int:
    crc = 0xFFFFFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xEDB88320
            else:
                crc >>= 1
    return crc ^ 0xFFFFFFFF


def read_u16_le(data: bytes, offset: int) -> int:
    return struct.unpack_from("<H", data, offset)[0]


def iter_frames(ser: serial.Serial):
    buf = bytearray()
    while True:
        chunk = ser.read(4096)
        if chunk:
            buf.extend(chunk)
        else:
            yield None
            continue

        while True:
            sync_at = buf.find(SYNC)
            if sync_at < 0:
                del buf[:-3]
                break
            if sync_at:
                del buf[:sync_at]

            frame = try_extract_frame(buf)
            if frame is None:
                break

            del buf[: len(frame.raw)]
            yield frame


def try_extract_frame(buf: bytearray) -> PimtpFrame | None:
    if len(buf) < 42:
        return None

    # The field log shows several firmware/header variants in the wild. Score
    # all nearby interpretations and prefer one whose CRC and next sync align.
    candidates = []
    for msg_id_off in (12, 14, 16, 18):
        for payload_off in (24, 28, 32, 34, 36, 38, 40, 42, 44, 48):
            candidates.append((msg_id_off, msg_id_off + 2, payload_off))

    best: tuple[int, PimtpFrame] | None = None
    waiting_for_more = False
    for msg_id_off, length_off, payload_off in candidates:
        if len(buf) < payload_off:
            continue
        msg_id = read_u16_le(buf, msg_id_off)
        if msg_id not in (MSG_ID_PVA, MSG_ID_IMU):
            continue

        payload_len = read_u16_le(buf, length_off)
        if payload_len <= 0 or payload_len > 4096:
            continue

        total_len = payload_off + payload_len + 4
        if len(buf) < total_len:
            waiting_for_more = True
            continue

        raw = bytes(buf[:total_len])
        payload = raw[payload_off : payload_off + payload_len]
        received_crc = struct.unpack_from("<I", raw, payload_off + payload_len)[0]
        calculated_crc = crc32_pimtp(raw[:-4])
        crc_valid = received_crc == calculated_crc
        next_sync = len(buf) == total_len or buf[total_len : total_len + len(SYNC)] == SYNC

        score = 0
        if crc_valid:
            score += 100
        if next_sync:
            score += 20
        if payload_off == 38:
            score += 3
        if msg_id_off == 14:
            score += 2

        frame = PimtpFrame(
            msg_id=msg_id,
            payload=payload,
            raw=raw,
            msg_id_offset=msg_id_off,
            length_offset=length_off,
            payload_offset=payload_off,
            payload_len=payload_len,
            crc_valid=crc_valid,
            received_crc=received_crc,
            calculated_crc=calculated_crc,
        )
        if best is None or score > best[0]:
            best = (score, frame)

    if best:
        return best[1]
    if waiting_for_more:
        return None

    del buf[0]
    return None


def parse_pva(payload: bytes) -> Pva | None:
    layouts = (
        ("automotive_pva", 8, 36, 60),
        ("time_status_first", 8, 32, 60),
        ("status_type_first", 8, 32, 56),
        ("lat_first", 0, 32, 56),
        ("time_then_lat", 8, 40, 64),
        ("status_time_then_lat", 12, 44, 68),
    )

    best: tuple[int, Pva] | None = None
    for name, pos_off, vel_off, att_off in layouts:
        need = att_off + 24
        if len(payload) < need:
            continue
        try:
            lat, lon, height = struct.unpack_from("<ddd", payload, pos_off)
            vx, vy, vz = struct.unpack_from("<ddd", payload, vel_off)
            roll, pitch, yaw = struct.unpack_from("<ddd", payload, att_off)
        except struct.error:
            continue

        score = score_pva(lat, lon, height, vx, vy, vz, roll, pitch, yaw)
        if score < 9:
            continue

        pva = Pva(
            latitude_deg=lat,
            longitude_deg=lon,
            height_m=height,
            velocity_x_mps=vx,
            velocity_y_mps=vy,
            velocity_z_mps=vz,
            roll_deg=roll,
            pitch_deg=pitch,
            yaw_deg=yaw,
            layout=name,
        )
        if best is None or score > best[0]:
            best = (score, pva)

    return best[1] if best else None


def score_pva(
    lat: float,
    lon: float,
    height: float,
    vx: float,
    vy: float,
    vz: float,
    roll: float,
    pitch: float,
    yaw: float,
) -> int:
    values = (lat, lon, height, vx, vy, vz, roll, pitch, yaw)
    if not all(math.isfinite(v) for v in values):
        return 0

    score = 0
    if -90.0 <= lat <= 90.0 and abs(lat) > 0.000001:
        score += 2
    if -180.0 <= lon <= 180.0 and abs(lon) > 0.000001:
        score += 2
    if -1000.0 <= height <= 10000.0:
        score += 1
    if all(abs(v) < 150.0 for v in (vx, vy, vz)):
        score += 1
    if -180.0 <= roll <= 180.0:
        score += 1
    if -90.0 <= pitch <= 90.0:
        score += 1
    if -360.0 <= yaw <= 360.0:
        score += 1
    return score


def hex_preview(data: bytes, limit: int = 96) -> str:
    shown = data[:limit]
    suffix = " ..." if len(data) > limit else ""
    return " ".join(f"{byte:02X}" for byte in shown) + suffix


def print_frame_debug(frame: PimtpFrame) -> None:
    status = "ok" if frame.crc_valid else "bad"
    print(
        "FRAME "
        f"msg={frame.msg_id} payload_len={frame.payload_len} "
        f"offsets=(msg:{frame.msg_id_offset}, len:{frame.length_offset}, payload:{frame.payload_offset}) "
        f"crc={status} recv=0x{frame.received_crc:08X} calc=0x{frame.calculated_crc:08X}"
    )
    print(f"RAW {hex_preview(frame.raw)}")
    print(f"PAYLOAD {hex_preview(frame.payload)}")


def parse_imu(payload: bytes) -> Imu | None:
    if len(payload) < 36:
        return None

    try:
        gps_time, acc_z, acc_y, acc_x, gyro_z, gyro_y, gyro_x, status = struct.unpack_from(
            "<diiiiiiI", payload, 0
        )
    except struct.error:
        return None

    return Imu(
        gps_time_s=gps_time,
        accel_x_mps2=acc_x * 1.0e-9,
        accel_y_mps2=acc_y * 1.0e-9,
        accel_z_mps2=acc_z * 1.0e-9,
        gyro_x_radps=gyro_x * 1.0e-10,
        gyro_y_radps=gyro_y * 1.0e-10,
        gyro_z_radps=gyro_z * 1.0e-10,
        status=status,
    )


def send_config(ser: serial.Serial, config_path: Path) -> None:
    lines = [
        line.strip()
        for line in config_path.read_text(encoding="utf-8").splitlines()
        if line.strip() and not line.lstrip().startswith("#")
    ]
    for line in lines:
        ser.write(line.encode("ascii") + b"\r\n")
        ser.flush()
        print(f"CONFIG > {line}")
        time.sleep(0.1)


def print_pva(pva: Pva) -> None:
    speed_mps = math.sqrt(
        pva.velocity_x_mps**2 + pva.velocity_y_mps**2 + pva.velocity_z_mps**2
    )
    speed_kmh = speed_mps * 3.6
    print(
        "PVA "
        f"lat={pva.latitude_deg:.9f} deg, "
        f"lon={pva.longitude_deg:.9f} deg, "
        f"h={pva.height_m:.3f} m | "
        f"vel=({pva.velocity_x_mps:.3f}, {pva.velocity_y_mps:.3f}, {pva.velocity_z_mps:.3f}) m/s "
        f"speed={speed_kmh:.2f} km/h | "
        f"att=roll {pva.roll_deg:.3f}, pitch {pva.pitch_deg:.3f}, yaw {pva.yaw_deg:.3f} deg "
        f"[{pva.layout}]"
    )


def print_imu(imu: Imu) -> None:
    print(
        "IMU "
        f"t={imu.gps_time_s:.3f}s | "
        f"acc=({imu.accel_x_mps2:.6f}, {imu.accel_y_mps2:.6f}, {imu.accel_z_mps2:.6f}) m/s^2 | "
        f"gyro=({imu.gyro_x_radps:.6f}, {imu.gyro_y_radps:.6f}, {imu.gyro_z_radps:.6f}) rad/s | "
        f"status=0x{imu.status:08X}"
    )


def main() -> None:
    parser = argparse.ArgumentParser(description="Read ISRO-P2 PIMTP PVA/IMU serial data.")
    parser.add_argument("--port", default=DEFAULT_PORT)
    parser.add_argument("--baudrate", type=int, default=DEFAULT_BAUDRATE)
    parser.add_argument("--timeout", type=float, default=0.2)
    parser.add_argument("--send-config", action="store_true", help="Send config_0402.txt before reading.")
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--show-imu", action="store_true", help="Also print IMU frames.")
    parser.add_argument("--debug", action="store_true", help="Print frame offsets, CRC status, and raw hex.")
    parser.add_argument("--dump-raw", type=int, default=0, help="Print debug data for this many frames, then keep reading.")
    parser.add_argument("--max-frames", type=int, default=0, help="Stop after this many parsed frames; 0 means keep reading.")
    parser.add_argument("--show-crc-warning", action="store_true", help="Print CRC mismatches outside debug dumps.")
    args = parser.parse_args()

    with serial.Serial(args.port, args.baudrate, timeout=args.timeout) as ser:
        print(f"ISRO-P2 connected: {args.port} @ {args.baudrate}")
        if args.send_config:
            send_config(ser, args.config)
            time.sleep(0.5)

        print("Reading PIMTP PVA/IMU frames. Press Ctrl+C to stop.")
        dumped = 0
        frame_count = 0
        for frame in iter_frames(ser):
            if frame is None:
                continue

            frame_count += 1
            if args.debug or dumped < args.dump_raw:
                print_frame_debug(frame)
                dumped += 1
            elif args.show_crc_warning and not frame.crc_valid:
                print(
                    f"CRC warning: msg={frame.msg_id} recv=0x{frame.received_crc:08X} "
                    f"calc=0x{frame.calculated_crc:08X}",
                    file=sys.stderr,
                )

            if frame.msg_id == MSG_ID_PVA:
                pva = parse_pva(frame.payload)
                if pva:
                    print_pva(pva)
                else:
                    print(f"PVA parse failed: payload_len={len(frame.payload)}")
            elif frame.msg_id == MSG_ID_IMU and args.show_imu:
                imu = parse_imu(frame.payload)
                if imu:
                    print_imu(imu)
                else:
                    print(f"IMU parse failed: payload_len={len(frame.payload)}")

            if args.max_frames and frame_count >= args.max_frames:
                break


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nStopped")
