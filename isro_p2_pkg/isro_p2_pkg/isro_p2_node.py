# Copyright 2026 pi3
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from dataclasses import dataclass
import math
from pathlib import Path
import struct
import time

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TwistStamped
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus
from std_msgs.msg import String

try:
    import serial
except ImportError:  # pragma: no cover - handled at runtime with a clear log
    serial = None


SYNC = b'\xAC\x55\x96\x83'
MSG_ID_PVA = 2379
MSG_ID_IMU = 2389


def default_config_path() -> str:
    try:
        return str(Path(get_package_share_directory('isro_p2_pkg')) / 'config' / 'config_0402.txt')
    except Exception:
        return str(Path(__file__).resolve().parents[1] / 'config' / 'config_0402.txt')


DEFAULT_CONFIG = default_config_path()


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
class PvaData:
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

    @property
    def speed_mps(self):
        return math.sqrt(
            self.velocity_x_mps**2
            + self.velocity_y_mps**2
            + self.velocity_z_mps**2
        )

    @property
    def speed_kmh(self):
        return self.speed_mps * 3.6


@dataclass
class ImuData:
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
    return struct.unpack_from('<H', data, offset)[0]


def iter_frames(ser):
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
        payload = raw[payload_off:payload_off + payload_len]
        received_crc = struct.unpack_from('<I', raw, payload_off + payload_len)[0]
        calculated_crc = crc32_pimtp(raw[:-4])
        crc_valid = received_crc == calculated_crc
        next_sync = len(buf) == total_len or buf[total_len:total_len + len(SYNC)] == SYNC

        score = 0
        if crc_valid:
            score += 100
        if next_sync:
            score += 20
        if payload_off == 28:
            score += 4
        if msg_id_off == 12:
            score += 3

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


def parse_pva(payload: bytes) -> PvaData | None:
    layouts = (
        ('automotive_pva', 8, 36, 60),
        ('time_status_first', 8, 32, 60),
        ('status_type_first', 8, 32, 56),
        ('lat_first', 0, 32, 56),
        ('time_then_lat', 8, 40, 64),
        ('status_time_then_lat', 12, 44, 68),
    )

    best: tuple[int, PvaData] | None = None
    for name, pos_off, vel_off, att_off in layouts:
        need = att_off + 24
        if len(payload) < need:
            continue
        try:
            lat, lon, height = struct.unpack_from('<ddd', payload, pos_off)
            vx, vy, vz = struct.unpack_from('<ddd', payload, vel_off)
            roll, pitch, yaw = struct.unpack_from('<ddd', payload, att_off)
        except struct.error:
            continue

        score = score_pva(lat, lon, height, vx, vy, vz, roll, pitch, yaw)
        if score < 9:
            continue

        pva = PvaData(
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


def score_pva(lat, lon, height, vx, vy, vz, roll, pitch, yaw) -> int:
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


def parse_imu(payload: bytes) -> ImuData | None:
    if len(payload) < 36:
        return None

    try:
        gps_time, acc_z, acc_y, acc_x, gyro_z, gyro_y, gyro_x, status = struct.unpack_from(
            '<diiiiiiI', payload, 0
        )
    except struct.error:
        return None

    return ImuData(
        gps_time_s=gps_time,
        accel_x_mps2=acc_x * 1.0e-9,
        accel_y_mps2=acc_y * 1.0e-9,
        accel_z_mps2=acc_z * 1.0e-9,
        gyro_x_radps=gyro_x * 1.0e-10,
        gyro_y_radps=gyro_y * 1.0e-10,
        gyro_z_radps=gyro_z * 1.0e-10,
        status=status,
    )


class IsroP2Node(Node):
    def __init__(self):
        super().__init__('isro_p2_node')

        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 460800)
        self.declare_parameter('timeout', 0.02)
        self.declare_parameter('poll_hz', 100.0)
        self.declare_parameter('frame_id', 'isro_p2_link')
        self.declare_parameter('gps_frame_id', 'gps')
        self.declare_parameter('send_config', False)
        self.declare_parameter('config_path', DEFAULT_CONFIG)
        self.declare_parameter('log_crc_warnings', False)

        self.port = self.get_parameter('port').value
        self.baudrate = int(self.get_parameter('baudrate').value)
        self.timeout = float(self.get_parameter('timeout').value)
        self.poll_hz = float(self.get_parameter('poll_hz').value)
        self.frame_id = self.get_parameter('frame_id').value
        self.gps_frame_id = self.get_parameter('gps_frame_id').value

        self.raw_pub = self.create_publisher(String, 'pva/raw', 10)
        self.fix_pub = self.create_publisher(NavSatFix, 'pva/fix', 10)
        self.velocity_pub = self.create_publisher(TwistStamped, 'pva/velocity', 10)
        self.pva_imu_pub = self.create_publisher(Imu, 'pva/imu', 10)

        self.serial_port = self._open_serial()
        if bool(self.get_parameter('send_config').value):
            self._send_config(Path(self.get_parameter('config_path').value))

        self.frames = iter_frames(self.serial_port)
        timer_period = 1.0 / self.poll_hz if self.poll_hz > 0.0 else 0.01
        self.timer = self.create_timer(timer_period, self.read_once)

        self.get_logger().info(
            f'isro_p2_node started: port={self.port}, baudrate={self.baudrate}, '
            f'poll_hz={self.poll_hz}'
        )

    def _open_serial(self):
        if serial is None:
            raise RuntimeError(
                'python3-serial is not installed. Install it with: sudo apt install python3-serial'
            )

        try:
            ser = serial.Serial(self.port, self.baudrate, timeout=self.timeout)
        except serial.SerialException as exc:
            raise RuntimeError(f'Failed to open serial port {self.port}: {exc}') from exc

        time.sleep(0.5)
        self.get_logger().info('ISRO-P2 serial port opened')
        return ser

    def _send_config(self, config_path: Path) -> None:
        lines = [
            line.strip()
            for line in config_path.read_text(encoding='utf-8').splitlines()
            if line.strip() and not line.lstrip().startswith('#')
        ]
        for line in lines:
            self.serial_port.write(line.encode('ascii') + b'\r\n')
            self.serial_port.flush()
            self.get_logger().info(f'CONFIG > {line}')
            time.sleep(0.1)
        time.sleep(0.5)

    def read_once(self):
        frame = next(self.frames)
        if frame is None:
            return

        if bool(self.get_parameter('log_crc_warnings').value) and not frame.crc_valid:
            self.get_logger().warn(
                f'CRC mismatch: msg={frame.msg_id} '
                f'recv=0x{frame.received_crc:08X} calc=0x{frame.calculated_crc:08X}'
            )

        if frame.msg_id == MSG_ID_PVA:
            pva = parse_pva(frame.payload)
            if pva is None:
                self.get_logger().warn(f'PVA parse failed: payload_len={len(frame.payload)}')
                return
            self.publish_pva(pva)

    def publish_pva(self, pva: PvaData):
        stamp = self.get_clock().now().to_msg()
        raw = (
            f'lat={pva.latitude_deg:.9f}, lon={pva.longitude_deg:.9f}, '
            f'height={pva.height_m:.3f}, '
            f'vel=({pva.velocity_x_mps:.3f}, {pva.velocity_y_mps:.3f}, '
            f'{pva.velocity_z_mps:.3f}) m/s, speed={pva.speed_kmh:.2f} km/h, '
            f'roll={pva.roll_deg:.3f}, pitch={pva.pitch_deg:.3f}, '
            f'yaw={pva.yaw_deg:.3f}, layout={pva.layout}'
        )
        self.raw_pub.publish(String(data=raw))

        fix_msg = NavSatFix()
        fix_msg.header.stamp = stamp
        fix_msg.header.frame_id = self.gps_frame_id
        fix_msg.status.status = NavSatStatus.STATUS_FIX
        fix_msg.status.service = NavSatStatus.SERVICE_GPS
        fix_msg.latitude = pva.latitude_deg
        fix_msg.longitude = pva.longitude_deg
        fix_msg.altitude = pva.height_m
        fix_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
        self.fix_pub.publish(fix_msg)

        velocity_msg = TwistStamped()
        velocity_msg.header.stamp = stamp
        velocity_msg.header.frame_id = self.frame_id
        velocity_msg.twist.linear.x = pva.velocity_x_mps
        velocity_msg.twist.linear.y = pva.velocity_y_mps
        velocity_msg.twist.linear.z = pva.velocity_z_mps
        self.velocity_pub.publish(velocity_msg)

        imu_msg = Imu()
        imu_msg.header.stamp = stamp
        imu_msg.header.frame_id = self.frame_id
        roll = math.radians(pva.roll_deg)
        pitch = math.radians(pva.pitch_deg)
        yaw = math.radians(pva.yaw_deg)
        qx, qy, qz, qw = self._euler_to_quaternion(roll, pitch, yaw)
        imu_msg.orientation.x = qx
        imu_msg.orientation.y = qy
        imu_msg.orientation.z = qz
        imu_msg.orientation.w = qw
        imu_msg.orientation_covariance[0] = 0.0
        imu_msg.angular_velocity_covariance[0] = -1.0
        imu_msg.linear_acceleration_covariance[0] = -1.0
        self.pva_imu_pub.publish(imu_msg)

    @staticmethod
    def _euler_to_quaternion(roll, pitch, yaw):
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        return qx, qy, qz, qw

    def destroy_node(self):
        if hasattr(self, 'timer'):
            self.timer.cancel()
        if hasattr(self, 'serial_port') and self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
            if rclpy.ok():
                self.get_logger().info('Serial port closed')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = IsroP2Node()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        if node is not None:
            node.get_logger().error(str(exc))
        else:
            print(exc)
        raise
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
