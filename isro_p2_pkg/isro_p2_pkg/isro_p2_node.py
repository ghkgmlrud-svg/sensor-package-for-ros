import math
from pathlib import Path
import struct
import time
from dataclasses import dataclass

from geometry_msgs.msg import TwistStamped
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus
from std_msgs.msg import String

try:
    import serial
except ImportError:  # pragma: no cover
    serial = None


SYNC = b"\xAC\x55\x96\x83"
MSG_ID_PVA = 2379
MSG_ID_IMU = 2389

PREFERRED_MSG_ID_OFFSET = 12
PREFERRED_PAYLOAD_OFFSET = 28


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
    crc_method: str
    received_crc: int
    calculated_crc: int
    next_sync_ok: bool


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


def crc32_pimtp(data: bytes) -> int:
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xEDB88320
            else:
                crc >>= 1
    return crc & 0xFFFFFFFF


def read_u16_le(data: bytes, offset: int) -> int:
    return struct.unpack_from("<H", data, offset)[0]


def read_u32_le(data: bytes, offset: int) -> int:
    return struct.unpack_from("<I", data, offset)[0]


def read_u32_be(data: bytes, offset: int) -> int:
    return struct.unpack_from(">I", data, offset)[0]


def check_crc(raw: bytes, payload_offset: int, payload_len: int):
    crc_offset = payload_offset + payload_len
    received_le = read_u32_le(raw, crc_offset)
    received_be = read_u32_be(raw, crc_offset)
    ranges = (
        ("sync+header+payload", raw[:crc_offset]),
        ("header+payload", raw[len(SYNC):crc_offset]),
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


def try_extract_frame(buf: bytearray) -> PimtpFrame | None:
    if len(buf) < 42:
        return None

    best: tuple[int, PimtpFrame] | None = None
    waiting_for_more = False

    for msg_id_offset in (12, 14, 16, 18):
        for payload_offset in (24, 28, 32, 34, 36, 38, 40, 42, 44, 48):
            if len(buf) < payload_offset:
                continue

            msg_id = read_u16_le(buf, msg_id_offset)
            if msg_id not in (MSG_ID_PVA, MSG_ID_IMU):
                continue

            length_offset = msg_id_offset + 2
            payload_len = read_u16_le(buf, length_offset)
            if payload_len <= 0 or payload_len > 4096:
                continue

            total_len = payload_offset + payload_len + 4
            if len(buf) < total_len:
                waiting_for_more = True
                continue

            raw = bytes(buf[:total_len])
            payload = raw[payload_offset:payload_offset + payload_len]
            crc_valid, crc_method, received_crc, calculated_crc = check_crc(
                raw, payload_offset, payload_len
            )
            next_sync_ok = (
                len(buf) == total_len
                or buf[total_len:total_len + len(SYNC)] == SYNC
            )

            score = 0
            if crc_valid:
                score += 200
            if next_sync_ok:
                score += 50
            if payload_offset == PREFERRED_PAYLOAD_OFFSET:
                score += 10
            if msg_id_offset == PREFERRED_MSG_ID_OFFSET:
                score += 10

            frame = PimtpFrame(
                msg_id=msg_id,
                payload=payload,
                raw=raw,
                msg_id_offset=msg_id_offset,
                length_offset=length_offset,
                payload_offset=payload_offset,
                payload_len=payload_len,
                crc_valid=crc_valid,
                crc_method=crc_method,
                received_crc=received_crc,
                calculated_crc=calculated_crc,
                next_sync_ok=next_sync_ok,
            )
            if best is None or score > best[0]:
                best = (score, frame)

    if best is not None:
        return best[1]
    if waiting_for_more:
        return None

    del buf[0]
    return None


def parse_pva(payload: bytes) -> PvaData | None:
    layouts = (
        ("automotive_pva", 8, 36, 60),
        ("time_status_first", 8, 32, 60),
        ("status_type_first", 8, 32, 56),
        ("lat_first", 0, 32, 56),
        ("time_then_lat", 8, 40, 64),
        ("status_time_then_lat", 12, 44, 68),
    )

    best: tuple[int, PvaData] | None = None
    for name, pos_offset, vel_offset, att_offset in layouts:
        if len(payload) < att_offset + 24:
            continue

        try:
            lat, lon, height = struct.unpack_from("<ddd", payload, pos_offset)
            vx, vy, vz = struct.unpack_from("<ddd", payload, vel_offset)
            roll, pitch, yaw = struct.unpack_from("<ddd", payload, att_offset)
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
    if not all(math.isfinite(value) for value in values):
        return 0

    score = 0
    if -90.0 <= lat <= 90.0 and abs(lat) > 0.000001:
        score += 2
    if -180.0 <= lon <= 180.0 and abs(lon) > 0.000001:
        score += 2
    if -1000.0 <= height <= 10000.0:
        score += 1
    if all(abs(value) < 150.0 for value in (vx, vy, vz)):
        score += 1
    if -180.0 <= roll <= 180.0:
        score += 1
    if -90.0 <= pitch <= 90.0:
        score += 1
    if -360.0 <= yaw <= 360.0:
        score += 1
    return score


class IsroP2Node(Node):
    def __init__(self):
        super().__init__("isro_p2_node")

        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baudrate", 460800)
        self.declare_parameter("timeout", 0.05)
        self.declare_parameter("read_size", 256)
        self.declare_parameter("poll_hz", 100.0)
        self.declare_parameter("frame_id", "isro_p2_link")
        self.declare_parameter("gps_frame_id", "gps")
        self.declare_parameter("send_config", False)
        self.declare_parameter("strict_crc", True)
        self.declare_parameter("log_crc_warnings", False)
        self.declare_parameter("log_frame_debug", False)
        self.declare_parameter("max_frames_per_poll", 20)

        self.port = self.get_parameter("port").value
        self.baudrate = int(self.get_parameter("baudrate").value)
        self.timeout = float(self.get_parameter("timeout").value)
        self.read_size = int(self.get_parameter("read_size").value)
        self.poll_hz = float(self.get_parameter("poll_hz").value)
        self.frame_id = self.get_parameter("frame_id").value
        self.gps_frame_id = self.get_parameter("gps_frame_id").value
        self.strict_crc = bool(self.get_parameter("strict_crc").value)
        self.max_frames_per_poll = int(self.get_parameter("max_frames_per_poll").value)

        self.buffer = bytearray()

        self.raw_pub = self.create_publisher(String, "pva/raw", 10)
        self.fix_pub = self.create_publisher(NavSatFix, "pva/fix", 10)
        self.velocity_pub = self.create_publisher(TwistStamped, "pva/velocity", 10)
        self.pva_imu_pub = self.create_publisher(Imu, "pva/imu", 10)

        self.serial_port = self._open_serial()
        if bool(self.get_parameter("send_config").value):
            self._send_config(Path(self.get_parameter("config_path").value))

        timer_period = 1.0 / self.poll_hz if self.poll_hz > 0.0 else 0.01
        self.timer = self.create_timer(timer_period, self.read_once)

        self.get_logger().info(
            f"isro_p2_node started: port={self.port}, baudrate={self.baudrate}, "
            f"timeout={self.timeout}, read_size={self.read_size}, poll_hz={self.poll_hz}"
        )

    def _open_serial(self):
        if serial is None:
            raise RuntimeError(
                "python3-serial is not installed. Install it with: sudo apt install python3-serial"
            )

        try:
            ser = serial.Serial(self.port, self.baudrate, timeout=self.timeout)
        except serial.SerialException as exc:
            raise RuntimeError(f"Failed to open serial port {self.port}: {exc}") from exc

        time.sleep(0.5)
        self.get_logger().info("ISRO-P2 serial port opened")
        return ser

    def _send_config(self, config_path: Path) -> None:
        lines = [
            line.strip()
            for line in config_path.read_text(encoding="utf-8").splitlines()
            if line.strip() and not line.lstrip().startswith("#")
        ]
        for line in lines:
            self.serial_port.write(line.encode("ascii") + b"\r\n")
            self.serial_port.flush()
            self.get_logger().info(f"CONFIG > {line}")
            time.sleep(0.1)
        time.sleep(0.5)

    def read_once(self):
        try:
            chunk = self.serial_port.read(self.read_size)
        except serial.SerialException as exc:
            self.get_logger().error(f"Serial read failed: {exc}")
            return

        if chunk:
            self.buffer.extend(chunk)

        handled = 0
        while handled < self.max_frames_per_poll:
            sync_index = self.buffer.find(SYNC)
            if sync_index < 0:
                if len(self.buffer) > len(SYNC) - 1:
                    del self.buffer[:-(len(SYNC) - 1)]
                break
            if sync_index > 0:
                del self.buffer[:sync_index]

            frame = try_extract_frame(self.buffer)
            if frame is None:
                break

            del self.buffer[:len(frame.raw)]
            handled += 1
            self.handle_frame(frame)

    def handle_frame(self, frame: PimtpFrame) -> None:
        if bool(self.get_parameter("log_frame_debug").value):
            self.get_logger().info(
                f"frame msg={frame.msg_id} payload_len={frame.payload_len} "
                f"offsets=({frame.msg_id_offset},{frame.length_offset},{frame.payload_offset}) "
                f"next_sync={frame.next_sync_ok} crc={frame.crc_valid} method={frame.crc_method}"
            )

        if bool(self.get_parameter("log_crc_warnings").value) and not frame.crc_valid:
            self.get_logger().warn(
                f"CRC mismatch: msg={frame.msg_id} recv=0x{frame.received_crc:08X} "
                f"calc=0x{frame.calculated_crc:08X}"
            )

        if frame.msg_id != MSG_ID_PVA:
            return

        if self.strict_crc and not frame.crc_valid:
            return

        pva = parse_pva(frame.payload)
        if pva is None:
            self.get_logger().warn(f"PVA parse failed: payload_len={len(frame.payload)}")
            return

        self.publish_pva(pva)

    def publish_pva(self, pva: PvaData):
        stamp = self.get_clock().now().to_msg()
        raw = (
            f"lat={pva.latitude_deg:.9f}, lon={pva.longitude_deg:.9f}, "
            f"height={pva.height_m:.3f}, "
            f"vel=({pva.velocity_x_mps:.3f}, {pva.velocity_y_mps:.3f}, "
            f"{pva.velocity_z_mps:.3f}) m/s, speed={pva.speed_kmh:.2f} km/h, "
            f"roll={pva.roll_deg:.3f}, pitch={pva.pitch_deg:.3f}, "
            f"yaw={pva.yaw_deg:.3f}, layout={pva.layout}"
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
        qx, qy, qz, qw = self._euler_to_quaternion(
            math.radians(pva.roll_deg),
            math.radians(pva.pitch_deg),
            math.radians(pva.yaw_deg),
        )
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
        if hasattr(self, "timer"):
            self.timer.cancel()
        if hasattr(self, "serial_port") and self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
            if rclpy.ok():
                self.get_logger().info("Serial port closed")
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


if __name__ == "__main__":
    main()
