import math
import struct
import time

from geometry_msgs.msg import TwistStamped
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus
from std_msgs.msg import String

try:
    import serial
except ImportError:  # pragma: no cover
    serial = None


ISRO_SYNC = b"\xAC\x55\x96\x83"
ISRO_MSG_ID_PVA = 2379
ISRO_MSG_ID_IMU = 2389
ISRO_PREFERRED_MSG_ID_OFFSET = 12
ISRO_PREFERRED_PAYLOAD_OFFSET = 28


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


def format_isro(isro):
    return (
        f"lat={isro['lat']:.9f}, lon={isro['lon']:.9f}, "
        f"h={isro['height']:.2f}m, speed={isro['speed_kmh']:.2f}km/h, "
        f"roll={isro['roll']:.2f}, pitch={isro['pitch']:.2f}, yaw={isro['yaw']:.2f}"
    )


def euler_to_quaternion(roll, pitch, yaw):
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


class FusionSensorNode(Node):
    def __init__(self):
        super().__init__("fusion_sensor_node")

        self.declare_parameter("isro_port", "/dev/ttyUSB0")
        self.declare_parameter("isro_baudrate", 460800)
        self.declare_parameter("isro_timeout", 0.05)
        self.declare_parameter("isro_read_size", 256)
        self.declare_parameter("isro_strict_crc", True)
        self.declare_parameter("isro_frame_id", "isro_p2_link")
        self.declare_parameter("isro_gps_frame_id", "gps")
        self.declare_parameter("mw_port", "/dev/ttyUSB1")
        self.declare_parameter("mw_baudrate", 115200)
        self.declare_parameter("mw_timeout", 0.05)
        self.declare_parameter("mw_frame_id", "imu_link")
        self.declare_parameter("mw_configure", True)
        self.declare_parameter("mw_config_wait", 1.0)
        self.declare_parameter("poll_hz", 100.0)
        self.declare_parameter("pair_max_age_sec", 0.5)
        self.declare_parameter("status_hz", 1.0)

        self.isro_port = self.get_parameter("isro_port").value
        self.isro_baudrate = int(self.get_parameter("isro_baudrate").value)
        self.isro_timeout = float(self.get_parameter("isro_timeout").value)
        self.isro_read_size = int(self.get_parameter("isro_read_size").value)
        self.isro_strict_crc = bool(self.get_parameter("isro_strict_crc").value)
        self.isro_frame_id = self.get_parameter("isro_frame_id").value
        self.isro_gps_frame_id = self.get_parameter("isro_gps_frame_id").value
        self.mw_port = self.get_parameter("mw_port").value
        self.mw_baudrate = int(self.get_parameter("mw_baudrate").value)
        self.mw_timeout = float(self.get_parameter("mw_timeout").value)
        self.mw_frame_id = self.get_parameter("mw_frame_id").value
        self.pair_max_age_sec = float(self.get_parameter("pair_max_age_sec").value)
        self.status_period = 1.0 / float(self.get_parameter("status_hz").value)

        self.isro_buffer = bytearray()
        self.latest_isro = None
        self.latest_mw = None
        self.latest_isro_time = None
        self.latest_mw_time = None
        self.isro_pva_count = 0
        self.mw_count = 0
        self.fusion_count = 0
        self.last_status_time = 0.0

        self.fusion_raw_pub = self.create_publisher(String, "fusion/raw", 10)
        self.fusion_status_pub = self.create_publisher(String, "fusion/status", 10)

        self.mw_compat_raw_pub = self.create_publisher(String, "/imu/raw", 10)
        self.mw_compat_imu_pub = self.create_publisher(Imu, "/imu/data", 10)

        self.pva_raw_pub = self.create_publisher(String, "pva/raw", 10)
        self.pva_fix_pub = self.create_publisher(NavSatFix, "pva/fix", 10)
        self.pva_velocity_pub = self.create_publisher(TwistStamped, "pva/velocity", 10)
        self.pva_imu_pub = self.create_publisher(Imu, "pva/imu", 10)

        self.isro_serial = self._open_serial(
            self.isro_port, self.isro_baudrate, self.isro_timeout, "ISRO-P2"
        )
        self.mw_serial = self._open_serial(
            self.mw_port, self.mw_baudrate, self.mw_timeout, "MW-AHRS-X1"
        )
        if bool(self.get_parameter("mw_configure").value):
            self._configure_mw()

        timer_period = 1.0 / float(self.get_parameter("poll_hz").value)
        self.timer = self.create_timer(timer_period, self.read_once)
        self.get_logger().info(
            f"fusion_sensor_node started: ISRO={self.isro_port}@{self.isro_baudrate}, "
            f"MW={self.mw_port}@{self.mw_baudrate}"
        )

    def _open_serial(self, port, baudrate, timeout, name):
        if serial is None:
            raise RuntimeError(
                "python3-serial is not installed. Install it with: sudo apt install python3-serial"
            )
        try:
            ser = serial.Serial(port, baudrate, timeout=timeout)
        except serial.SerialException as exc:
            raise RuntimeError(f"Failed to open {name} serial port {port}: {exc}") from exc
        time.sleep(0.5)
        self.get_logger().info(f"{name} serial port opened: {port} @ {baudrate}")
        return ser

    def _configure_mw(self):
        self.mw_serial.reset_input_buffer()
        self.mw_serial.write(b"ss=4\r\n")
        self.mw_serial.write(b"sp=100\r\n")
        self.mw_serial.flush()
        time.sleep(float(self.get_parameter("mw_config_wait").value))
        self.mw_serial.reset_input_buffer()
        self.get_logger().info("MW CONFIG > ss=4, sp=100")

    def read_once(self):
        self._read_isro()
        self._read_mw()
        self._publish_status()

    def _read_isro(self):
        try:
            chunk = self.isro_serial.read(self.isro_read_size)
        except serial.SerialException as exc:
            self.get_logger().error(f"ISRO serial read failed: {exc}")
            return

        if not chunk:
            return

        self.isro_buffer.extend(chunk)
        while True:
            sync_index = self.isro_buffer.find(ISRO_SYNC)
            if sync_index < 0:
                if len(self.isro_buffer) > len(ISRO_SYNC) - 1:
                    del self.isro_buffer[:-(len(ISRO_SYNC) - 1)]
                return
            if sync_index > 0:
                del self.isro_buffer[:sync_index]

            frame = find_isro_frame(self.isro_buffer)
            if frame is None:
                return

            del self.isro_buffer[:frame["raw_len"]]
            if frame["msg_id"] != ISRO_MSG_ID_PVA:
                continue
            if self.isro_strict_crc and not frame["crc_ok"]:
                continue

            pva = parse_isro_pva(frame["payload"])
            if pva is None:
                continue

            now = self.get_clock().now().nanoseconds * 1.0e-9
            self.latest_isro = pva
            self.latest_isro_time = now
            self.isro_pva_count += 1
            self._publish_isro_compat_topics(pva)
            self._publish_fusion_if_ready(now)

    def _read_mw(self):
        try:
            line = self.mw_serial.readline()
        except serial.SerialException as exc:
            self.get_logger().error(f"MW serial read failed: {exc}")
            return

        if not line:
            return

        printable_count = sum(1 for byte in line if byte in (9, 10, 13) or 32 <= byte <= 126)
        if printable_count / max(len(line), 1) < 0.7:
            return

        text = line.decode("utf-8", errors="ignore").strip()
        if not text:
            return

        values = parse_mw_line(text)
        if not values:
            return

        now = self.get_clock().now().nanoseconds * 1.0e-9
        self.latest_mw = {"raw": text, "values": values}
        self.latest_mw_time = now
        self.mw_count += 1
        self.mw_compat_raw_pub.publish(String(data=text))
        imu_msg = self._make_mw_imu_msg(values)
        if imu_msg is not None:
            self.mw_compat_imu_pub.publish(imu_msg)

    def _make_mw_imu_msg(self, values):
        if len(values) < 3:
            return None

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.mw_frame_id

        roll = math.radians(values[0])
        pitch = math.radians(values[1])
        yaw = math.radians(values[2])
        qx, qy, qz, qw = euler_to_quaternion(roll, pitch, yaw)
        msg.orientation.x = qx
        msg.orientation.y = qy
        msg.orientation.z = qz
        msg.orientation.w = qw
        msg.orientation_covariance[0] = 0.0
        msg.orientation_covariance[4] = 0.0
        msg.orientation_covariance[8] = 0.0

        if len(values) >= 6:
            msg.linear_acceleration.x = values[3]
            msg.linear_acceleration.y = values[4]
            msg.linear_acceleration.z = values[5]
            msg.linear_acceleration_covariance[0] = 0.0
            msg.linear_acceleration_covariance[4] = 0.0
            msg.linear_acceleration_covariance[8] = 0.0
        else:
            msg.linear_acceleration_covariance[0] = -1.0

        if len(values) >= 9:
            msg.angular_velocity.x = math.radians(values[6])
            msg.angular_velocity.y = math.radians(values[7])
            msg.angular_velocity.z = math.radians(values[8])
            msg.angular_velocity_covariance[0] = 0.0
            msg.angular_velocity_covariance[4] = 0.0
            msg.angular_velocity_covariance[8] = 0.0
        else:
            msg.angular_velocity_covariance[0] = -1.0

        return msg

    def _publish_isro_compat_topics(self, pva):
        stamp = self.get_clock().now().to_msg()
        raw = (
            f"lat={pva['lat']:.9f}, lon={pva['lon']:.9f}, "
            f"height={pva['height']:.3f}, "
            f"vel=({pva['vel_x']:.3f}, {pva['vel_y']:.3f}, {pva['vel_z']:.3f}) m/s, "
            f"speed={pva['speed_kmh']:.2f} km/h, "
            f"roll={pva['roll']:.3f}, pitch={pva['pitch']:.3f}, "
            f"yaw={pva['yaw']:.3f}, layout=automotive_pva"
        )
        self.pva_raw_pub.publish(String(data=raw))

        fix_msg = NavSatFix()
        fix_msg.header.stamp = stamp
        fix_msg.header.frame_id = self.isro_gps_frame_id
        fix_msg.status.status = NavSatStatus.STATUS_FIX
        fix_msg.status.service = NavSatStatus.SERVICE_GPS
        fix_msg.latitude = pva["lat"]
        fix_msg.longitude = pva["lon"]
        fix_msg.altitude = pva["height"]
        fix_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
        self.pva_fix_pub.publish(fix_msg)

        velocity_msg = TwistStamped()
        velocity_msg.header.stamp = stamp
        velocity_msg.header.frame_id = self.isro_frame_id
        velocity_msg.twist.linear.x = pva["vel_x"]
        velocity_msg.twist.linear.y = pva["vel_y"]
        velocity_msg.twist.linear.z = pva["vel_z"]
        self.pva_velocity_pub.publish(velocity_msg)

        imu_msg = Imu()
        imu_msg.header.stamp = stamp
        imu_msg.header.frame_id = self.isro_frame_id
        qx, qy, qz, qw = euler_to_quaternion(
            math.radians(pva["roll"]),
            math.radians(pva["pitch"]),
            math.radians(pva["yaw"]),
        )
        imu_msg.orientation.x = qx
        imu_msg.orientation.y = qy
        imu_msg.orientation.z = qz
        imu_msg.orientation.w = qw
        imu_msg.orientation_covariance[0] = 0.0
        imu_msg.angular_velocity_covariance[0] = -1.0
        imu_msg.linear_acceleration_covariance[0] = -1.0
        self.pva_imu_pub.publish(imu_msg)

    def _publish_fusion_if_ready(self, now):
        if self.latest_isro is None or self.latest_mw is None:
            return
        if self.latest_isro_time is None or self.latest_mw_time is None:
            return

        dt = abs(self.latest_isro_time - self.latest_mw_time)
        if dt > self.pair_max_age_sec:
            return

        isro = self.latest_isro
        mw_text = format_mw(self.latest_mw["values"], self.latest_mw["raw"])
        line = (
            f"t={now:.3f} dt={dt:.3f}s | "
            f"ISRO {format_isro(isro)} | "
            f"MW {mw_text}"
        )
        self.fusion_count += 1
        self.fusion_raw_pub.publish(String(data=line))

    def _publish_status(self):
        now = self.get_clock().now().nanoseconds * 1.0e-9
        if now - self.last_status_time < self.status_period:
            return
        self.last_status_time = now
        status = (
            f"isro_pva={self.isro_pva_count}, mw={self.mw_count}, fusion={self.fusion_count}, "
            f"has_isro={self.latest_isro is not None}, has_mw={self.latest_mw is not None}"
        )
        self.fusion_status_pub.publish(String(data=status))

    def destroy_node(self):
        if hasattr(self, "timer"):
            self.timer.cancel()
        for ser in (getattr(self, "isro_serial", None), getattr(self, "mw_serial", None)):
            if ser and ser.is_open:
                ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = FusionSensorNode()
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
