import math
import re
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64MultiArray, String

try:
    import serial
except ImportError:  # pragma: no cover - handled at runtime with a clear log
    serial = None


NUMBER_RE = re.compile(r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?")


class ImuNode(Node):
    def __init__(self):
        super().__init__("mw_ahrs_x1_node")

        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("timeout", 1.0)
        self.declare_parameter("output_hz", 50.0)
        self.declare_parameter("frame_id", "imu_link")
        self.declare_parameter("enter_command_mode", True)
        self.declare_parameter("command_mode_string", "+++")
        self.declare_parameter("command_mode_delay", 1.5)
        self.declare_parameter("startup_command", "ss=4")
        self.declare_parameter("startup_command_delay", 0.5)

        self.port = self.get_parameter("port").value
        self.baudrate = int(self.get_parameter("baudrate").value)
        self.timeout = float(self.get_parameter("timeout").value)
        self.output_hz = float(self.get_parameter("output_hz").value)
        self.frame_id = self.get_parameter("frame_id").value

        self.raw_pub = self.create_publisher(String, "imu/raw", 10)
        self.data_pub = self.create_publisher(Float64MultiArray, "imu/data", 10)
        self.imu_pub = self.create_publisher(Imu, "imu", 10)

        self.serial_port = self._open_serial()
        self._configure_sensor()

        timer_period = 1.0 / self.output_hz if self.output_hz > 0.0 else 0.02
        self.timer = self.create_timer(timer_period, self.read_once)

        self.get_logger().info(
            f"mw_ahrs_x1_node started: port={self.port}, baudrate={self.baudrate}, "
            f"output_hz={self.output_hz}"
        )

    def _open_serial(self):
        if serial is None:
            raise RuntimeError("python3-serial is not installed. Install it with: sudo apt install python3-serial")

        try:
            ser = serial.Serial(self.port, self.baudrate, timeout=self.timeout)
        except serial.SerialException as exc:
            raise RuntimeError(f"Failed to open serial port {self.port}: {exc}") from exc

        time.sleep(1.0)
        self.get_logger().info("IMU serial port opened")
        return ser

    def _configure_sensor(self):
        if bool(self.get_parameter("enter_command_mode").value):
            command_mode_string = self.get_parameter("command_mode_string").value
            self.serial_port.write(command_mode_string.encode("utf-8"))
            self.serial_port.flush()
            time.sleep(float(self.get_parameter("command_mode_delay").value))

        startup_command = self.get_parameter("startup_command").value
        if startup_command:
            command = startup_command
            if not command.endswith("\r\n"):
                command += "\r\n"
            self.serial_port.write(command.encode("utf-8"))
            self.serial_port.flush()
            time.sleep(float(self.get_parameter("startup_command_delay").value))
            self.get_logger().info(f"Sent startup command: {startup_command}")

    def read_once(self):
        line = self.serial_port.readline()
        if not line:
            self.get_logger().debug("No IMU data received")
            return

        text = line.decode("utf-8", errors="ignore").strip()
        if not text:
            return

        self.raw_pub.publish(String(data=text))

        values = [float(match.group(0)) for match in NUMBER_RE.finditer(text)]
        if values:
            self.data_pub.publish(Float64MultiArray(data=values))
            imu_msg = self._to_imu_msg(values)
            if imu_msg is not None:
                self.imu_pub.publish(imu_msg)

    def _to_imu_msg(self, values):
        if len(values) < 3:
            return None

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        roll, pitch, yaw = [math.radians(value) for value in values[0:3]]
        qx, qy, qz, qw = self._euler_to_quaternion(roll, pitch, yaw)
        msg.orientation.x = qx
        msg.orientation.y = qy
        msg.orientation.z = qz
        msg.orientation.w = qw

        msg.orientation_covariance[0] = 0.0
        msg.angular_velocity_covariance[0] = -1.0
        msg.linear_acceleration_covariance[0] = -1.0

        if len(values) >= 6:
            msg.linear_acceleration.x = values[3]
            msg.linear_acceleration.y = values[4]
            msg.linear_acceleration.z = values[5]
            msg.linear_acceleration_covariance[0] = 0.0

        if len(values) >= 9:
            msg.angular_velocity.x = math.radians(values[6])
            msg.angular_velocity.y = math.radians(values[7])
            msg.angular_velocity.z = math.radians(values[8])
            msg.angular_velocity_covariance[0] = 0.0

        return msg

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
        if hasattr(self, "serial_port") and self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
            self.get_logger().info("Serial port closed")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ImuNode()
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
        rclpy.shutdown()


if __name__ == "__main__":
    main()
