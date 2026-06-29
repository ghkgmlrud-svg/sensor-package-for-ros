import math
import re
import time

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import Imu

import serial


# 센서 문자열에서 숫자만 뽑기 위한 정규식
NUMBER_RE = re.compile(r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?")


class MwAhrsX1Node(Node):
    def __init__(self):
        super().__init__("mw_ahrs_x1_node")  # ROS 노드 이름을 정하는 코드

        # =========================
        # 1. 기본 설정
        # =========================
        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("timeout", 1.0)

        # sp=100ms 이므로 센서 출력은 10Hz
        self.declare_parameter("read_timer_hz", 10.0)

        self.declare_parameter("frame_id", "imu_link")

        # 파라미터 값을 실제 변수에 저장
        self.port = self.get_parameter("port").value
        self.baudrate = int(self.get_parameter("baudrate").value)
        self.timeout = float(self.get_parameter("timeout").value)
        self.read_timer_hz = float(self.get_parameter("read_timer_hz").value)
        self.frame_id = self.get_parameter("frame_id").value

        # =========================
        # 2. Publisher 생성
        # =========================

        # 센서에서 받은 원본 문자열
        self.raw_pub = self.create_publisher(
            String,
            "/imu/raw",
            10
        )

        # ROS 표준 IMU 메시지
        self.imu_pub = self.create_publisher(
            Imu,
            "/imu/data",
            10
        )

        # =========================
        # 3. 시리얼 연결
        # =========================
        self.ser = serial.Serial(
            self.port,
            self.baudrate,
            timeout=self.timeout
        )

        time.sleep(1)

        self.get_logger().info("IMU connected")
        self.get_logger().info("Reading IMU data...")

        # =========================
        # 4. 센서 출력 모드 설정
        # =========================
        self.configure_sensor()

        # =========================
        # 5. ROS 타이머 설정
        # =========================
        timer_period = 1.0 / self.read_timer_hz

        self.timer = self.create_timer(
            timer_period,
            self.read_imu_data
        )

    def configure_sensor(self):
        """
        테스트 통과한 일반 파이썬 코드와 같은 흐름으로 센서 설정
        """

        # 이전에 남아있던 데이터 제거
        self.ser.reset_input_buffer()

        # 센서 출력 모드 설정
        self.ser.write(b"ss=4\r\n")

        # 센서 출력 주기 설정
        # sp=100ms = 0.1초마다 1번 = 10Hz
        self.ser.write(b"sp=100\r\n")

        # 센서가 명령을 처리할 시간 대기
        time.sleep(1)

        # 명령 직후 섞여 들어온 데이터 제거
        self.ser.reset_input_buffer()

        self.get_logger().info("Sensor configured: ss=4, sp=100")

    def read_imu_data(self):
        """
        센서에서 한 줄 읽고 ROS 토픽으로 발행
        """

        # 센서에서 한 줄 읽기
        line = self.ser.readline()

        # timeout 동안 아무 데이터도 못 읽으면 빈 값이 들어옴
        if not line:
            self.get_logger().debug("No data")
            return

        # 바이트 데이터를 문자열로 변환
        data = line.decode("utf-8", errors="ignore").strip()

        # 줄바꿈만 있거나 공백만 있는 데이터는 무시
        if not data:
            return

        # =========================
        # 1. 원본 문자열 publish
        # =========================
        raw_msg = String()
        raw_msg.data = data
        self.raw_pub.publish(raw_msg)

        # =========================
        # 2. 숫자 추출
        # =========================
        # /imu/data 메시지를 만들기 위해 필요함
        values = self.extract_numbers(data)

        # =========================
        # 3. sensor_msgs/Imu 메시지 publish
        # =========================
        imu_msg = self.make_imu_msg(values)

        if imu_msg is not None:
            self.imu_pub.publish(imu_msg)

    def extract_numbers(self, text):
        """
        센서 문자열에서 숫자만 뽑아서 float 리스트로 변환
        """

        values = []

        for match in NUMBER_RE.finditer(text):
            values.append(float(match.group(0)))

        return values

    def make_imu_msg(self, values):
        """
        숫자 배열을 sensor_msgs/Imu 메시지로 변환

        현재 가정:
        values[0] = roll  degree
        values[1] = pitch degree
        values[2] = yaw   degree

        values[3] = acc_x
        values[4] = acc_y
        values[5] = acc_z

        values[6] = gyro_x degree/s
        values[7] = gyro_y degree/s
        values[8] = gyro_z degree/s

        실제 ss=4 출력 순서가 다르면 이 부분은 수정해야 함.
        """

        if len(values) < 3:
            return None

        msg = Imu()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        # =========================
        # 1. 자세각 roll, pitch, yaw
        # =========================
        roll = math.radians(values[0])
        pitch = math.radians(values[1])
        yaw = math.radians(values[2])

        qx, qy, qz, qw = self.euler_to_quaternion(
            roll,
            pitch,
            yaw
        )

        msg.orientation.x = qx
        msg.orientation.y = qy
        msg.orientation.z = qz
        msg.orientation.w = qw

        # orientation 값은 사용 가능
        msg.orientation_covariance[0] = 0.0
        msg.orientation_covariance[4] = 0.0
        msg.orientation_covariance[8] = 0.0

        # =========================
        # 2. 가속도
        # =========================
        if len(values) >= 6:
            msg.linear_acceleration.x = values[3]
            msg.linear_acceleration.y = values[4]
            msg.linear_acceleration.z = values[5]

            msg.linear_acceleration_covariance[0] = 0.0
            msg.linear_acceleration_covariance[4] = 0.0
            msg.linear_acceleration_covariance[8] = 0.0
        else:
            # 가속도 값 없음
            msg.linear_acceleration_covariance[0] = -1.0

        # =========================
        # 3. 각속도
        # =========================
        if len(values) >= 9:
            # degree/s → rad/s 변환
            msg.angular_velocity.x = math.radians(values[6])
            msg.angular_velocity.y = math.radians(values[7])
            msg.angular_velocity.z = math.radians(values[8])

            msg.angular_velocity_covariance[0] = 0.0
            msg.angular_velocity_covariance[4] = 0.0
            msg.angular_velocity_covariance[8] = 0.0
        else:
            # 각속도 값 없음
            msg.angular_velocity_covariance[0] = -1.0

        return msg

    @staticmethod
    def euler_to_quaternion(roll, pitch, yaw):
        """
        roll, pitch, yaw 값을 quaternion으로 변환
        ROS Imu 메시지는 자세를 quaternion으로 넣어야 함
        """

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
        """
        노드 종료 시 시리얼 포트 닫기
        """

        if hasattr(self, "ser"):
            if self.ser is not None and self.ser.is_open:
                self.ser.close()
                self.get_logger().info("Serial closed")

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = None

    try:
        node = MwAhrsX1Node()
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        if node is not None:
            node.destroy_node()

        rclpy.shutdown()


if __name__ == "__main__":
    main()