import serial
import time

# 기본 설정 - 센서가 연결된 포트와 통신 속도를 정하기
PORT = "/dev/ttyUSB0"
BAUDRATE = 115200


def main():
    # Serial 클래스로 ser라는 시리얼 통신 객체를 만든다.(센서와 연결 통로 만들기)
    ser = serial.Serial(PORT, BAUDRATE, timeout=1)  # timeout=1은 데이터를 기다릴 때 최대 1초까지만 기다리겠다는 뜻
    time.sleep(1)

    print("MW connected")
    print("Reading MW data...")
    print("Press Ctrl+C to stop")
    print("-" * 40)

    # 센서 출력 모드 설정
    # ser 통로로 센서에게 명령 보내기
    ser.reset_input_buffer()  # 센서가 명령을 처리하기 전, 이전에 남아있던 데이터를 지우기 위해 버퍼를 초기화

    ser.write(b"ss=4\r\n")
    ser.write(b"sp=100\r\n")  # b는 문자열을 바이트 데이터로 만든다는 뜻
    time.sleep(1)  # 센서가 명령을 처리할 시간을 주기 위해 1초 대기

    ser.reset_input_buffer()  # 센서가 명령을 처리한 후, 명령 직후 섞인 데이터를 지우기 위해 버퍼를 초기화

    try:
        while True:
            line = ser.readline()  # ser 통로로 센서 데이터 읽기

            if line:
                data = line.decode("utf-8", errors="ignore").strip()
                print(data)
            else:
                print("No data")

    except KeyboardInterrupt:
        print("\nStopped")

    finally:
        ser.close()
        print("Serial closed")


if __name__ == "__main__":
    main()
