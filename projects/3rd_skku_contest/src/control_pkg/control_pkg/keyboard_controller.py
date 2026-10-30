"""키보드 수동 주행 + 학습 데이터 수집 도구.

w/a/s/d로 차량을 직접 조종하면서 'e' 키로 웹캠 프레임을 저장한다.
YOLO 세그멘테이션 학습용 차선 이미지(lib/lane2)를 트랙에서 수집할 때 사용했다.
ROS 토픽을 거치지 않고 아두이노 시리얼에 직접 명령을 보낸다.
"""
import rclpy
from rclpy.node import Node
import sys
import tty
import termios
import serial
import cv2
import os
import re

class KeyboardController(Node):
    def __init__(self):
        super().__init__('keyboard_controller')

        # --- Serial setup ---
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('serial_baudrate', 9600)
        port = self.get_parameter('serial_port').value
        baud = int(self.get_parameter('serial_baudrate').value)
        self.ser = serial.Serial(port, baud)

        # --- Webcam setup ---
        self.declare_parameter('camera_index', 2)
        self.cap = cv2.VideoCapture(int(self.get_parameter('camera_index').value))
        if not self.cap.isOpened():
            self.get_logger().error("웹캠을 열 수 없습니다.")
            sys.exit(1)

        # --- 저장 디렉토리 설정 ---
        # 현재 스크립트 파일의 위치를 기준으로 상대경로 설정
        default_capture_dir = os.path.join(os.path.expanduser('~'), 'ros2_av_captures', '3rd_skku')
        self.declare_parameter('capture_dir', default_capture_dir)
        self.save_dir = os.path.expanduser(self.get_parameter('capture_dir').value)
        os.makedirs(self.save_dir, exist_ok=True)

        # --- 이미지 인덱스 초기화 ---
        existing = os.listdir(self.save_dir)
        pattern = re.compile(r'image(\d+)\.jpg')
        nums = [int(m.group(1)) for f in existing if (m := pattern.match(f))]
        self.image_index = max(nums) + 1 if nums else 1

        # --- 제어 변수 ---
        self.angle = 0
        self.speed = 0
        self.max_angle = 10
        self.min_angle = -10
        self.max_speed = 255
        self.min_speed = -255
        self.step_speed = 20

        self.get_logger().info(
            "Keyboard control started. Use w/a/s/d keys, 'e' to snap, Ctrl+C to quit."
        )

        try:
            while True:
                key = self.get_key()

                # --- image capture ---
                if key == 'e':
                    ret, frame = self.cap.read()
                    if ret:
                        fname = os.path.join(self.save_dir, f'image{self.image_index}.jpg')
                        cv2.imwrite(fname, frame)
                        self.get_logger().info(f"Saved image → {fname}")
                        self.image_index += 1
                    else:
                        self.get_logger().warn("이미지 캡처 실패")
                    continue

                # --- speed/angle control ---
                if key == 'w':
                    self.speed += self.step_speed
                elif key == 's':
                    if self.speed <= 0 and self.angle == 0:
                        self.speed -= self.step_speed
                    else:
                        self.speed = 0
                        self.angle = 0
                elif key == 'a':
                    self.angle -= 1
                elif key == 'd':
                    self.angle += 1
                elif key == 'q':
                    self.get_logger().info("Quit key pressed. Exiting.")
                    break
                else:
                    continue

                # 값 제한
                self.angle = max(self.min_angle, min(self.angle, self.max_angle))
                self.speed = max(self.min_speed, min(self.speed, self.max_speed))

                # 동일한 좌우 속도 설정
                command = f"s{self.angle}l{self.speed}r{self.speed}\n"
                self.ser.write(command.encode())
                self.get_logger().info(f"Sent: {command.strip()}")

        except KeyboardInterrupt:
            self.get_logger().info("Keyboard control stopped.")
        finally:
            # cleanup
            self.ser.close()
            self.cap.release()
            cv2.destroyAllWindows()

    def get_key(self):
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)
        return ch


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardController()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
