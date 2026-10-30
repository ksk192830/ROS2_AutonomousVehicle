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
        self.declare_parameter('serial_port', 'ttyACM0')
        self.declare_parameter('baud_rate', 115200)
        serial_port = self._resolve_device_path(
            self.get_parameter('serial_port').get_parameter_value().string_value
        )
        baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value
        self.ser = serial.Serial(serial_port, baud_rate)

        # --- Webcam setup ---
        self.cap = cv2.VideoCapture(2)
        if not self.cap.isOpened():
            self.get_logger().error("웹캠을 열 수 없습니다.")
            sys.exit(1)

        # ➜ 해상도 강제 설정
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

        # --- 저장 디렉토리 설정 ---
        # 현재 스크립트 파일의 위치를 기준으로 상대경로 설정
        self.declare_parameter('save_dir', 'obstacle_near')
        self.save_dir = self.get_parameter('save_dir').get_parameter_value().string_value
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

    @staticmethod
    def _resolve_device_path(port: str) -> str:
        if os.path.isabs(port):
            return port
        return os.path.join(os.sep, 'dev', port)

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
