#!/usr/bin/env python3
"""하드웨어 제어 노드 (주차 미션용, 토글 없는 단순 버전).

motion_command를 받는 즉시 아두이노로 전송하고,
종료 시 정지 명령(s0r0l0)을 보낸 뒤 시리얼 포트를 닫는다.
"""
import rclpy
from rclpy.node import Node
from interfaces_pkg.msg import MotionCommand
import serial

class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')  # 노드 이름을 control_node로 명시

        # 장치가 달라도 코드 수정 없이 ROS 2 파라미터로 포트 선택
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('serial_baudrate', 9600)
        port = self.get_parameter('serial_port').value
        baud = int(self.get_parameter('serial_baudrate').value)
        try:
            self.ser = serial.Serial(port, baud)
            self.get_logger().info(f"Connected to Arduino on {port} @{baud}")
        except Exception as e:
            self.get_logger().error(f"🔥 Failed to connect to Arduino: {e}")
            rclpy.shutdown()
            return

        # MotionCommand 구독 설정
        self.create_subscription(
            MotionCommand,
            'motion_command',
            self.motion_command_callback,
            10
        )
        self.get_logger().info("✅ Control node is ready and listening for /motion_command.")

    def motion_command_callback(self, msg: MotionCommand):
        # MotionCommand 메시지를 받으면 즉시 아두이노 명령으로 변환하여 전송
        cmd = f"s{msg.steering}r{msg.right_speed}l{msg.left_speed}\n"
        try:
            self.ser.write(cmd.encode())
            # self.get_logger().info(f"Sent: {cmd.strip()}")
        except Exception as e:
            self.get_logger().error(f"🔥 Failed to write to serial port: {e}")

    def destroy_node(self):
        # 노드 종료 시 정지 명령 전송 및 시리얼 포트 닫기
        if hasattr(self, 'ser') and self.ser.is_open:
            # 정지 명령을 확실히 보냄
            stop_cmd = "s0r0l0\n"
            self.ser.write(stop_cmd.encode())
            self.ser.close()
            self.get_logger().info("🛑 Sent stop command and closed serial port.")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 노드가 종료될 때 destroy_node가 호출되도록 보장
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()