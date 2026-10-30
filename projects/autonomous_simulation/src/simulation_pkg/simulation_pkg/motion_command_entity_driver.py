import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from interfaces_pkg.msg import MotionCommand


class MotionCommandEntityDriver(Node):
    """
    motion_command -> Twist 변환 노드 (non-ackermann 경로)

    - left/right speed를 각각 반영해 선속도/회전속도 생성
    - steering 값을 추가 yaw-rate로 반영
    - 출력 토픽은 모델의 planar_move 플러그인 cmd_vel remap과 연결
    """

    def __init__(self):
        super().__init__('motion_command_entity_driver')

        self.declare_parameter('motion_topic', '/motion_command')
        self.declare_parameter('cmd_topic', '/cmd_vel_noack')
        self.declare_parameter('max_wheel_speed', 1.2)  # m/s @ 255
        self.declare_parameter('max_input', 255.0)
        self.declare_parameter('rear_track', 0.487)
        self.declare_parameter('wheel_base', 0.54)
        self.declare_parameter('max_steer_rad', 0.4394)
        self.declare_parameter('steer_weight', 1.0)

        self.motion_topic = self.get_parameter('motion_topic').value
        self.cmd_topic = self.get_parameter('cmd_topic').value
        self.max_wheel_speed = float(self.get_parameter('max_wheel_speed').value)
        self.max_input = float(self.get_parameter('max_input').value)
        self.rear_track = float(self.get_parameter('rear_track').value)
        self.wheel_base = float(self.get_parameter('wheel_base').value)
        self.max_steer_rad = float(self.get_parameter('max_steer_rad').value)
        self.steer_weight = float(self.get_parameter('steer_weight').value)

        self.pub = self.create_publisher(Twist, self.cmd_topic, 10)
        self.sub = self.create_subscription(MotionCommand, self.motion_topic, self._on_motion, 10)

        self.get_logger().info(
            f'motion_command_entity_driver started: {self.motion_topic} -> {self.cmd_topic}')

    def _map_speed(self, s: float) -> float:
        return max(-self.max_wheel_speed, min(self.max_wheel_speed, (s / self.max_input) * self.max_wheel_speed))

    def _on_motion(self, msg: MotionCommand):
        v_l = self._map_speed(float(msg.left_speed))
        v_r = self._map_speed(float(msg.right_speed))

        # 후륜 좌우 속도 기반
        v = 0.5 * (v_l + v_r)
        omega_diff = (v_r - v_l) / max(self.rear_track, 1e-6)

        # steering 추가 반영
        # 프로젝트 규칙 반영: steering 음수=좌회전, 양수=우회전
        steer_norm = max(-1.0, min(1.0, -float(msg.steering) / 10.0))
        steer_rad = steer_norm * self.max_steer_rad
        omega_steer = 0.0 if abs(v) < 1e-6 else (v * math.tan(steer_rad) / max(self.wheel_base, 1e-6))

        tw = Twist()
        tw.linear.x = float(v)
        tw.angular.z = float(omega_diff + self.steer_weight * omega_steer)
        self.pub.publish(tw)


def main(args=None):
    rclpy.init(args=args)
    node = MotionCommandEntityDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
