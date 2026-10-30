import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from interfaces_pkg.msg import MotionCommand


class MotionCommandRos2ControlBridge(Node):
    """
    /motion_command(s,l,r) -> ros2_control command topics

    - steering(s): front_steer_controller (position, rad)
    - left/right wheel speed(l,r): rear_wheel_controller (velocity, rad/s)

    Project rule mapping:
    - steering negative => left turn, positive => right turn
    """

    def __init__(self):
        super().__init__('motion_command_ros2control_bridge')

        self.declare_parameter('motion_topic', '/motion_command')
        self.declare_parameter('front_steer_topic', '/front_steer_controller/commands')
        self.declare_parameter('rear_wheel_topic', '/rear_wheel_controller/commands')

        # Measured/confirmed values
        self.declare_parameter('max_steer_rad', 0.4394)            # 25.18 deg
        self.declare_parameter('max_speed_mps', 1.2345)             # 255 입력 시 최대속도
        self.declare_parameter('wheel_radius_m', 0.0975)           # diameter 0.195 m
        self.declare_parameter('wheel_base_m', 0.60814)            # SDF wheel centers
        self.declare_parameter('front_track_m', 0.487)
        self.declare_parameter('ackermann_front_steering', True)
        self.declare_parameter('rear_track_m', 0.487)
        self.declare_parameter('ackermann_rear_speed_split', True)

        # Input ranges
        self.declare_parameter('steering_input_max_abs', 10.0)     # s in [-10,10]
        self.declare_parameter('wheel_input_max_abs', 255.0)       # l/r in [-255,255]

        # Sign knobs for quick tuning without touching SDF/URDF
        self.declare_parameter('steer_sign', -1.0)                 # -10 (left) -> +rad (CCW)
        # Positive speed input should mean forward motion.
        # With current wheel joint axes (0 -1 0), default signs must be -1.
        self.declare_parameter('left_wheel_sign', -1.0)
        self.declare_parameter('right_wheel_sign', -1.0)

        self.motion_topic = self.get_parameter('motion_topic').value
        self.front_steer_topic = self.get_parameter('front_steer_topic').value
        self.rear_wheel_topic = self.get_parameter('rear_wheel_topic').value

        self.max_steer_rad = abs(float(self.get_parameter('max_steer_rad').value))
        self.max_speed_mps = abs(float(self.get_parameter('max_speed_mps').value))
        self.wheel_radius_m = max(1e-6, abs(float(self.get_parameter('wheel_radius_m').value)))
        self.wheel_base_m = max(1e-6, abs(float(self.get_parameter('wheel_base_m').value)))
        self.front_track_m = max(0.0, abs(float(self.get_parameter('front_track_m').value)))
        self.ackermann_front_steering = bool(self.get_parameter('ackermann_front_steering').value)
        self.rear_track_m = max(0.0, abs(float(self.get_parameter('rear_track_m').value)))
        self.ackermann_rear_speed_split = bool(self.get_parameter('ackermann_rear_speed_split').value)

        self.steering_input_max_abs = max(1e-6, abs(float(self.get_parameter('steering_input_max_abs').value)))
        self.wheel_input_max_abs = max(1e-6, abs(float(self.get_parameter('wheel_input_max_abs').value)))

        self.steer_sign = float(self.get_parameter('steer_sign').value)
        self.left_wheel_sign = float(self.get_parameter('left_wheel_sign').value)
        self.right_wheel_sign = float(self.get_parameter('right_wheel_sign').value)

        self.sub = self.create_subscription(MotionCommand, self.motion_topic, self._on_motion, 20)
        self.pub_front = self.create_publisher(Float64MultiArray, self.front_steer_topic, 20)
        self.pub_rear = self.create_publisher(Float64MultiArray, self.rear_wheel_topic, 20)

        self.get_logger().info(
            f'motion_command_ros2control_bridge started: {self.motion_topic} -> '
            f'[{self.front_steer_topic}, {self.rear_wheel_topic}],'
            f'max_speed={self.max_speed_mps:.3f} m/s,'
            f'ackermann_front_steering={self.ackermann_front_steering},'
            f'ackermann_rear_speed_split={self.ackermann_rear_speed_split}'
        )

    @staticmethod
    def _clamp(v: float, lo: float, hi: float) -> float:
        return max(lo, min(hi, v))
    
    def _ackermann_front_angles(self, steer_rad: float):
        if abs(steer_rad) < 1e-6 or self.front_track_m <= 1e-6:
            return steer_rad, steer_rad
        
        turn_sign = 1.0 if steer_rad > 0.0 else -1.0
        abs_delta = abs(steer_rad)

        radius = self.wheel_base_m / max(math.tan(abs_delta), 1e-6)
        half_track = 0.5 * self.front_track_m

        inner = math.atan(self.wheel_base_m / max(radius - half_track, 1e-6))
        outer = math.atan(self.wheel_base_m / (radius + half_track))

        if turn_sign > 0.0:
            #left turn: left wheel is inner
            return inner, outer
        
        # right turn: right wheel is inner
        return -outer, -inner

    def _on_motion(self, msg: MotionCommand):
        s = float(msg.steering)
        l = float(msg.left_speed)
        r = float(msg.right_speed)

        # steering: input [-10,10] -> rad [-max_steer, +max_steer]
        s_norm = self._clamp(s / self.steering_input_max_abs, -1.0, 1.0)
        steer_rad = self.steer_sign * s_norm * self.max_steer_rad

        # wheel speed: input [-255,255] -> linear m/s [-max,+max] -> rad/s
        l_norm = self._clamp(l / self.wheel_input_max_abs, -1.0, 1.0)
        r_norm = self._clamp(r / self.wheel_input_max_abs, -1.0, 1.0)

        v_l = l_norm * self.max_speed_mps
        v_r = r_norm * self.max_speed_mps

        if self.ackermann_rear_speed_split:
            v_center = 0.5 * (v_l + v_r)
            yaw_rate = v_center * math.tan(steer_rad) / self.wheel_base_m
            rear_split = 0.5 * yaw_rate * self.rear_track_m
            v_l -= rear_split
            v_r += rear_split

        omega_l = self.left_wheel_sign * (v_l / self.wheel_radius_m)
        omega_r = self.right_wheel_sign * (v_r / self.wheel_radius_m)

        front_msg = Float64MultiArray()

        if self.ackermann_front_steering:
            steer_left, steer_right = self._ackermann_front_angles(steer_rad)
        else:
            steer_left, steer_right = steer_rad, steer_rad
        
        front_msg.data = [float(steer_left), float(steer_right)]

        rear_msg = Float64MultiArray()
        rear_msg.data = [float(omega_l), float(omega_r)]

        self.pub_front.publish(front_msg)
        self.pub_rear.publish(rear_msg)


def main(args=None):
    rclpy.init(args=args)
    node = MotionCommandRos2ControlBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
