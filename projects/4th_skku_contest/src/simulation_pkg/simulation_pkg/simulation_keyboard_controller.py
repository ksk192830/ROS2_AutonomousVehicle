#!/usr/bin/env python3

import select
import sys
import termios
import threading
import tty

import rclpy
from interfaces_pkg.msg import MotionCommand
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class SimulationKeyboardController(Node):
    def __init__(self):
        super().__init__('simulation_keyboard_controller')

        self.declare_parameter('motion_topic', '/motion_command')
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('speed_step', 15)
        self.declare_parameter('steering_step', 1)
        self.declare_parameter('max_speed', 255)
        self.declare_parameter('max_steering', 10)

        self.motion_topic = self.get_parameter('motion_topic').value
        publish_rate_hz = max(1.0, float(self.get_parameter('publish_rate_hz').value))
        self.speed_step = int(self.get_parameter('speed_step').value)
        self.steering_step = int(self.get_parameter('steering_step').value)
        self.max_speed = abs(int(self.get_parameter('max_speed').value))
        self.max_steering = abs(int(self.get_parameter('max_steering').value))

        self.steering = 0
        self.speed = 0
        self.lock = threading.Lock()

        self.pub = self.create_publisher(MotionCommand, self.motion_topic, 10)
        self.timer = self.create_timer(1.0 / publish_rate_hz, self.publish_command)

        self.keyboard_thread = threading.Thread(target=self.keyboard_loop, daemon=True)
        self.keyboard_thread.start()

        self.get_logger().info(
            'Simulation keyboard controller ready. '
            'W/S speed, A/D steering, C center, SPACE stop, Q quit.'
        )
        self.get_logger().info(
            f'Publishing MotionCommand to {self.motion_topic} at {publish_rate_hz:.1f} Hz.'
        )

    @staticmethod
    def clamp(value, lower, upper):
        return max(lower, min(value, upper))

    def keyboard_loop(self):
        if not sys.stdin.isatty():
            self.get_logger().warn(
                'No TTY stdin available. Run this node in its own terminal with ros2 run.'
            )
            return

        old_settings = termios.tcgetattr(sys.stdin.fileno())
        try:
            tty.setcbreak(sys.stdin.fileno())
            while rclpy.ok():
                readable, _, _ = select.select([sys.stdin], [], [], 0.1)
                if not readable:
                    continue

                key = sys.stdin.read(1).lower()
                if key == 'q':
                    self.stop_vehicle()
                    rclpy.shutdown()
                    return

                updated = self.apply_key(key)
                if updated:
                    self.log_state()
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

    def apply_key(self, key):
        with self.lock:
            if key == 'w':
                self.speed += self.speed_step
            elif key == 's':
                self.speed -= self.speed_step
            elif key == 'a':
                self.steering -= self.steering_step
            elif key == 'd':
                self.steering += self.steering_step
            elif key == 'c':
                self.steering = 0
            elif key == ' ':
                self.speed = 0
                self.steering = 0
            else:
                return False

            self.speed = self.clamp(self.speed, -self.max_speed, self.max_speed)
            self.steering = self.clamp(
                self.steering,
                -self.max_steering,
                self.max_steering,
            )
            return True

    def publish_command(self):
        with self.lock:
            steering = self.steering
            speed = self.speed

        msg = MotionCommand()
        msg.steering = int(steering)
        msg.left_speed = int(speed)
        msg.right_speed = int(speed)
        self.pub.publish(msg)

    def stop_vehicle(self):
        with self.lock:
            self.steering = 0
            self.speed = 0
        self.publish_command()

    def log_state(self):
        with self.lock:
            steering = self.steering
            speed = self.speed
        self.get_logger().info(f'steering={steering}, speed={speed}')


def main(args=None):
    rclpy.init(args=args)
    node = SimulationKeyboardController()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.stop_vehicle()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
