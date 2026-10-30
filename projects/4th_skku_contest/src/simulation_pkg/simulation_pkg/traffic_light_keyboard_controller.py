import queue
import select
import subprocess
import sys
import termios
import threading
import tty

import rclpy
from gazebo_msgs.srv import SetLightProperties
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import ColorRGBA, String


class TrafficLightKeyboardController(Node):
    def __init__(self):
        super().__init__('traffic_light_keyboard_controller')

        self.declare_parameter('model_name', 'traffic_light_stand')
        self.declare_parameter('set_visual_material', True)
        self.declare_parameter('initial_state', 'red')

        self.model_name = self.get_parameter('model_name').value
        self.set_visual_material = self.get_parameter('set_visual_material').value
        self.state = 'green' if self.get_parameter('initial_state').value == 'green' else 'red'

        self.light_client = self.create_client(
            SetLightProperties,
            '/gazebo/set_light_properties',
        )
        self.command_queue = queue.Queue()
        self.initialized = False

        self.create_subscription(String, 'traffic_light_manual_state', self.topic_callback, 10)
        self.create_timer(0.1, self.process_commands)
        self.create_timer(0.5, self.ensure_initial_state)

        self.keyboard_thread = threading.Thread(target=self.keyboard_loop, daemon=True)
        self.keyboard_thread.start()

        self.get_logger().info(
            'Traffic light keyboard controller ready. Press SPACE to toggle red/green, q to quit.'
        )
        self.get_logger().info(
            'You can also publish: ros2 topic pub --once /traffic_light_manual_state std_msgs/msg/String "{data: green}"'
        )

    def topic_callback(self, msg):
        command = msg.data.strip().lower()
        if command in ('red', 'green', 'toggle'):
            self.command_queue.put(command)
        else:
            self.get_logger().warn(f'Ignoring unknown traffic light command: {msg.data}')

    def keyboard_loop(self):
        if not sys.stdin.isatty():
            self.get_logger().warn('No TTY stdin available. Use /traffic_light_manual_state topic instead.')
            return

        old_settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setcbreak(sys.stdin.fileno())
            while rclpy.ok():
                readable, _, _ = select.select([sys.stdin], [], [], 0.1)
                if not readable:
                    continue
                key = sys.stdin.read(1)
                if key == ' ':
                    self.command_queue.put('toggle')
                elif key.lower() == 'q':
                    rclpy.shutdown()
                    return
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

    def ensure_initial_state(self):
        if self.initialized:
            return
        if not self.light_client.service_is_ready():
            self.get_logger().info('Waiting for /gazebo/set_light_properties...', throttle_duration_sec=2.0)
            return
        self.initialized = True
        self.apply_state(self.state)

    def process_commands(self):
        while not self.command_queue.empty():
            command = self.command_queue.get()
            if command == 'toggle':
                command = 'green' if self.state == 'red' else 'red'
            self.apply_state(command)

    def apply_state(self, state):
        if state not in ('red', 'green'):
            return

        red_on = state == 'red'
        green_on = state == 'green'
        self.set_light('red_light_link', 'red', (1.0, 0.0, 0.0), red_on)
        self.set_light('yellow_light_link', 'yellow', (1.0, 1.0, 0.0), False)
        self.set_light('green_light_link', 'green', (0.0, 1.0, 0.0), green_on)

        if self.set_visual_material:
            self.set_visual('red_light_link', 'red', (1.0, 0.0, 0.0), red_on)
            self.set_visual('yellow_light_link', 'yellow', (1.0, 1.0, 0.0), False)
            self.set_visual('green_light_link', 'green', (0.0, 1.0, 0.0), green_on)

        self.state = state
        self.get_logger().info(f'Traffic light state: {state}')

    def set_light(self, link_name, light_name, rgb, enabled):
        request = SetLightProperties.Request()
        request.light_name = f'{self.model_name}::{link_name}::{light_name}'
        request.diffuse = self.color(rgb if enabled else (0.0, 0.0, 0.0), 1.0)
        request.attenuation_constant = 0.2 if enabled else 1.0
        request.attenuation_linear = 0.05 if enabled else 10.0
        request.attenuation_quadratic = 0.0

        if not self.light_client.service_is_ready():
            self.get_logger().warn('/gazebo/set_light_properties is not ready yet.')
            return

        future = self.light_client.call_async(request)
        future.add_done_callback(lambda done: self.log_light_result(done, request.light_name))

    def log_light_result(self, future, light_name):
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().warn(f'Failed to set {light_name}: {exc}')
            return
        if not response.success:
            self.get_logger().warn(f'Gazebo rejected {light_name}: {response.status_message}')

    def set_visual(self, link_name, visual_name, rgb, enabled):
        color = rgb if enabled else tuple(channel * 0.03 for channel in rgb)
        emissive = rgb if enabled else (0.0, 0.0, 0.0)
        scoped_link = f'{self.model_name}::{link_name}'
        scoped_visual = f'{scoped_link}::{visual_name}'
        message = (
            f'name: "{scoped_visual}" '
            f'parent_name: "{scoped_link}" '
            'type: VISUAL '
            'material { '
            f'ambient {{ r: {color[0]} g: {color[1]} b: {color[2]} a: 1.0 }} '
            f'diffuse {{ r: {color[0]} g: {color[1]} b: {color[2]} a: 1.0 }} '
            'specular { r: 1.0 g: 1.0 b: 1.0 a: 1.0 } '
            f'emissive {{ r: {emissive[0]} g: {emissive[1]} b: {emissive[2]} a: 1.0 }} '
            '}'
        )
        try:
            subprocess.run(
                ['gz', 'topic', '-p', '~/visual', '-m', message],
                check=False,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                timeout=0.5,
            )
        except (subprocess.SubprocessError, FileNotFoundError):
            self.get_logger().warn('Failed to publish Gazebo visual material update.', throttle_duration_sec=2.0)

    @staticmethod
    def color(rgb, alpha):
        msg = ColorRGBA()
        msg.r = float(rgb[0])
        msg.g = float(rgb[1])
        msg.b = float(rgb[2])
        msg.a = float(alpha)
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = TrafficLightKeyboardController()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
