#!/usr/bin/env python3

"""Minimal Linux joystick reader that does not require evdev permissions.

SDL-based ROS joy nodes open the matching ``/dev/input/event*`` device as well
as ``/dev/input/js*``.  Some display-manager sessions only grant the former to
the greeter, so a Bluetooth controller is connected but invisible to SDL.  The
legacy joystick API is readable in that situation and contains every axis and
button needed by the exhibition game.
"""

import glob
import os
import struct
import threading
import time
from pathlib import Path
from typing import Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from exhibition_gui_pkg.gamepad_layout import GamepadLayout


EVENT = struct.Struct('IhBB')
BUTTON_EVENT = 0x01
AXIS_EVENT = 0x02
INITIAL_EVENT = 0x80


class LinuxJoyNode(Node):
    def __init__(self) -> None:
        super().__init__('race_game_controller')
        self.publisher = self.create_publisher(Joy, '/joy', 10)
        self.axes = [0.0] * 8
        self.buttons = [0] * 16
        self.initialized_axes = set()
        self.device_path: Optional[str] = None
        self.device_file = None
        self.layout = None
        self.lock = threading.Lock()
        self.running = True
        self.connected = False
        self.reader = threading.Thread(target=self._read_loop, daemon=True)
        self.reader.start()
        self.create_timer(1.0 / 50.0, self._publish)

    @staticmethod
    def _device_name(device_path: str) -> str:
        js_name = Path(device_path).name
        name_path = Path('/sys/class/input') / js_name / 'device' / 'name'
        try:
            return name_path.read_text(encoding='utf-8').strip()
        except OSError:
            return ''

    def _find_device(self) -> Optional[str]:
        devices = sorted(glob.glob('/dev/input/js*'))
        if not devices:
            return None
        for device in devices:
            if any(name in self._device_name(device).lower() for name in ('xbox', 'gamepad', 'controller')):
                return device
        return None

    def _close_device(self) -> None:
        with self.lock:
            self.connected = False
            self.axes = [0.0] * 8
            self.buttons = [0] * 16
            self.initialized_axes.clear()
        if self.device_file is not None:
            try:
                self.device_file.close()
            except OSError:
                pass
        self.device_file = None
        self.device_path = None
        self.layout = None

    def _read_loop(self) -> None:
        while self.running:
            if self.device_file is None:
                candidate = self._find_device()
                if candidate is None:
                    time.sleep(0.5)
                    continue
                try:
                    self.device_file = open(candidate, 'rb', buffering=0)
                    self.layout = GamepadLayout.read(self.device_file)
                    self.device_path = candidate
                    self.get_logger().info(
                        f'Gamepad ready: {candidate} '
                        f'({self._device_name(candidate)})'
                    )
                except (OSError, ValueError) as exc:
                    self.get_logger().warning(
                        f'Cannot open {candidate}: {exc}',
                        throttle_duration_sec=5.0,
                    )
                    self._close_device()
                    time.sleep(0.5)
                    continue

            try:
                payload = self.device_file.read(EVENT.size)
                if len(payload) != EVENT.size:
                    raise OSError('controller disconnected')
                _, value, event_type, number = EVENT.unpack(payload)
                event_type &= ~INITIAL_EVENT
                with self.lock:
                    if event_type == AXIS_EVENT:
                        while number >= len(self.axes):
                            self.axes.append(0.0)
                        self.axes[number] = max(-1.0, min(1.0, value / 32767.0))
                        self.initialized_axes.add(number)
                    elif event_type == BUTTON_EVENT:
                        while number >= len(self.buttons):
                            self.buttons.append(0)
                        self.buttons[number] = int(value != 0)
                    # Publish only after steering and both trigger initial
                    # values arrived; never leak a previous connection's input.
                    self.connected = self.layout.required.issubset(self.initialized_axes)
            except (OSError, ValueError) as exc:
                self.get_logger().warning(f'Gamepad lost: {exc}')
                self._close_device()
                time.sleep(0.5)

    def _publish(self) -> None:
        with self.lock:
            if not self.connected:
                return
            raw_axes = list(self.axes)
            buttons = list(self.buttons)
            axes, buttons = self.layout.normalize(raw_axes, buttons)
        message = Joy()
        message.header.stamp = self.get_clock().now().to_msg()
        message.axes = axes
        message.buttons = buttons
        self.publisher.publish(message)

    def destroy_node(self) -> bool:
        self.running = False
        self._close_device()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LinuxJoyNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
