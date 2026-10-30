#!/usr/bin/env python3

import sys
import termios
import tty
import threading
import os
import select
import glob
import subprocess

import rclpy
from rclpy.node import Node
from interfaces_pkg.msg import MotionCommand
import serial


class ControlNode(Node):
    def __init__(self):
        super().__init__("control")

        # Serial port
        self.declare_parameter("serial_port", "auto")
        self.declare_parameter("baud_rate", 115200)
        serial_port = self._resolve_device_path(
            self.get_parameter("serial_port").get_parameter_value().string_value
        )
        baud_rate = self.get_parameter("baud_rate").get_parameter_value().integer_value
        self.ser = serial.Serial(serial_port, baud_rate, timeout=0.2)
        self.get_logger().info(
            "\n"
            "🟢🟢🟢🔌🔌🔌  ARDUINO SERIAL OK  🔌🔌🔌🟢🟢🟢\n"
            f"✅ port: {serial_port} | baud: {baud_rate}\n"
            f"✅ resolved: {os.path.realpath(serial_port)} | source: serial\n"
            "🟢🟢🟢━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━🟢🟢🟢"
        )

        # Mode: "ZERO", "MOTION", "MANUAL"
        self.mode = "ZERO"

        # Manual control variables
        self.manual_steering = 0
        self.manual_left_speed = 0
        self.manual_right_speed = 0

        # Limits for manual control
        self.max_speed = 255
        self.min_speed = -255
        self.max_steering = 10
        self.min_steering = -10
        self.speed_step = 10
        self.steering_step = 1

        # Subscription
        self.create_subscription(
            MotionCommand,
            "motion_command",
            self.motion_command_callback,
            10,
        )

        # Keyboard thread
        self._kb_stop_event = threading.Event()
        self._terminal_fd = None
        self._terminal_settings = None
        self._kb_thread = threading.Thread(target=self._keyboard_loop, daemon=True)
        self._kb_thread.start()

        # Forward Arduino Serial output to the ROS terminal log.
        self._serial_stop_event = threading.Event()
        self._serial_thread = threading.Thread(
            target=self._serial_read_loop,
            daemon=True,
        )
        self._serial_thread.start()

        self.get_logger().info(
            "Space: change mode (ZERO → MOTION → MANUAL → ...), "
            "Manual mode: w/s (+/- speed), a/d (steer), f (reset), "
            "p (steering calibration)."
        )

    @staticmethod
    def _resolve_device_path(port: str) -> str:
        if port.lower() == "auto":
            return ControlNode._find_arduino_port()
        if os.path.isabs(port):
            resolved = port
        else:
            resolved = os.path.join(os.sep, 'dev', port)
        ControlNode._reject_lidar_port(resolved)
        return resolved

    @staticmethod
    def _find_arduino_port() -> str:
        by_id_candidates = sorted(glob.glob("/dev/serial/by-id/*"))
        for path in by_id_candidates:
            if ControlNode._is_lidar_port(path):
                continue
            if ControlNode._is_arduino_port(path):
                return path

        acm_candidates = sorted(glob.glob("/dev/ttyACM*"))
        for path in acm_candidates:
            if not ControlNode._is_lidar_port(path):
                return path

        usb_candidates = sorted(glob.glob("/dev/ttyUSB*"))
        for path in usb_candidates:
            if ControlNode._is_lidar_port(path):
                continue
            if ControlNode._is_arduino_port(path):
                return path

        raise serial.SerialException(
            "Arduino serial port not found. "
            "Refusing to use lidar-like /dev/ttyUSB* ports. "
            "Set serial_port:=/dev/ttyACM* or check Arduino USB connection."
        )

    @staticmethod
    def _udev_properties(path: str) -> str:
        try:
            completed = subprocess.run(
                ["udevadm", "info", "-q", "property", "-n", path],
                stdout=subprocess.PIPE,
                stderr=subprocess.DEVNULL,
                text=True,
                timeout=1.0,
                check=False,
            )
            return completed.stdout
        except (FileNotFoundError, subprocess.TimeoutExpired):
            return ""

    @staticmethod
    def _device_label(path: str) -> str:
        real_path = os.path.realpath(path)
        aliases = []
        for pattern in ("/dev/rplidar", "/dev/serial/by-id/*", "/dev/serial/by-path/*"):
            for candidate in glob.glob(pattern):
                if os.path.realpath(candidate) == real_path:
                    aliases.append(candidate)
        parts = [path, real_path, *aliases, ControlNode._udev_properties(path)]
        return " ".join(parts).lower()

    @staticmethod
    def _is_lidar_port(path: str) -> bool:
        label = ControlNode._device_label(path)
        return any(
            marker in label
            for marker in (
                "/dev/rplidar",
                "rplidar",
                "cp210",
                "silicon_labs",
                "silicon labs",
            )
        )

    @staticmethod
    def _is_arduino_port(path: str) -> bool:
        label = ControlNode._device_label(path)
        return any(
            marker in label
            for marker in (
                "arduino",
                "mega",
                "uno",
                "ch340",
                "qinheng",
                "wch",
            )
        )

    @staticmethod
    def _reject_lidar_port(path: str) -> None:
        name = os.path.basename(path)
        unknown_usb_serial = name.startswith("ttyUSB") and not ControlNode._is_arduino_port(path)
        if ControlNode._is_lidar_port(path) or unknown_usb_serial:
            raise serial.SerialException(
                f"Refusing to open lidar or unknown USB serial port as Arduino: {path}. "
                "Use a confirmed Arduino port, usually /dev/ttyACM*."
            )

    @staticmethod
    def _normalize_key(ch: str) -> str:
        """Map uppercase and Korean 2-set keyboard input to control keys."""
        key_map = {
            "ㅈ": "w",
            "ㅉ": "w",
            "ㅁ": "a",
            "ㄴ": "s",
            "ㅇ": "d",
            "ㄹ": "f",
            "ㅔ": "p",
            "ㅖ": "p",
        }
        return key_map.get(ch, ch.lower())

    def motion_command_callback(self, msg: MotionCommand):
        """Handle MotionCommand messages depending on the current mode."""
        if self.mode == "MOTION":
            cmd = f"s{msg.steering}l{msg.right_speed}r{msg.left_speed}\n"
            self.ser.write(cmd.encode())
            self.get_logger().info(f"[MOTION] Sent: {cmd.strip()}")

        elif self.mode == "ZERO":
            cmd = "s0l0r0\n"
            self.ser.write(cmd.encode())
            self.get_logger().info(f"[ZERO] Sent: {cmd.strip()}")

        elif self.mode == "MANUAL":
            # Ignore topic messages in manual mode
            return

    def _keyboard_loop(self):
        """Read keyboard input: space for mode, WASD/Z for manual control."""
        fd = sys.stdin.fileno()
        if not os.isatty(fd):
            self.get_logger().warning(
                "Keyboard control disabled because stdin is not a terminal"
            )
            return

        old = termios.tcgetattr(fd)
        self._terminal_fd = fd
        self._terminal_settings = old

        try:
            tty.setcbreak(fd)
            while rclpy.ok() and not self._kb_stop_event.is_set():
                readable, _, _ = select.select([sys.stdin], [], [], 0.1)
                if not readable:
                    continue
                ch = self._normalize_key(sys.stdin.read(1))

                # Mode change with spacebar
                if ch == " ":
                    self._cycle_mode()
                    # Clear serial buffers when changing mode
                    self.ser.reset_input_buffer()
                    self.ser.reset_output_buffer()
                    continue

                # Manual control keys are only active in MANUAL mode
                if self.mode != "MANUAL":
                    continue

                updated = False

                if ch == "w":
                    # Increase left/right speed
                    self.manual_left_speed += self.speed_step
                    self.manual_right_speed += self.speed_step
                    updated = True

                elif ch == "s":
                    # Decrease left/right speed
                    self.manual_left_speed -= self.speed_step
                    self.manual_right_speed -= self.speed_step
                    updated = True

                elif ch == "a":
                    # Decrease steering
                    self.manual_steering -= self.steering_step
                    updated = True

                elif ch == "d":
                    # Increase steering
                    self.manual_steering += self.steering_step
                    updated = True

                elif ch == "f":
                    # Reset all to zero
                    self.manual_steering = 0
                    self.manual_left_speed = 0
                    self.manual_right_speed = 0
                    updated = True

                elif ch == "p":
                    self.manual_steering = 0
                    self.manual_left_speed = 0
                    self.manual_right_speed = 0
                    self.ser.write(b"calibrate\n")
                    self.get_logger().info(
                        "Steering calibration requested. Keep clear of the wheels."
                    )
                    continue

                if updated:
                    # Clamp values
                    self.manual_left_speed = max(
                        self.min_speed, min(self.max_speed, self.manual_left_speed)
                    )
                    self.manual_right_speed = max(
                        self.min_speed, min(self.max_speed, self.manual_right_speed)
                    )
                    self.manual_steering = max(
                        self.min_steering, min(self.max_steering, self.manual_steering)
                    )

                    cmd = (
                        f"s{self.manual_steering}"
                        f"l{self.manual_left_speed}"
                        f"r{self.manual_right_speed}\n"
                    )
                    self.ser.write(cmd.encode())
                    self.get_logger().info(
                        f"Sent: {cmd.strip()}"
                    )

        finally:
            self._restore_terminal()

    def _serial_read_loop(self):
        """Forward newline-terminated Arduino messages to the ROS logger."""
        while rclpy.ok() and not self._serial_stop_event.is_set():
            try:
                raw = self.ser.readline()
                if not raw:
                    continue

                message = raw.decode("utf-8", errors="replace").strip()
                if message:
                    self.get_logger().info(f"[ARDUINO] {message}")
            except serial.SerialException as exc:
                if not self._serial_stop_event.is_set():
                    self.get_logger().error(f"Arduino serial read failed: {exc}")
                break

    def _restore_terminal(self):
        """Restore terminal echo/canonical mode; safe to call more than once."""
        if self._terminal_fd is None or self._terminal_settings is None:
            return
        try:
            termios.tcsetattr(
                self._terminal_fd,
                termios.TCSANOW,
                self._terminal_settings,
            )
        except termios.error as exc:
            self.get_logger().warning(f"Failed to restore terminal settings: {exc}")

    def stop_keyboard(self):
        """Stop keyboard input and restore the terminal before process exit."""
        self._kb_stop_event.set()
        if self._kb_thread.is_alive():
            self._kb_thread.join(timeout=1.0)
        # Fallback restoration in case the keyboard thread ended unexpectedly.
        self._restore_terminal()

    def stop_serial_reader(self):
        """Stop the Arduino serial log forwarding thread."""
        self._serial_stop_event.set()
        if self._serial_thread.is_alive():
            self._serial_thread.join(timeout=1.0)

    def _cycle_mode(self):
        """Cycle mode: ZERO -> MOTION -> MANUAL -> ZERO."""
        if self.mode == "ZERO":
            self.mode = "MOTION"
        elif self.mode == "MOTION":
            self.mode = "MANUAL"
            # Reset manual values when entering MANUAL mode
            self.manual_steering = 0
            self.manual_left_speed = 0
            self.manual_right_speed = 0
        else:
            self.mode = "MOTION"

        self.get_logger().info(f"Mode changed to: {self.mode}")

        # When entering ZERO mode, send immediate zero command
        if self.mode == "ZERO":
            cmd = "s0l0r0\n"
            self.ser.write(cmd.encode())
            self.get_logger().info(f"[ZERO] Sent: {cmd.strip()} (mode change)")

        # When entering MANUAL mode, send initial zero command too
        if self.mode == "MANUAL":
            cmd = "s0l0r0\n"
            self.ser.write(cmd.encode())
            self.get_logger().info(f"[MANUAL] Sent: {cmd.strip()} (mode change)")

def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_keyboard()
        node.stop_serial_reader()
        node.ser.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
