#!/usr/bin/env python3
"""Check lidar, Arduino, and camera device connections.

This script is intentionally standalone so it can be run before ROS launch:

    python3 convenient/check_devices.py

Optional:

    python3 convenient/check_devices.py --arduino-open
"""

from __future__ import annotations

import argparse
import glob
import os
import stat
import subprocess
import sys
import time
from dataclasses import dataclass
from typing import Iterable


RPLIDAR_HEALTH_CMD = bytes([0xA5, 0x52])
RPLIDAR_STOP_CMD = bytes([0xA5, 0x25])
RPLIDAR_SCAN_CMD = bytes([0xA5, 0x20])
RPLIDAR_FORCE_SCAN_CMD = bytes([0xA5, 0x21])
RPLIDAR_SYNC = bytes([0xA5, 0x5A])
RPLIDAR_SCAN_RESPONSE_TYPE = 0x81
RPLIDAR_START_MOTOR_CMD = bytes([0xA5, 0xF0, 0x02, 0x94, 0x02, 0xC1])
RPLIDAR_STOP_MOTOR_CMD = bytes([0xA5, 0xF0, 0x02, 0x00, 0x00, 0x57])
RPLIDAR_SCAN_PROBES = (
    ("normal", RPLIDAR_SCAN_CMD),
    ("force", RPLIDAR_FORCE_SCAN_CMD),
)


@dataclass
class CheckResult:
    name: str
    ok: bool
    details: list[str]
    warn: bool = False
    headline: str | None = None


DEVICE_ICONS = {
    "LIDAR": "📡",
    "ARDUINO": "🔌",
    "CAMERAS": "📷",
}

STATUS_TEXT = {
    "OK": ("✅", "정상"),
    "WARN": ("⚠️", "주의"),
    "FAIL": ("❌", "실패"),
}


def run_cmd(args: list[str], timeout: float = 2.0) -> str:
    try:
        completed = subprocess.run(
            args,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            timeout=timeout,
            check=False,
        )
        return completed.stdout.strip()
    except (FileNotFoundError, subprocess.TimeoutExpired) as exc:
        return f"{type(exc).__name__}: {exc}"


def realpath(path: str) -> str:
    try:
        return os.path.realpath(path)
    except OSError:
        return path


def mode_text(path: str) -> str:
    try:
        st = os.stat(path)
    except OSError as exc:
        return f"stat failed: {exc}"
    return stat.filemode(st.st_mode)


def is_busy(path: str) -> tuple[bool, str]:
    out = run_cmd(["fuser", "-v", path], timeout=1.5)
    return bool(out), out


def serial_by_id() -> list[str]:
    return sorted(glob.glob("/dev/serial/by-id/*"))


def find_lidar_port() -> str | None:
    candidates = ["/dev/rplidar"]
    candidates.extend(serial_by_id())
    candidates.extend(sorted(glob.glob("/dev/ttyUSB*")))

    for path in candidates:
        base = os.path.basename(path).lower()
        target = realpath(path)
        label = f"{path} {target}".lower()
        if "rplidar" in label or "cp210" in label or "silicon" in label:
            return path

    for path in sorted(glob.glob("/dev/ttyUSB*")):
        return path
    return None


def find_arduino_port() -> str | None:
    candidates = []
    candidates.extend(serial_by_id())
    candidates.extend(sorted(glob.glob("/dev/ttyACM*")))
    candidates.extend(sorted(glob.glob("/dev/ttyUSB*")))

    for path in candidates:
        label = f"{path} {realpath(path)}".lower()
        if "arduino" in label or "mega" in label:
            return path

    acm = sorted(glob.glob("/dev/ttyACM*"))
    return acm[0] if acm else None


def read_serial_payload(ser, size: int, timeout: float) -> bytes:
    deadline = time.monotonic() + timeout
    payload = bytearray()
    while len(payload) < size and time.monotonic() < deadline:
        waiting = ser.in_waiting
        if waiting:
            payload.extend(ser.read(min(waiting, size - len(payload))))
            continue
        chunk = ser.read(size - len(payload))
        if chunk:
            payload.extend(chunk)
            continue
        time.sleep(0.05)
    return bytes(payload)


def probe_lidar_scan(serial_module, target: str) -> tuple[str | None, list[str]]:
    details: list[str] = []
    for scan_mode, scan_cmd in RPLIDAR_SCAN_PROBES:
        scan_descriptor = b""
        scan_payload = b""
        try:
            with serial_module.Serial(target, 115200, timeout=0.2) as ser:
                try:
                    ser.setDTR(False)
                    ser.reset_input_buffer()
                    ser.write(RPLIDAR_START_MOTOR_CMD)
                    time.sleep(1.0)
                    ser.reset_input_buffer()
                    ser.write(scan_cmd)
                    scan_descriptor = ser.read(7)
                    scan_payload = read_serial_payload(ser, 25, timeout=2.0)
                finally:
                    ser.write(RPLIDAR_STOP_CMD)
                    ser.write(RPLIDAR_STOP_MOTOR_CMD)
                    ser.setDTR(True)
        except Exception as exc:
            details.append(f"{scan_mode} scan probe failed: {type(exc).__name__}: {exc}")
            continue

        details.append(f"{scan_mode} scan descriptor: {scan_descriptor.hex(' ')}")
        details.append(f"{scan_mode} scan payload sample: {scan_payload.hex(' ')}")
        scan_descriptor_ok = (
            scan_descriptor.startswith(RPLIDAR_SYNC)
            and len(scan_descriptor) == 7
            and scan_descriptor[-1] == RPLIDAR_SCAN_RESPONSE_TYPE
        )
        if scan_descriptor_ok and len(scan_payload) >= 5:
            return scan_mode, details

    return None, details


def check_lidar() -> CheckResult:
    details: list[str] = []
    port = find_lidar_port()
    if not port:
        return CheckResult(
            "LIDAR",
            False,
            [
                "❌ connection: no lidar port found",
                "No /dev/rplidar, CP210x, or /dev/ttyUSB* found",
            ],
            headline="LIDAR CONNECTION FAILED",
        )

    target = realpath(port)
    details.append(f"port: {port} -> {target}")
    details.append(f"mode: {mode_text(target)}")
    busy, busy_out = is_busy(target)
    if busy:
        details.append("busy: yes")
        details.append(busy_out)
        return CheckResult(
            "LIDAR",
            False,
            ["❌ connection: lidar port is busy", *details],
            headline="LIDAR CONNECTION FAILED",
        )
    details.append("busy: no")

    try:
        import serial
    except ImportError:
        details.append("pyserial is not installed; skipped active health probe")
        return CheckResult("LIDAR", True, details, warn=True)

    try:
        with serial.Serial(target, 115200, timeout=1) as ser:
            try:
                ser.setDTR(False)
                ser.reset_input_buffer()
                ser.write(RPLIDAR_STOP_CMD)
                ser.write(RPLIDAR_STOP_MOTOR_CMD)
                time.sleep(0.1)
                ser.reset_input_buffer()
                ser.write(RPLIDAR_HEALTH_CMD)
                descriptor = ser.read(7)
                payload = ser.read(3)
            finally:
                ser.write(RPLIDAR_STOP_CMD)
        details.append(f"health descriptor: {descriptor.hex(' ')}")
        details.append(f"health payload: {payload.hex(' ')}")
        if not descriptor.startswith(RPLIDAR_SYNC):
            return CheckResult(
                "LIDAR",
                False,
                [
                    "❌ scan: port opened, but RPLidar did not return a valid health descriptor",
                    *details,
                    "bad RPLidar descriptor",
                ],
                headline="LIDAR SCAN NOT OK",
            )
        if len(payload) != 3:
            return CheckResult(
                "LIDAR",
                False,
                [
                    "❌ scan: port opened, but RPLidar health payload is missing",
                    *details,
                    "missing health payload",
                ],
                headline="LIDAR SCAN NOT OK",
            )
        status = payload[0]
        error_code = payload[1] | (payload[2] << 8)
        details.append(f"health: status={status}, error_code={error_code}")
        if status in (0, 1):
            scan_mode, scan_details = probe_lidar_scan(serial, target)
            details.extend(scan_details)
            if scan_mode is None:
                return CheckResult(
                    "LIDAR",
                    False,
                    [
                        "❌ scan: port opened and health is valid, but scan stream is not valid",
                        *details,
                    ],
                    headline="LIDAR SCAN NOT OK",
                )

            prefix = f"✅ scan: RPLidar health and {scan_mode} scan stream are valid"
            return CheckResult(
                "LIDAR",
                True,
                [prefix, *details],
                warn=(status == 1),
                headline="LIDAR SCAN OK" if status == 0 else "LIDAR HEALTH WARNING",
            )
        return CheckResult(
            "LIDAR",
            False,
            [
                "❌ scan: RPLidar returned an error health status",
                *details,
            ],
            headline="LIDAR SCAN NOT OK",
        )
    except Exception as exc:
        details.append(f"active probe failed: {type(exc).__name__}: {exc}")
        return CheckResult(
            "LIDAR",
            False,
            ["❌ connection: failed to open or probe lidar serial port", *details],
            headline="LIDAR CONNECTION FAILED",
        )


def check_arduino(open_port: bool) -> CheckResult:
    details: list[str] = []
    port = find_arduino_port()
    if not port:
        return CheckResult("ARDUINO", False, ["No Arduino-like /dev/serial/by-id or /dev/ttyACM* found"])

    target = realpath(port)
    details.append(f"port: {port} -> {target}")
    details.append(f"mode: {mode_text(target)}")
    busy, busy_out = is_busy(target)
    if busy:
        details.append("busy: yes")
        details.append(busy_out)
    else:
        details.append("busy: no")

    if not open_port:
        details.append("active open probe: skipped; use --arduino-open to test open()")
        return CheckResult("ARDUINO", not busy, details, warn=busy)

    try:
        import serial
    except ImportError:
        details.append("pyserial is not installed; skipped active open probe")
        return CheckResult("ARDUINO", not busy, details, warn=True)

    try:
        with serial.Serial(target, 9600, timeout=0.5):
            pass
        details.append("active open probe: ok at 9600 baud")
        return CheckResult("ARDUINO", True, details)
    except Exception as exc:
        details.append(f"active open probe failed: {type(exc).__name__}: {exc}")
        return CheckResult("ARDUINO", False, details)


def video_number(path: str) -> int:
    return int("".join(filter(str.isdigit, os.path.basename(path))) or 0)


def video_nodes() -> list[str]:
    return sorted(glob.glob("/dev/video*"), key=video_number)


def udev_name(path: str) -> str:
    out = run_cmd(["udevadm", "info", "-q", "property", "-n", path], timeout=1.5)
    for line in out.splitlines():
        if line.startswith("ID_V4L_PRODUCT="):
            return line.split("=", 1)[1]
        if line.startswith("ID_MODEL="):
            return line.split("=", 1)[1]
    return "unknown"


def read_frame(path: str) -> tuple[bool, str]:
    try:
        import cv2
    except ImportError:
        return False, "opencv-python is not installed"

    try:
        cv2.setLogLevel(0)
    except AttributeError:
        pass

    cap = cv2.VideoCapture(path)
    if not cap.isOpened():
        cap.release()
        return False, "open failed"

    ok = False
    shape = None
    for _ in range(5):
        ok, frame = cap.read()
        if ok and frame is not None:
            shape = frame.shape
            break
        time.sleep(0.05)
    cap.release()

    if not ok:
        return False, "frame read failed"
    return True, f"frame ok shape={shape}"


def check_cameras(expected_count: int) -> CheckResult:
    details: list[str] = []
    all_nodes = video_nodes()
    nodes = [path for path in all_nodes if video_number(path) >= 2]

    if not nodes:
        details.append("No external /dev/video* nodes found after /dev/video1")
        return CheckResult("CAMERAS", False, details)

    working = []
    for path in nodes:
        name = udev_name(path)
        busy, _ = is_busy(path)
        ok, probe = read_frame(path)
        state = "ok" if ok else "skip/fail"
        busy_text = "busy" if busy else "free"
        details.append(f"{path}: {state}, {busy_text}, {name}, {probe}")
        if ok:
            working.append(path)

    details.append(f"working capture nodes: {len(working)} / expected cameras: {expected_count}")
    return CheckResult("CAMERAS", len(working) >= expected_count, details)


def print_result(result: CheckResult) -> None:
    if result.ok and not result.warn:
        prefix = "OK"
    elif result.ok and result.warn:
        prefix = "WARN"
    else:
        prefix = "FAIL"
    status_icon, status_text = STATUS_TEXT[prefix]
    device_icon = DEVICE_ICONS.get(result.name, "🔎")
    headline = f" - {result.headline}" if result.headline else f" - {status_text}"
    print(f"{status_icon} [{prefix}] {device_icon} {result.name}{headline}")
    for detail in result.details:
        print(f"   • {detail}")


def main(argv: Iterable[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Check robot device ports.")
    parser.add_argument("--arduino-open", action="store_true", help="Open Arduino serial port for an active probe.")
    parser.add_argument("--expected-cameras", type=int, default=2, help="Expected number of working camera capture nodes.")
    args = parser.parse_args(list(argv) if argv is not None else None)

    results = [
        check_lidar(),
        check_arduino(args.arduino_open),
        check_cameras(args.expected_cameras),
    ]

    print("🔎 Device check summary")
    print("=======================")
    for idx, result in enumerate(results):
        if idx:
            print()
        print_result(result)

    return 0 if all(result.ok for result in results) else 1


if __name__ == "__main__":
    raise SystemExit(main())
