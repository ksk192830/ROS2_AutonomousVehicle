import logging
import os
import re
import shutil
import sys
import tempfile
from typing import Any


RAINBOW_COLORS = [
    '\033[31m',        # red
    '\033[38;5;208m',  # orange
    '\033[33m',        # yellow
    '\033[32m',        # green
    '\033[34m',        # blue
    '\033[38;5;54m',   # indigo
    '\033[35m',        # violet
]
RESET_COLOR = '\033[0m'


def resolve_yolo_device() -> str:
    forced = os.environ.get('E2E_YOLO_DEVICE')
    if forced:
        return forced
    return 'cuda:0' if shutil.which('nvidia-smi') else 'cpu'


def resolve_compact_logs(default: bool = True) -> bool:
    env_value = os.environ.get('E2E_COMPACT_LOGS')
    return default if env_value is None else env_value != '0'


def get_default_output(compact_logs: bool) -> Any:
    return {'stdout': 'log', 'stderr': 'log'} if compact_logs else 'screen'


def get_quiet_ros_args(compact_logs: bool) -> list[str]:
    return ['--ros-args', '--log-level', 'error'] if compact_logs else []


def configure_launch_logging(compact_logs: bool) -> None:
    if not compact_logs:
        return
    logging.getLogger().setLevel(logging.WARN)
    logging.getLogger('launch').setLevel(logging.WARN)
    logging.getLogger('launch_ros').setLevel(logging.WARN)


def _supports_color() -> bool:
    if os.environ.get('NO_COLOR'):
        return False
    if not sys.stdout.isatty():
        return False
    term = os.environ.get('TERM', '')
    return term not in ('', 'dumb')


def print_run_banner(scenario: str, yolo_device: str, compact_logs: bool) -> None:
    entries = [
        ('scenario', scenario),
        ('yolo_device', yolo_device),
        ('compact_logs', compact_logs),
    ]
    use_color = _supports_color()

    print('[run]')
    for idx, (key, value) in enumerate(entries):
        if use_color:
            color = RAINBOW_COLORS[idx % len(RAINBOW_COLORS)]
            print(f'{key} = {color}{value}{RESET_COLOR}')
        else:
            print(f'{key} = {value}')


def resolve_ros2_control_sdf(
    model_sdf_path: str,
    controllers_yaml_path: str,
    plugin_name: str = 'gazebo_ros2_control',
) -> str:
    """
    Return a runtime SDF path where <plugin name="gazebo_ros2_control"> <parameters>
    is replaced with the current machine path to controllers yaml.

    This removes hardcoded absolute paths from source SDF while keeping runtime stable.
    """
    if not os.path.exists(model_sdf_path):
        return model_sdf_path
    if not os.path.exists(controllers_yaml_path):
        return model_sdf_path

    with open(model_sdf_path, 'r', encoding='utf-8') as f:
        text = f.read()

    plugin_pattern = (
        rf'(<plugin[^>]*name=["\']{re.escape(plugin_name)}["\'][^>]*>.*?<parameters>)'
        rf'(.*?)'
        rf'(</parameters>)'
    )

    updated_text, count = re.subn(
        plugin_pattern,
        rf'\1{controllers_yaml_path}\3',
        text,
        count=1,
        flags=re.DOTALL,
    )

    if count == 0:
        # fallback: replace first parameters tag if plugin block pattern is not found
        updated_text, fallback_count = re.subn(
            r'(<parameters>)(.*?)(</parameters>)',
            rf'\1{controllers_yaml_path}\3',
            text,
            count=1,
            flags=re.DOTALL,
        )
        if fallback_count == 0:
            return model_sdf_path

    if updated_text == text:
        return model_sdf_path

    fd, tmp_path = tempfile.mkstemp(prefix='mercedes_runtime_', suffix='.sdf')
    os.close(fd)
    with open(tmp_path, 'w', encoding='utf-8') as f:
        f.write(updated_text)
    return tmp_path
