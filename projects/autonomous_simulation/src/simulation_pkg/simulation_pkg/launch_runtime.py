import logging
from functools import lru_cache
from pathlib import Path
import os
import re
import shutil
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET
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


def add_camera_rate_control(sensor, role, initial_rate):
    """Only exhibition runtime copies receive the optional rate controller."""
    if os.environ.get('E2E_DUEL_MODE') != '1':
        return False
    if sensor.find(f"plugin[@name='exhibition_{role}_rate']") is not None:
        return False
    from ament_index_python.packages import get_package_prefix
    library = Path(get_package_prefix('exhibition_sim_plugins')) / 'lib/libexhibition_camera_rate.so'
    plugin = ET.SubElement(sensor, 'plugin', name=f'exhibition_{role}_rate', filename=str(library))
    ET.SubElement(plugin, 'rate_topic').text = f'/exhibition/camera_rate/{role}'
    ET.SubElement(plugin, 'initial_rate').text = str(initial_rate)
    return True


@lru_cache(maxsize=1)
def probe_nvidia() -> tuple[bool, str]:
    """Check the loaded driver, not just whether its command is installed."""
    if not shutil.which('nvidia-smi'):
        return False, ''
    try:
        result = subprocess.run(
            ['nvidia-smi', '--query-gpu=name', '--format=csv,noheader'],
            capture_output=True, text=True, timeout=3, check=False,
        )
    except (OSError, subprocess.SubprocessError) as exc:
        return False, str(exc)
    return result.returncode == 0, (result.stderr or result.stdout).strip()


@lru_cache(maxsize=1)
def probe_cuda() -> tuple[bool, str]:
    """Test the inference runtime separately from NVML/OpenGL driver health.

    nvidia-smi can fail after a driver update while PyTorch CUDA still works.
    Desktop launches do not inherit a terminal's E2E_SEG_DEVICE override, so
    using the graphics probe here silently forced slow CPU inference and let
    every detection expire before the BEV stage could pair it with its image.
    Check an actual kernel in a bounded child process, without initializing
    CUDA in the ROS launch process itself.
    """
    code = (
        'import torch\n'
        'if not torch.cuda.is_available():\n'
        '    raise SystemExit(1)\n'
        'value = (torch.ones(1, device="cuda:0") * 2).item()\n'
        'raise SystemExit(0 if value == 2.0 else 1)\n'
    )
    try:
        result = subprocess.run(
            [sys.executable, '-c', code],
            capture_output=True, text=True, timeout=10, check=False,
        )
    except (OSError, subprocess.SubprocessError) as exc:
        return False, str(exc)
    return result.returncode == 0, (result.stderr or result.stdout).strip()


def resolve_graphics_environment() -> dict[str, str]:
    healthy, detail = probe_nvidia()
    display = os.environ.get('DISPLAY', '').rsplit(':', 1)[-1].split('.')[0]
    remote_display = display.isdigit() and int(display) >= 100
    vendor = os.environ.get('E2E_GLX_VENDOR') or (
        'nvidia' if healthy and not remote_display else 'mesa'
    )
    environment = {
        '__GLX_VENDOR_LIBRARY_NAME': vendor,
        '__NV_PRIME_RENDER_OFFLOAD': os.environ.get('E2E_PRIME_OFFLOAD') or (
            '1' if vendor == 'nvidia' else '0'
        ),
        'LIBGL_ALWAYS_SOFTWARE': os.environ.get('LIBGL_ALWAYS_SOFTWARE') or (
            '1' if not healthy and detail and vendor == 'mesa' else '0'
        ),
    }
    if not healthy and detail:
        print(
            '[graphics] NVIDIA 초기화 실패: ' + detail.replace('\n', ' ')
            + '\n[graphics] 현재 그래픽 설정: ' + str(environment)
            + '\n[graphics] 기본 그래픽 경로는 CPU입니다. 시뮬레이터용 OpenGL과 AI 인식용 GPU는 별도로 확인합니다.',
            flush=True,
        )
    return environment


@lru_cache(maxsize=1)
def resolve_simulator_graphics_environment(runtime_root: str = '') -> dict[str, str]:
    """Use verified, process-local GL libraries during a pending driver reboot.

    Updating NVIDIA userspace while the old kernel module is still loaded can
    break GLX even though CUDA works. An optional official Ubuntu package is
    extracted under .runtime (never installed system-wide). Only the simulator
    gets these libraries; inference and the rest of the desktop are unchanged.
    Missing files, a new kernel version, or a failed real context test simply
    retain the ordinary graphics environment.
    """
    if os.environ.get('E2E_GLX_VENDOR') or os.environ.get('LIBGL_ALWAYS_SOFTWARE') == '1':
        return {}
    try:
        version_text = Path('/proc/driver/nvidia/version').read_text()
    except OSError:
        return {}
    match = re.search(r'Kernel Module.*?\s(\d+\.\d+(?:\.\d+)?)\s', version_text)
    if not match:
        return {}
    version = match.group(1)
    root = Path(runtime_root) if runtime_root else Path(__file__).resolve().parents[3] / '.runtime'
    libraries = root / f'nvidia-{version}' / 'usr/lib/x86_64-linux-gnu'
    private_library = (libraries / f'libGLX_nvidia.so.{version}').is_file()
    if not private_library and not probe_nvidia()[0]:
        return {}
    environment = {
        '__GLX_VENDOR_LIBRARY_NAME': 'nvidia',
        '__NV_PRIME_RENDER_OFFLOAD': '0',
        'LIBGL_ALWAYS_SOFTWARE': '0',
        # The hidden sensor renderer must not wait for desktop scanout.
        '__GL_SYNC_TO_VBLANK': '0',
        'vblank_mode': '0',
    }
    if private_library:
        environment['LD_LIBRARY_PATH'] = os.pathsep.join(filter(None,
            [str(libraries), os.environ.get('LD_LIBRARY_PATH', '')]))
    code = '''import ctypes
from PyQt5.QtGui import QGuiApplication, QOpenGLContext, QOffscreenSurface, QSurfaceFormat
app = QGuiApplication([])
fmt = QSurfaceFormat(); fmt.setVersion(2, 1); fmt.setRenderableType(QSurfaceFormat.OpenGL)
context = QOpenGLContext(); context.setFormat(fmt)
surface = QOffscreenSurface(); surface.setFormat(fmt); surface.create()
if not context.create() or not context.makeCurrent(surface): raise SystemExit(1)
gl = ctypes.CDLL('libGL.so.1'); gl.glGetString.argtypes = [ctypes.c_uint]; gl.glGetString.restype = ctypes.c_char_p
renderer = gl.glGetString(7937)
if not renderer or b'NVIDIA' not in renderer: raise SystemExit(1)
print(renderer.decode(), flush=True)
context.doneCurrent()
'''
    try:
        result = subprocess.run([sys.executable, '-c', code], env={**os.environ, **environment},
            capture_output=True, text=True, timeout=8, check=False)
    except (OSError, subprocess.SubprocessError):
        return {}
    if result.returncode != 0:
        return {}
    source = f'전시 전용 GL {version}' if private_library else '시스템 GL'
    print(f'[graphics] 전시 GPU 렌더링: {result.stdout.strip()} · {source}', flush=True)
    return environment


def resolve_gazebo_model_path(project_models: str) -> str:
    paths = [project_models, *os.environ.get('GAZEBO_MODEL_PATH', '').split(os.pathsep),
             '/usr/share/gazebo-11/models']
    return os.pathsep.join(dict.fromkeys(path for path in paths if path))


def continue_after_success(next_actions: list, stage: str):
    """Do not launch controllers after a failed spawn or during shutdown."""
    def on_exit(event, context):
        if context.is_shutdown:
            return []
        if event.returncode == 0:
            return next_actions
        from launch.actions import EmitEvent, LogError
        from launch.events import Shutdown
        reason = f'{stage} failed (exit {event.returncode}); stopping exhibition.'
        return [LogError(msg=reason), EmitEvent(event=Shutdown(reason=reason))]
    return on_exit


def shutdown_on_exit(stage: str):
    def on_exit(event, context):
        if context.is_shutdown:
            return []
        from launch.actions import EmitEvent
        from launch.events import Shutdown
        return [EmitEvent(event=Shutdown(reason=f'{stage} exited ({event.returncode})'))]
    return on_exit


def resolve_seg_device() -> str:
    forced = os.environ.get('E2E_SEG_DEVICE') or os.environ.get('E2E_YOLO_DEVICE')
    if forced:
        return forced
    return 'cuda:0' if probe_cuda()[0] else 'cpu'


def resolve_yolo_device() -> str:
    """Backward-compatible alias for older launch files."""
    return resolve_seg_device()


def resolve_compact_logs(default: bool = True) -> bool:
    env_value = os.environ.get('E2E_COMPACT_LOGS')
    return default if env_value is None else env_value != '0'


def resolve_exhibition_world(
    world_path: str,
    top_width: int = 960,
    top_height: int = 720,
    top_rate: float = 20.0,
    top_noise_stddev: float = 0.0,
) -> str:
    """Return a temporary world tuned for the exhibition displays.

    gazebo_ros_state is a world plugin, so passing its library through
    ``gzserver -s`` does not activate it. Injecting it only into the runtime
    copy keeps the original track and the other project scenarios untouched.
    The same copy raises the overhead camera resolution so the full-screen race
    map is not an enlarged 480 x 360 preview.
    """
    if not os.path.exists(world_path):
        return world_path
    try:
        tree = ET.parse(world_path)
    except (ET.ParseError, OSError):
        return world_path

    root = tree.getroot()
    world = root if root.tag == 'world' else root.find('world')
    if world is None:
        return world_path
    changed = False
    if (os.environ.get('E2E_DUEL_MODE') == '1' and
            world.find("plugin[@name='exhibition_compact_state']") is None):
        from ament_index_python.packages import get_package_prefix
        library = Path(get_package_prefix('exhibition_sim_plugins')) / 'lib/libexhibition_compact_state.so'
        ET.SubElement(world, 'plugin', name='exhibition_compact_state', filename=str(library))
        changed = True
    has_state_plugin = any(
        plugin.get('filename') == 'libgazebo_ros_state.so'
        for plugin in world.findall('plugin')
    )
    if not has_state_plugin:
        ET.SubElement(
            world,
            'plugin',
            {
                'name': 'gazebo_ros_state',
                'filename': 'libgazebo_ros_state.so',
            },
        )
        changed = True

    def set_text(parent: ET.Element, tag: str, value: str) -> None:
        nonlocal changed
        child = parent.find(tag)
        if child is None:
            child = ET.SubElement(parent, tag)
        if child.text != value:
            child.text = value
            changed = True

    # The default (0) publishes every physics step, around 1000 times/sec,
    # including every decorative model/link. Match the 50 Hz driving control
    # loop so these large pose messages cannot crowd out the HD camera.
    for plugin in world.findall('plugin'):
        if plugin.get('filename') == 'libgazebo_ros_state.so':
            set_text(plugin, 'update_rate', '50')

    # model://race_track can resolve to an old ~/.gazebo/models copy, even
    # with the workspace first in GAZEBO_MODEL_PATH. Pin display resources to
    # this installation and use a unique material/texture name in OGRE.
    # Keep track.png unchanged: off-track detection uses its original geometry.
    materials = Path(world_path).resolve().parent.parent / 'models/race_track/materials'
    if (materials / 'exhibition/track_hd.material').is_file() and (materials / 'textures/track_hd.png').is_file():
        for script in world.findall('.//visual/material/script'):
            if script.findtext('name') not in ('track', 'RaceTrack/ExhibitionHD'):
                continue
            desired_uris = [(materials / 'exhibition').as_uri(), (materials / 'textures').as_uri()]
            if [uri.text for uri in script.findall('uri')] != desired_uris:
                for uri in list(script.findall('uri')):
                    script.remove(uri)
                for uri in desired_uris:
                    ET.SubElement(script, 'uri').text = uri
                changed = True
            set_text(script, 'name', 'RaceTrack/ExhibitionHD')

    for sensor in world.iter('sensor'):
        if sensor.get('name') != 'top_camera':
            continue
        changed = add_camera_rate_control(sensor, 'overview', top_rate) or changed
        set_text(sensor, 'update_rate', f'{float(top_rate):g}')
        image = sensor.find('./camera/image')
        if image is not None:
            set_text(image, 'width', str(int(top_width)))
            set_text(image, 'height', str(int(top_height)))
        noise = sensor.find('./camera/noise')
        if noise is not None:
            if float(top_noise_stddev) == 0.0:
                sensor.find('camera').remove(noise)
                changed = True
            else:
                set_text(noise, 'stddev', f'{float(top_noise_stddev):g}')
        plugin = sensor.find('plugin')
        if plugin is not None:
            set_text(plugin, 'updateRate', f'{float(top_rate):g}')

    if not changed:
        return world_path
    fd, tmp_path = tempfile.mkstemp(prefix='track_exhibition_', suffix='.world')
    os.close(fd)
    tree.write(tmp_path, encoding='utf-8', xml_declaration=False)
    return tmp_path


def get_default_output(compact_logs: bool) -> Any:
    return {'stdout': 'log', 'stderr': 'log'} if compact_logs else 'screen'


def get_quiet_ros_args(compact_logs: bool) -> list[str]:
    return ['--ros-args', '--log-level', 'error'] if compact_logs else []


def configure_launch_logging(compact_logs: bool) -> None:
    if not compact_logs:
        return
    # Compact mode routes node output to files. Keep process lifecycle and
    # stdout records at INFO so startup failures remain diagnosable there.
    logging.getLogger().setLevel(logging.INFO)
    logging.getLogger('launch').setLevel(logging.INFO)
    logging.getLogger('launch_ros').setLevel(logging.INFO)


def _supports_color() -> bool:
    if os.environ.get('NO_COLOR'):
        return False
    if not sys.stdout.isatty():
        return False
    term = os.environ.get('TERM', '')
    return term not in ('', 'dumb')


def print_run_banner(scenario: str, seg_device: str, compact_logs: bool) -> None:
    entries = [
        ('scenario', scenario),
        ('segmentation_device', seg_device),
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


def resolve_exhibition_sdf(
    model_sdf_path: str,
    front_width: int = 640,
    front_height: int = 360,
    front_rate: float = 20.0,
    front_noise_stddev: float = 0.0,
    disable_rear_camera: bool = True,
    disable_lidar: bool = True,
) -> str:
    """Build a temporary, lighter sensor profile for the exhibition scenario.

    The source model remains untouched so parking and mission launches can keep
    using the rear camera and lidar. A non-zero low update rate is used for
    disabled sensors because Gazebo interprets an update rate of zero as
    unlimited.
    """
    if not os.path.exists(model_sdf_path):
        return model_sdf_path

    try:
        tree = ET.parse(model_sdf_path)
    except (ET.ParseError, OSError):
        return model_sdf_path

    root = tree.getroot()
    changed = False

    def set_text(parent: ET.Element, tag: str, value: str) -> None:
        nonlocal changed
        child = parent.find(tag)
        if child is None:
            child = ET.SubElement(parent, tag)
        if child.text != value:
            child.text = value
            changed = True

    for sensor in root.iter('sensor'):
        sensor_name = sensor.get('name', '')

        if sensor_name == 'cam0':
            changed = add_camera_rate_control(sensor, 'front', front_rate) or changed
            set_text(sensor, 'always_on', '1')
            set_text(sensor, 'update_rate', f'{float(front_rate):g}')
            if os.environ.get('E2E_SENSOR_METRICS') == '1':
                set_text(sensor, 'enable_metrics', 'true')
            image = sensor.find('./camera/image')
            if image is not None:
                set_text(image, 'width', str(int(front_width)))
                set_text(image, 'height', str(int(front_height)))
            noise = sensor.find('./camera/noise')
            if noise is not None:
                if float(front_noise_stddev) == 0.0:
                    # A zero-strength Gaussian shader still adds an entire
                    # fullscreen rendering pass in Gazebo. Remove that pass
                    # for the clean exhibition camera, preserving its pixels.
                    sensor.find('camera').remove(noise)
                    changed = True
                else:
                    set_text(noise, 'stddev', f'{float(front_noise_stddev):g}')

        elif sensor_name == 'cam1' and disable_rear_camera:
            set_text(sensor, 'always_on', '0')
            set_text(sensor, 'update_rate', '0.1')
            plugin = sensor.find('plugin')
            if plugin is not None:
                set_text(plugin, 'alwaysOn', '0')

        elif sensor_name == 'lidar' and disable_lidar:
            set_text(sensor, 'always_on', '0')
            set_text(sensor, 'update_rate', '0.1')

    # Actual body contacts provide a second recovery path in addition to the
    # predictive footprint guard. Ground/wheel rolling contacts are excluded.
    chassis = root.find(".//link[@name='chassis']")
    if chassis is not None and chassis.find("collision[@name='chassis_collision']") is not None and chassis.find("sensor[@name='exhibition_contacts']") is None:
        sensor = ET.SubElement(chassis, 'sensor', {'name': 'exhibition_contacts', 'type': 'contact'})
        ET.SubElement(sensor, 'always_on').text = 'true'
        ET.SubElement(sensor, 'update_rate').text = '50'
        ET.SubElement(ET.SubElement(sensor, 'contact'), 'collision').text = 'chassis_collision'
        plugin = ET.SubElement(sensor, 'plugin', {'name': 'exhibition_bumper', 'filename': 'libgazebo_ros_bumper.so'})
        ros = ET.SubElement(plugin, 'ros')
        ET.SubElement(ros, 'namespace').text = '/exhibition'
        ET.SubElement(ros, 'remapping').text = 'bumper_states:=contacts'
        ET.SubElement(plugin, 'frame_name').text = 'world'
        changed = True

    if not changed:
        return model_sdf_path

    fd, tmp_path = tempfile.mkstemp(prefix='mercedes_exhibition_', suffix='.sdf')
    os.close(fd)
    # gazebo_ros passes the file contents to lxml as a Unicode string; an XML
    # encoding declaration makes that code path fail before spawning.
    tree.write(tmp_path, encoding='utf-8', xml_declaration=False)
    return tmp_path
