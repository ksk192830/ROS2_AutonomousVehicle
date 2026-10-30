#!/usr/bin/env python3

import os
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from simulation_pkg.launch_runtime import shutdown_on_exit, resolve_simulator_graphics_environment


def active_monitor_count() -> int:
    """Return active X11 monitor count, with a safe single-screen fallback."""
    forced_count = os.environ.get('E2E_MONITOR_COUNT')
    if forced_count:
        try:
            return max(1, int(forced_count))
        except ValueError:
            pass
    try:
        result = subprocess.run(
            ['xrandr', '--listactivemonitors'],
            check=False,
            capture_output=True,
            text=True,
            timeout=2.0,
        )
        monitors = [
            line for line in result.stdout.splitlines()[1:]
            if line.strip()
        ]
        return max(1, len(monitors))
    except (OSError, subprocess.SubprocessError):
        return 1


def generate_launch_description():
    os.environ.setdefault('E2E_GAME_CAMERA_PROFILE', '1')
    os.environ['E2E_DUEL_MODE'] = '1'
    monitor_count = active_monitor_count()
    dual_display = monitor_count >= 2
    driver_graphics = resolve_simulator_graphics_environment()
    if driver_graphics:
        driver_graphics = {**driver_graphics, 'E2E_GPU_CAMERA_DISPLAY': '1'}
    launch_share = get_package_share_directory('launch_pkg')
    general_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_share, 'launch', 'general_sim.py')
        ),
        launch_arguments={
            'use_auto_motion': 'true',
            'autonomy_motion_topic': '/autonomy/motion_command',
            'show_gazebo_gui': 'false',
            'show_rviz': 'false',
            'show_exhibition_gui': 'false',
            'enable_obstacles': 'true',
            # The guided coordinator owns pause/reset, including manual play.
            'obstacle_auto_recovery': 'false',
            # The real camera pipeline stays alive across demo and play. Only
            # the game coordinator publishes commands to the vehicle.
            'run_perception': 'true',
            # The simulated chassis reaches only ~0.837 m/s when the bridge's
            # nominal wheel-surface target is 1.2345 m/s.  Race-only loss
            # compensation restores the advertised 1.2345 m/s player maximum
            # without changing autonomous-driving launches.
            'bridge_max_speed_mps': '1.82',
            'x': '-1.15',
            'y': '-6.80',
            'z': '0.0',
            'roll': '0.0',
            'pitch': '0.0',
            'yaw': '0.0',
        }.items(),
    )
    opponent = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        os.path.join(launch_share, 'launch', 'duel_opponent.py')))

    controller = Node(
        package='exhibition_gui_pkg',
        executable='exhibition_linux_joy',
        name='race_game_controller',
        output='log',
    )

    dashboard = Node(
        package='exhibition_gui_pkg',
        executable='exhibition_dashboard',
        name='exhibition_dashboard',
        output='log',
        additional_env=driver_graphics,
        parameters=[
            {'fullscreen': True},
            {'display_fps': 15.0},
            {'screen_index': 0},
            {'race_mode': True},
            {'motion_topic': '/motion_command'},
        ],
    )

    race_game = Node(
        package='exhibition_gui_pkg',
        executable='exhibition_race_game',
        name='exhibition_race_game',
        remappings=[('/model_states', '/exhibition/model_states')],
        output='log',
        additional_env=driver_graphics,
        parameters=[
            {'fullscreen': True},
            {'display_fps': 240.0},
            {'screen_index': 1 if dual_display else 0},
            {'guided_experience': True},
            {'duel_enabled': True},
            {'analysis_screen_enabled': dual_display},
        ],
    )

    actions = [
        RegisterEventHandler(OnProcessExit(
            target_action=race_game, on_exit=shutdown_on_exit('Race game'),
        )),
        general_sim, opponent, controller,
    ]
    if dual_display:
        actions.append(dashboard)
    actions.append(race_game)
    return LaunchDescription(actions)
