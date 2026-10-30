#!/usr/bin/env python3

import os
import random

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
from simulation_pkg.launch_runtime import (
    configure_launch_logging,
    get_default_output,
    get_quiet_ros_args,
    print_run_banner,
    resolve_compact_logs,
    resolve_ros2_control_sdf,
    resolve_yolo_device,
)


# logging config
# compact logs: process outputs go to ~/.ros/log instead of terminal
COMPACT_LOGS_DEFAULT = True
# COMPACT_LOGS_DEFAULT = False
SIM_SCENARIO_NAME = 'general'


# ego spawn config (easy to edit)
# mode: 'random' or 'fixed'
EGO_SPAWN_MODE = 'random'
# EGO_SPAWN_MODE = 'fixed'

# base pose (used for both fixed/random)
EGO_SPAWN_POSE = {
    'x': -1.15,
    'y': -6.80,
    'z': 0.0,
    'roll': 0.0,
    'pitch': 0.0,
    'yaw': 0.0,
}

# random jitter (used when EGO_SPAWN_MODE == 'random')
# final = base + uniform(-jitter, +jitter)
EGO_SPAWN_XY_JITTER = 0.05
EGO_SPAWN_YAW_JITTER = 0.1


def generate_launch_description():
    pkg_share = get_package_share_directory('simulation_pkg')
    rviz_pkg_share = get_package_share_directory('launch_pkg')
    camera_pkg_share = get_package_share_directory('camera_pkg')
    yolo_device = resolve_yolo_device()

    compact_logs = resolve_compact_logs(COMPACT_LOGS_DEFAULT)
    default_output = get_default_output(compact_logs)
    quiet_ros_args = get_quiet_ros_args(compact_logs)
    configure_launch_logging(compact_logs)

    print_run_banner(SIM_SCENARIO_NAME, yolo_device, compact_logs)

    world_file = os.path.join(pkg_share, 'worlds', 'track.world')
    urdf_file = os.path.join(pkg_share, 'urdf', 'mercedes_split_tex_control.urdf')
    rviz_default = os.path.join(rviz_pkg_share, 'rviz', 'general_sim.rviz')  # updated

    model_sdf_default = os.path.join(pkg_share, 'models', 'mercedes_split_tex', 'model.sdf')
    controllers_yaml = os.path.join(pkg_share, 'config', 'mercedes_controllers.yaml')
    model_sdf_default = resolve_ros2_control_sdf(model_sdf_default, controllers_yaml)
    model_path = os.path.join(pkg_share, 'models')
    spawn_script = os.path.join(get_package_prefix('gazebo_ros'), 'lib', 'gazebo_ros', 'spawn_entity.py')
    cam0_model_path = os.path.join(camera_pkg_share, 'model', 'track.pt')

    if EGO_SPAWN_MODE == 'fixed':
        default_x = str(EGO_SPAWN_POSE['x'])
        default_y = str(EGO_SPAWN_POSE['y'])
        default_z = str(EGO_SPAWN_POSE['z'])
        default_roll = str(EGO_SPAWN_POSE['roll'])
        default_pitch = str(EGO_SPAWN_POSE['pitch'])
        default_yaw = str(EGO_SPAWN_POSE['yaw'])
    else:
        default_x = str(EGO_SPAWN_POSE['x'] + random.uniform(-EGO_SPAWN_XY_JITTER, EGO_SPAWN_XY_JITTER))
        default_y = str(EGO_SPAWN_POSE['y'] + random.uniform(-EGO_SPAWN_XY_JITTER, EGO_SPAWN_XY_JITTER))
        default_z = str(EGO_SPAWN_POSE['z'])
        default_roll = str(EGO_SPAWN_POSE['roll'])
        default_pitch = str(EGO_SPAWN_POSE['pitch'])
        default_yaw = str(EGO_SPAWN_POSE['yaw'] + random.uniform(-EGO_SPAWN_YAW_JITTER, EGO_SPAWN_YAW_JITTER))

    with open(urdf_file, 'r', encoding='utf-8') as f:
        robot_description = f.read()

    model_name = LaunchConfiguration('model_name')
    model_file = LaunchConfiguration('model_file')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    roll = LaunchConfiguration('roll')
    pitch = LaunchConfiguration('pitch')
    yaw = LaunchConfiguration('yaw')

    declare_args = [
        DeclareLaunchArgument('model_name', default_value='ego_vehicle'),
        DeclareLaunchArgument('model_file', default_value=model_sdf_default),
        DeclareLaunchArgument('x', default_value=default_x),
        DeclareLaunchArgument('y', default_value=default_y),
        DeclareLaunchArgument('z', default_value=default_z),
        DeclareLaunchArgument('roll', default_value=default_roll),
        DeclareLaunchArgument('pitch', default_value=default_pitch),
        DeclareLaunchArgument('yaw', default_value=default_yaw),
        DeclareLaunchArgument(
            'use_auto_motion',
            default_value='true',
            # default_value='false',      # 셀프 조종할 때는 false로 설정
            description='Start the autonomous motion node that publishes /motion_command.',
        ),
    ]

    # Environment
    set_cuda = SetEnvironmentVariable('CUDA_VISIBLE_DEVICES', '0')
    set_rcutils_level = SetEnvironmentVariable(
        'RCUTILS_LOGGING_SEVERITY_THRESHOLD',
        'WARN' if compact_logs else 'INFO',
    )
    set_model_env = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=[model_path, ':', EnvironmentVariable('GAZEBO_MODEL_PATH', default_value='')],
    )

    # Gazebo + RViz
    gazebo = ExecuteProcess(
        cmd=['gazebo', world_file, '-s', 'libgazebo_ros_factory.so'],
        output=default_output,
    )
    gzserver = ExecuteProcess(
        cmd=['gzserver', world_file, '-s', 'libgazebo_ros_factory.so'],
        output=default_output,
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_default] + quiet_ros_args,
        output=default_output,
    )

    # Perception / decision (driving_sim-style)
    yolo_seg0 = Node(
        package='camera_pkg',
        executable='yolo_seg',
        name='yolo_seg0',
        namespace='cam0',
        output=default_output,
        arguments=quiet_ros_args,
        parameters=[
            {'device': yolo_device},
            {'model_path': cam0_model_path},
            {'threshold': 0.5},
        ],
        remappings=[
            ('image_raw', '/cam0/image_raw'),
            ('detections', '/cam0/detections'),
            ('seg_vis', '/cam0/seg_vis'),
        ],
    )

    lane_detector = Node(
        package='camera_pkg',
        executable='lane',
        name='lane_detector_cam0',
        namespace='cam0',
        output=default_output,
        arguments=quiet_ros_args,
        parameters=[
            {'camera_topic': '/cam0/image_raw'},
            {'detection_topic': '/cam0/detections'},
        ],
    )

    traffic_light = Node(
        package='camera_pkg',
        executable='traffic_light',
        name='traffic_light',
        output=default_output,
        arguments=quiet_ros_args,
        parameters=[],
    )

    motion_node = Node(
        package='decision_making_pkg',
        executable='motion',
        name='motion_node',
        output=default_output,
        arguments=quiet_ros_args,
        condition=IfCondition(LaunchConfiguration('use_auto_motion'))
    )

    # ros2_control simulation chain
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output=default_output,
        arguments=quiet_ros_args,
        parameters=[
            {'use_sim_time': True},
            {'robot_description': robot_description},
        ],
    )

    spawn_entity = ExecuteProcess(
        cmd=[
            'python3', spawn_script,
            '-entity', model_name,
            '-file', model_file,
            '-x', x,
            '-y', y,
            '-z', z,
            '-R', roll,
            '-P', pitch,
            '-Y', yaw,
            '--ros-args', '--log-level', 'error' if compact_logs else 'info',
        ],
        output=default_output,
    )

    spawner_jsb = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output=default_output,
    )

    spawner_front = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['front_steer_controller', '--controller-manager', '/controller_manager'],
        output=default_output,
    )

    spawner_rear = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['rear_wheel_controller', '--controller-manager', '/controller_manager'],
        output=default_output,
    )

    bridge = Node(
        package='simulation_pkg',
        executable='motion_command_ros2control_bridge',
        name='motion_command_ros2control_bridge',
        output=default_output,
    )

    # startup order: spawn -> JSB -> steer -> rear -> bridge
    chain_1 = RegisterEventHandler(
        OnProcessExit(target_action=spawn_entity, on_exit=[spawner_jsb])
    )
    chain_2 = RegisterEventHandler(
        OnProcessExit(target_action=spawner_jsb, on_exit=[spawner_front])
    )
    chain_3 = RegisterEventHandler(
        OnProcessExit(target_action=spawner_front, on_exit=[spawner_rear])
    )
    chain_4 = RegisterEventHandler(
        OnProcessExit(target_action=spawner_rear, on_exit=[bridge])
    )

    return LaunchDescription(
        declare_args
        + [
            set_model_env,
            set_cuda,
            set_rcutils_level,
            gazebo,
            # gzserver,
            rviz2,
            yolo_seg0,
            lane_detector,
            traffic_light, #YOLO 신호등 감지
            motion_node,
            rsp,
            spawn_entity,
            chain_1,
            chain_2,
            chain_3,
            chain_4,
        ]
    )
