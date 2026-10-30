#!/usr/bin/env python3

import os
import random

from ament_index_python.packages import get_package_share_directory
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
SIM_SCENARIO_NAME = 'mission'


# mission obstacle spawn config (easy to edit)
# obstacle2/3 can choose independent case (case1~case3)
MISSION_OBSTACLE2_CASE = 'case1'
MISSION_OBSTACLE3_CASE = 'case3'

# common random jitter for mission obstacles
# final = base + uniform(-jitter, +jitter)
MISSION_OBSTACLE_XY_JITTER = 0.08
MISSION_OBSTACLE_YAW_JITTER = 0.1

# format: (x, y, z, roll, pitch, yaw)
MISSION_OBSTACLE1_POSE = (3.4, -5.1, 0.0, 0.0, 0.0, 0.9)
MISSION_OBSTACLE2_CASES = {
    'case1': (4.9, -1.4, 0.0, 0.0, 0.0, 1.7), 
    'case2': (4.9, -0.9, 0.0, 0.0, 0.0, 1.7),
    'case3': (4.8, -0.4, 0.0, 0.0, 0.0, 1.7),
}
MISSION_OBSTACLE3_CASES = {
    'case1': (3.6, 2.0, 0.0, 0.0, 0.0, 1.7),
    'case2': (3.6, 2.5, 0.0, 0.0, 0.0, 1.7),
    'case3': (3.6, 3.0, 0.0, 0.0, 0.0, 1.7),
}

# ego spawn config (easy to edit)
# mode: 'random' or 'fixed'
EGO_SPAWN_MODE = 'random'

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
    rviz_default = os.path.join(rviz_pkg_share, 'rviz', 'mission_sim.rviz')

    model_sdf_default = os.path.join(pkg_share, 'models', 'mercedes_split_tex', 'model.sdf')
    controllers_yaml = os.path.join(pkg_share, 'config', 'mercedes_controllers.yaml')
    model_sdf_default = resolve_ros2_control_sdf(model_sdf_default, controllers_yaml)
    model_path = os.path.join(pkg_share, 'models')
    spawn_script = '/opt/ros/humble/lib/gazebo_ros/spawn_entity.py'
    cam0_model_path = os.path.join(camera_pkg_share, 'model', 'track.pt')
    cam0_model_path_cw = os.path.join(camera_pkg_share, 'model', 'crosswalk.pt')
    traffic_light_model_path = os.path.join(camera_pkg_share, 'model', 'traffic_light_sim.pt')

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
            # default_value='false',        # 셀프 조종할 때는 false로 설정
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

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_default] + quiet_ros_args,
        output=default_output,
    )

    # Perception / decision
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

    yolo_seg1 = Node(
        package='camera_pkg',
        executable='yolo_seg',
        name='yolo_seg1',
        namespace='cam0',
        output=default_output,
        arguments=quiet_ros_args,
        parameters=[
            {'device': yolo_device},
            {'model_path': cam0_model_path_cw},
            {'threshold': 0.5},
        ],
        remappings=[
            ('image_raw', '/cam0/image_raw'),
            ('detections', '/cam0/detections_cw'),
            ('seg_vis', '/cam0/seg_vis_cw'),
        ],
    )

    yolo_traffic_light = Node(
        package='camera_pkg',
        executable='yolo_seg',
        name='yolo_traffic_light',
        namespace='cam0',
        output=default_output,
        arguments=quiet_ros_args,
        parameters=[
            {'device': yolo_device},
            {'model_path': traffic_light_model_path},
            {'threshold': 0.5},
        ],
        remappings=[
            ('image_raw', '/cam0/image_raw'),
            ('detections', '/cam0/traffic_light_detections'),
            ('seg_vis', '/cam0/traffic_light_seg_vis'),
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
        parameters=[
            {'image_topic': '/cam0/image_raw'},
            {'detection_topic': '/cam0/traffic_light_detections'},
        ],
    )

    obstacle = Node(
        package='camera_pkg',
        executable='obstacle',
        name='obstacle',
        output=default_output,
        arguments=quiet_ros_args,
        parameters=[],
    )

    cross_walk = Node(
        package='camera_pkg',
        executable='cross_walk_detect',
        name='cross_walk',
        output=default_output,
        arguments=quiet_ros_args,
        parameters=[],
    )

    motion_node = Node(
        package='decision_making_pkg',
        executable='motion_mission',
        name='motion_mission_node',
        output=default_output,
        arguments=quiet_ros_args,
        condition=IfCondition(LaunchConfiguration('use_auto_motion')),
    )

    obstacle2_pose = MISSION_OBSTACLE2_CASES.get(MISSION_OBSTACLE2_CASE, MISSION_OBSTACLE2_CASES['case1'])
    obstacle3_pose = MISSION_OBSTACLE3_CASES.get(MISSION_OBSTACLE3_CASE, MISSION_OBSTACLE3_CASES['case1'])

    mission_obstacle_spawns = [
        ('obstacle1', MISSION_OBSTACLE1_POSE),
        ('obstacle2', obstacle2_pose),
        ('obstacle3', obstacle3_pose),
    ]

    def _with_obstacle_jitter(pose):
        px, py, pz, rr, pp, yy = pose
        return (
            px + random.uniform(-MISSION_OBSTACLE_XY_JITTER, MISSION_OBSTACLE_XY_JITTER),
            py + random.uniform(-MISSION_OBSTACLE_XY_JITTER, MISSION_OBSTACLE_XY_JITTER),
            pz,
            rr,
            pp,
            yy + random.uniform(-MISSION_OBSTACLE_YAW_JITTER, MISSION_OBSTACLE_YAW_JITTER),
        )

    obstacle_spawn_nodes = [
        Node(
            package='simulation_pkg',
            executable='load_obstable_car_node',
            name=f'load_obstacle_car_{i+1}',
            output=default_output,
            arguments=[
                '--entity', name,
                '--x', str(pose_with_jitter[0]),
                '--y', str(pose_with_jitter[1]),
                '--z', str(pose_with_jitter[2]),
                '--roll', str(pose_with_jitter[3]),
                '--pitch', str(pose_with_jitter[4]),
                '--yaw', str(pose_with_jitter[5]),
            ],
        )
        for i, (name, pose_with_jitter) in enumerate(
            (name, _with_obstacle_jitter(pose)) for name, pose in mission_obstacle_spawns
        )
    ]

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
            '/usr/bin/python3', spawn_script,
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
            rviz2,
            yolo_seg0, # 전방 횡단보도를 뺀 나머지
            yolo_seg1, # 전방 횡단보도
            lane_detector,
            yolo_traffic_light,
            traffic_light,
            obstacle,
            cross_walk,
            motion_node,
            *obstacle_spawn_nodes,
            rsp,
            spawn_entity,
            chain_1,
            chain_2,
            chain_3,
            chain_4,
        ]
    )
