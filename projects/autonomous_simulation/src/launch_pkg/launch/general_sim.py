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
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from simulation_pkg.launch_runtime import (
    configure_launch_logging,
    continue_after_success,
    get_default_output,
    get_quiet_ros_args,
    print_run_banner,
    resolve_exhibition_world,
    resolve_compact_logs,
    resolve_exhibition_sdf,
    resolve_ros2_control_sdf,
    resolve_seg_device,
    resolve_graphics_environment,
    resolve_simulator_graphics_environment,
    resolve_gazebo_model_path,
    shutdown_on_exit,
)


# logging config
# compact logs: process outputs go to ~/.ros/log instead of terminal
COMPACT_LOGS_DEFAULT = True
# COMPACT_LOGS_DEFAULT = False
SIM_SCENARIO_NAME = 'general'

# Exhibition defaults: keep the live dashboard, skip engineering-only windows,
# and use a lighter sensor profile. Set E2E_PERFORMANCE_MODE=0 to opt out.
PERFORMANCE_MODE_DEFAULT = True
SHOW_GAZEBO_GUI_DEFAULT = False
SHOW_RVIZ_DEFAULT = False
SHOW_EXHIBITION_GUI_DEFAULT = True


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
    seg_device = resolve_seg_device()
    graphics_settings = resolve_graphics_environment()
    simulator_graphics = resolve_simulator_graphics_environment()
    high_resolution = os.environ.get('E2E_GAME_CAMERA_PROFILE') == '1'
    front_width = max(320, int(os.environ.get('E2E_FRONT_CAMERA_WIDTH', '1280' if high_resolution else '480')))
    front_height = max(180, int(os.environ.get('E2E_FRONT_CAMERA_HEIGHT', '720' if high_resolution else '270')))
    front_rate = max(10.0, float(os.environ.get('E2E_FRONT_CAMERA_RATE', '120' if high_resolution else '20')))
    # Only display quality changes: the model, masks, traffic-light thresholds
    # and lane controller keep the same 480 x 270 coordinate system.
    perception_width = 480 if high_resolution else front_width
    perception_height = 270 if high_resolution else front_height
    perception_topic = '/cam0/perception/image_raw' if high_resolution else '/cam0/image_raw'

    compact_logs = resolve_compact_logs(COMPACT_LOGS_DEFAULT)
    performance_mode = (
        os.environ.get('E2E_PERFORMANCE_MODE', '1' if PERFORMANCE_MODE_DEFAULT else '0') != '0'
    )
    default_output = get_default_output(compact_logs)
    quiet_ros_args = get_quiet_ros_args(compact_logs)
    configure_launch_logging(compact_logs)

    print_run_banner(SIM_SCENARIO_NAME, seg_device, compact_logs)

    world_file = resolve_exhibition_world(
        os.path.join(pkg_share, 'worlds', 'track.world'),
        top_width=960,
        top_height=720,
        top_rate=20.0,
        top_noise_stddev=0.0,
    )
    urdf_file = os.path.join(pkg_share, 'urdf', 'mercedes_split_tex_control.urdf')
    rviz_default = os.path.join(rviz_pkg_share, 'rviz', 'general_sim.rviz')  # updated

    model_sdf_default = os.path.join(pkg_share, 'models', 'mercedes_split_tex', 'model.sdf')
    controllers_yaml = os.path.join(pkg_share, 'config', 'mercedes_controllers.yaml')
    model_sdf_default = resolve_ros2_control_sdf(model_sdf_default, controllers_yaml)
    if performance_mode or high_resolution:
        model_sdf_default = resolve_exhibition_sdf(
            model_sdf_default,
            front_width=front_width,
            front_height=front_height,
            front_rate=front_rate,
            front_noise_stddev=0.0,
            disable_rear_camera=performance_mode,
            disable_lidar=performance_mode,
        )
    model_path = os.path.join(pkg_share, 'models')
    if os.environ.get('E2E_DUEL_MODE') == '1':
        from simulation_pkg.duel_runtime import duel_vehicle
        model_sdf_default = duel_vehicle(model_sdf_default)
    spawn_script = '/opt/ros/humble/lib/gazebo_ros/spawn_entity.py'
    cam0_model_path = os.path.join(
        camera_pkg_share,
        'model',
        'general_mission_with_crosswalk.pth',
    )

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
        DeclareLaunchArgument('autonomy_motion_topic', default_value='/motion_command'),
        DeclareLaunchArgument(
            'use_auto_motion',
            default_value='true',
            # default_value='false',      # 셀프 조종할 때는 false로 설정
            description='Start the autonomous motion node that publishes /motion_command.',
        ),
        DeclareLaunchArgument(
            'show_gazebo_gui',
            default_value='true' if SHOW_GAZEBO_GUI_DEFAULT else 'false',
            description='Show the Gazebo engineering window.',
        ),
        DeclareLaunchArgument(
            'show_rviz',
            default_value='true' if SHOW_RVIZ_DEFAULT else 'false',
            description='Show the RViz engineering window.',
        ),
        DeclareLaunchArgument(
            'show_exhibition_gui',
            default_value='true' if SHOW_EXHIBITION_GUI_DEFAULT else 'false',
            description='Show the visitor-facing autonomous-driving dashboard.',
        ),
        DeclareLaunchArgument(
            'enable_obstacles',
            default_value=LaunchConfiguration('show_exhibition_gui'),
            description='Run obstacle controls independently of the dashboard UI.',
        ),
        DeclareLaunchArgument(
            'obstacle_auto_recovery',
            default_value='true',
            description='Let the legacy obstacle manager reset a stuck vehicle.',
        ),
        DeclareLaunchArgument(
            'run_perception',
            default_value='true',
            description='Run segmentation, lane processing, and traffic-light nodes.',
        ),
        DeclareLaunchArgument(
            'bridge_max_speed_mps',
            default_value='1.2345',
            description='Maximum wheel-surface speed used by the ros2_control bridge.',
        ),
    ]

    # Environment
    set_cuda = SetEnvironmentVariable('CUDA_VISIBLE_DEVICES', '0')
    graphics_environment = [
        SetEnvironmentVariable(name, value)
        for name, value in graphics_settings.items()
    ]
    set_rcutils_level = SetEnvironmentVariable(
        'RCUTILS_LOGGING_SEVERITY_THRESHOLD',
        'WARN' if compact_logs else 'INFO',
    )
    set_model_env = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=resolve_gazebo_model_path(model_path),
    )
    set_model_database = SetEnvironmentVariable(
        'GAZEBO_MODEL_DATABASE_URI', os.environ.get('GAZEBO_MODEL_DATABASE_URI', ''),
    )

    # Gazebo + RViz
    gzserver = ExecuteProcess(
        cmd=['gzserver', world_file, '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
        output=default_output,
        additional_env=simulator_graphics,
        sigterm_timeout='5',
        sigkill_timeout='5',
    )

    gzclient = ExecuteProcess(
        cmd=['gzclient'],
        output=default_output,
        additional_env=simulator_graphics,
        sigterm_timeout='5',
        sigkill_timeout='5',
        condition=IfCondition(LaunchConfiguration('show_gazebo_gui')),
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_default] + quiet_ros_args,
        output=default_output,
        additional_env={'GTK_PATH': ''},
        condition=IfCondition(LaunchConfiguration('show_rviz')),
    )

    exhibition_gui = Node(
        package='exhibition_gui_pkg',
        executable='exhibition_dashboard',
        name='exhibition_dashboard',
        output=default_output,
        arguments=quiet_ros_args,
        parameters=[
            {'fullscreen': True},
            {'display_fps': 20.0},
        ],
        condition=IfCondition(LaunchConfiguration('show_exhibition_gui')),
    )

    exhibition_obstacle_manager = Node(
        package='exhibition_gui_pkg',
        executable='exhibition_obstacle_manager',
        name='exhibition_obstacle_manager',
        remappings=([('/model_states', '/exhibition/model_states')]
                    if os.environ.get('E2E_DUEL_MODE') == '1' else []),
        output=default_output,
        arguments=quiet_ros_args,
        parameters=[
            {'auto_recovery': LaunchConfiguration('obstacle_auto_recovery')},
            {'startup_grace_sec': 25.0},
            {'stuck_timeout_sec': 6.0},
            {'progress_distance_m': 0.18},
            {'recovery_cooldown_sec': 12.0},
        ],
        condition=IfCondition(LaunchConfiguration('enable_obstacles')),
    )

    perception_image = Node(
        package='camera_pkg', executable='perception_image', name='perception_image',
        output=default_output, arguments=quiet_ros_args,
        parameters=[{'width': perception_width, 'height': perception_height, 'max_rate': 20.0}],
        condition=IfCondition(LaunchConfiguration('run_perception')),
    )

    # Perception / decision (driving_sim-style)
    pth_seg0 = Node(
        package='camera_pkg',
        executable='pth_seg',
        name='pth_seg0',
        namespace='cam0',
        output=default_output,
        arguments=quiet_ros_args,
        parameters=[
            {'device': seg_device},
            {'model_path': cam0_model_path},
            {'threshold': 0.35},
            {'use_half': True},
            {'max_det': 20},
            {'mask_alpha': 0.38},
            {'exhibition_role': 'human' if os.environ.get('E2E_DUEL_MODE') == '1' else ''},
        ],
        remappings=[
            ('image_raw', perception_topic),
            ('detections', '/cam0/detections'),
            ('seg_vis', '/cam0/seg_vis'),
        ],
        condition=IfCondition(LaunchConfiguration('run_perception')),
    )

    lane_detector = Node(
        package='camera_pkg',
        executable='lane',
        name='lane_detector_cam0',
        namespace='cam0',
        output=default_output,
        arguments=quiet_ros_args,
        parameters=[
            {'camera_topic': perception_topic},
            {'detection_topic': '/cam0/detections'},
            {'control_reference_width': 480.0},
        ],
        condition=IfCondition(LaunchConfiguration('run_perception')),
    )

    traffic_light = Node(
        package='camera_pkg',
        executable='traffic_light',
        name='traffic_light',
        output=default_output,
        arguments=quiet_ros_args,
        parameters=[{'image_topic': perception_topic}],
        condition=IfCondition(LaunchConfiguration('run_perception')),
    )

    motion_node = Node(
        package='decision_making_pkg',
        executable='motion',
        name='motion_node',
        output=default_output,
        arguments=quiet_ros_args,
        remappings=[('motion_command', LaunchConfiguration('autonomy_motion_topic'))],
        parameters=[
            {'camera_width': float(perception_width)},
            {'exhibition_safety_enabled': ParameterValue(LaunchConfiguration('enable_obstacles'), value_type=bool)},
            {'obstacle_trigger_y': 105.0 * perception_height / 270.0},
            {'obstacle_trigger_height': 28.0 * perception_height / 270.0},
            {'lane1_obstacle_trigger_y': 100.0 * perception_height / 270.0},
            {'lane1_obstacle_trigger_height': 30.0 * perception_height / 270.0},
            {'lane1_center_trigger_fraction': 0.16},
            {'lane1_center_trigger_y': 100.0 * perception_height / 270.0},
            {'lane1_center_trigger_height': 30.0 * perception_height / 270.0},
            {'obstacle_confirm_frames': 2},
            {'obstacle_lane_margin': 18.0 * perception_width / 480.0},
            {'obstacle_clear_frames': 6},
            {'obstacle_avoidance_cooldown_sec': 1.0},
            {'lane_change_complete_error': 38.0},
        ],
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
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager',
                   '--controller-manager-timeout', '30'],
        output=default_output,
    )

    spawner_front = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['front_steer_controller', '--controller-manager', '/controller_manager',
                   '--controller-manager-timeout', '30'],
        output=default_output,
    )

    spawner_rear = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['rear_wheel_controller', '--controller-manager', '/controller_manager',
                   '--controller-manager-timeout', '30'],
        output=default_output,
    )

    bridge = Node(
        package='simulation_pkg',
        executable='motion_command_ros2control_bridge',
        name='motion_command_ros2control_bridge',
        output=default_output,
        parameters=[
            {
                'max_speed_mps': ParameterValue(
                    LaunchConfiguration('bridge_max_speed_mps'),
                    value_type=float,
                )
            }
        ],
    )

    # startup order: spawn -> JSB -> steer -> rear -> bridge
    chain_1 = RegisterEventHandler(
        OnProcessExit(target_action=spawn_entity,
                      on_exit=continue_after_success([spawner_jsb], 'Vehicle spawn'))
    )
    chain_2 = RegisterEventHandler(
        OnProcessExit(target_action=spawner_jsb,
                      on_exit=continue_after_success([spawner_front], 'Joint state controller'))
    )
    chain_3 = RegisterEventHandler(
        OnProcessExit(target_action=spawner_front,
                      on_exit=continue_after_success([spawner_rear], 'Steering controller'))
    )
    chain_4 = RegisterEventHandler(
        OnProcessExit(target_action=spawner_rear,
                      on_exit=continue_after_success([bridge], 'Wheel controller'))
    )
    stop_on_server_exit = RegisterEventHandler(
        OnProcessExit(target_action=gzserver, on_exit=shutdown_on_exit('Simulator'))
    )
    stop_on_dashboard_exit = RegisterEventHandler(
        OnProcessExit(target_action=exhibition_gui, on_exit=shutdown_on_exit('Dashboard'))
    )

    return LaunchDescription(
        declare_args
        + [
            set_model_env,
            set_model_database,
            set_cuda,
            *graphics_environment,
            set_rcutils_level,
            chain_1,
            chain_2,
            chain_3,
            chain_4,
            stop_on_server_exit,
            stop_on_dashboard_exit,
            gzserver,
            gzclient,
            rviz2,
            exhibition_gui,
            exhibition_obstacle_manager,
            *([perception_image] if high_resolution else []),
            pth_seg0,
            lane_detector,
            traffic_light, #YOLO 신호등 감지
            motion_node,
            rsp,
            spawn_entity,
        ]
    )
