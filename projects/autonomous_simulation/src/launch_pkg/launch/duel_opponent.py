"""Independent camera-driven AI opponent, with the same chassis and actuators."""
import os
import tempfile
import yaml
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from simulation_pkg.duel_runtime import duel_vehicle
from simulation_pkg.launch_runtime import resolve_exhibition_sdf, resolve_seg_device, continue_after_success


def generate_launch_description():
    sim = get_package_share_directory('simulation_pkg')
    camera = get_package_share_directory('camera_pkg')
    with open(os.path.join(sim, 'config/mercedes_controllers.yaml')) as stream:
        config = yaml.safe_load(stream)
    fd, controllers = tempfile.mkstemp(prefix='duel_controllers_', suffix='.yaml')
    with os.fdopen(fd, 'w') as stream:
        yaml.safe_dump({'/ai/' + key: value for key, value in config.items()}, stream)
    source = resolve_exhibition_sdf(os.path.join(sim, 'models/mercedes_split_tex/model.sdf'),
                                    front_width=480, front_height=270, front_rate=20)
    plugin = os.path.join(get_package_prefix('exhibition_sim_plugins'), 'lib/libexhibition_camera_filter.so')
    model = duel_vehicle(source, ai=True, controllers=controllers, camera_filter=plugin)
    with open(os.path.join(sim, 'urdf/mercedes_split_tex_control.urdf')) as stream:
        description = stream.read()
    rsp = Node(package='robot_state_publisher', executable='robot_state_publisher',
               namespace='ai', name='robot_state_publisher', output='log',
               parameters=[{'use_sim_time': True, 'robot_description': description}])
    spawn = ExecuteProcess(cmd=['/usr/bin/python3', '/opt/ros/humble/lib/gazebo_ros/spawn_entity.py',
                                '-entity', 'ai_rival', '-file', model, '-x', '32', '-y', '30', '-z', '0'], output='log')
    # Humble's Gazebo control plugin mutates the process-wide ROS arguments.
    # Finish creating the human controllers before loading the second plugin;
    # otherwise late human controllers inherit /ai and both cars obey AI input.
    wait_player = ExecuteProcess(cmd=['/usr/bin/python3', '-c', '''
import time, rclpy
rclpy.init()
n=rclpy.create_node('wait_for_player_controllers')
end=time.monotonic()+60
ready=False
while time.monotonic()<end:
    rclpy.spin_once(n, timeout_sec=.1)
    ready=all(n.count_subscribers(t)>0 for t in ('/front_steer_controller/commands','/rear_wheel_controller/commands'))
    if ready: break
n.destroy_node()
rclpy.shutdown()
raise SystemExit(0 if ready else 1)
'''], output='log')
    controllers_to_start = [Node(package='controller_manager', executable='spawner', namespace='ai',
        arguments=[name, '--controller-manager', '/ai/controller_manager', '--controller-manager-timeout', '45'],
        output='log') for name in ('joint_state_broadcaster', 'front_steer_controller', 'rear_wheel_controller')]
    bridge = Node(package='simulation_pkg', executable='motion_command_ros2control_bridge',
        namespace='ai', name='bridge', output='log', parameters=[{
            'motion_topic': '/ai/motion_command', 'front_steer_topic': '/ai/front_steer_controller/commands',
            'rear_wheel_topic': '/ai/rear_wheel_controller/commands', 'max_speed_mps': 1.82}])
    chain = [RegisterEventHandler(OnProcessExit(target_action=wait_player,
        on_exit=continue_after_success([spawn], 'Human vehicle controllers')))]
    previous = spawn
    for action in [*controllers_to_start, bridge]:
        chain.append(RegisterEventHandler(OnProcessExit(target_action=previous,
            on_exit=continue_after_success([action], 'AI vehicle controller'))))
        previous = action
    seg = Node(package='camera_pkg', executable='pth_seg', namespace='ai/cam0', name='segmentation',
        output='log', parameters=[{'device': resolve_seg_device(),
            'model_path': os.path.join(camera, 'model/general_mission_with_crosswalk.pth'),
            'threshold': .35, 'use_half': True, 'max_det': 20, 'mask_alpha': .38,
            'exhibition_role': 'opponent'}])
    lane = Node(package='camera_pkg', executable='lane', namespace='ai/cam0', name='lane', output='log',
        parameters=[{'camera_topic': '/ai/cam0/image_raw', 'detection_topic': '/ai/cam0/detections',
                     'control_reference_width': 480.0}],
        remappings=[('/target_lane', '/ai/target_lane'), ('/lane_change_active', '/ai/lane_change_active')])
    motion = Node(package='decision_making_pkg', executable='motion', namespace='ai', name='motion',
        output='log', parameters=[{'exhibition_safety_enabled': True}], remappings=[
            ('/cam0/lane_info', '/ai/cam0/lane_info'), ('/cam0/detections', '/ai/cam0/detections'),
            ('/target_lane', '/ai/target_lane'), ('/lane_change_active', '/ai/lane_change_active'),
            ('/exhibition/command', '/ai/exhibition/command'), ('/exhibition/safety', '/ai/exhibition/safety'),
            ('motion_command', '/ai/autonomy/motion_command')])
    return LaunchDescription([*chain, rsp, wait_player, seg, lane, motion])
