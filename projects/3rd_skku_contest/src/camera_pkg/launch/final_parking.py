from camera_pkg.hardware_settings import load_hardware_settings
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    settings = load_hardware_settings()
    return LaunchDescription([

        #################### 주차 로직 ######################
         
        Node(
            package='decision_making_pkg',
            executable='motion_parking',
            name='motion_parking',
            output='screen'
        ),

        #################### Lidar ######################

        Node(
            package='rplidar_ros',
            executable='rplidar_composition',
            name='rplidar',
            output='screen',
            parameters=[{
                'serial_port': settings['lidar_serial_port'],
                'serial_baudrate': 115200,
                'frame_id': 'laser',
                'inverted': False,
                'angle_compensate': True,
            }],
        ),


        #################### Static TF ######################

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='lidar_tf_pub',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'laser'],
        ),

        #################### CONTROL ######################
        # Node(
        #     package='control_pkg',  
        #     executable='parking_control',  
        #     name='control_node',
        #     output='screen'
        # )
    ])
