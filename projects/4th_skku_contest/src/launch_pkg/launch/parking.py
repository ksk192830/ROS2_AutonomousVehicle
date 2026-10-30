from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os
import shutil


def resolve_yolo_device():
    forced = os.environ.get('E2E_YOLO_DEVICE')
    if forced:
        return forced
    return 'cuda:0' if shutil.which('nvidia-smi') else 'cpu'

def generate_launch_description():
    yolo_device = resolve_yolo_device()
    camera_pkg_share = get_package_share_directory('camera_pkg')
    decision_pkg_share = get_package_share_directory('decision_making_pkg')

    # 각 카메라에 맞는 모델 경로
    cam0_model_path = os.path.join(camera_pkg_share, 'model', 'parking_front.engine')
    cam1_model_path = os.path.join(camera_pkg_share, 'model', 'parking_rear.pt')
    cam0_image_dir = os.path.join(camera_pkg_share, 'lib', 'parking_front_4th')
    cam1_image_dir = os.path.join(camera_pkg_share, 'lib', 'parking_rear_4th')
    parking_map_yaml = os.path.join(decision_pkg_share, 'config', 'parking_map.yaml')
    rviz_config = os.path.join(get_package_share_directory('launch_pkg'), 'rviz', 'parking.rviz')


    return LaunchDescription([
        DeclareLaunchArgument(
            'lidar_port',
            default_value='ttyUSB0',
            description='LiDAR device name or full port path',
        ),

        #################### CAMERA1(전방) ######################
        Node(
            package='camera_pkg',
            executable='image',  # 실행할 노드 파일명
            name='cam0',
            namespace='cam0',
            parameters=[
                {'data_source': 'camera'},  # 기본: camera
                {'fallback_to_images': True},

                {'cam_num': 2},
                {'img_dir': cam0_image_dir},
                {'pub_topic': '/cam0/image_raw'},
                {'window_name': 'Raw 0'},
                {'show_image': False},

                {'timer_period': 0.03},
            ]
        ),

        #################### CAMERA2(후방) ######################
        Node(
            package='camera_pkg',
            executable='image',  # 실행할 노드 파일명
            name='cam1',
            namespace='cam1',
            parameters=[
                {'data_source': 'camera'},  # 기본: camera
                {'fallback_to_images': True},
                {'rotate_mode': 1},
                {'rotate_camera_only': True},

                {'cam_num': 4},
                {'img_dir': cam1_image_dir},
                {'pub_topic': '/cam1/image_raw'},
                {'window_name': 'Raw 1'},
                {'show_image': False},
 
                {'timer_period': 0.03},
            ]
        ),
        
        #################### CAMERA1 YOLO SEG ######################
        SetEnvironmentVariable('CUDA_VISIBLE_DEVICES', '0'),
        Node(
            package='camera_pkg',
            executable='engine_seg',
            name='engine_seg0',
            namespace='cam0',
            output='log',
            parameters=[
                # cpu or cuda:0
                {'device': yolo_device},
                {'model_path': cam0_model_path},
                {'threshold': 0.5},

                # segmentation 수행할 state 목록
                {'allowed_states': ['SEARCHING_SPACE', 'TURN_RIGHT']}
            ],
            remappings=[
                ('image_raw', '/cam0/image_raw'),  
                ('detections', '/cam0/detections'),
                ('seg_vis', '/cam0/seg_vis')  
            ]
        ),

        #################### CAMERA1 detection ######################
        Node(
            package="camera_pkg",           
            executable="parking_front_detect",
            name="parking_front_detect",
            output="log",
            parameters=[{
                "image_topic": "/cam0/image_raw",
                "detection_topic": "/cam0/detections",
                "viz_topic": "/front_viz",
            }],
        ),
        
        #################### CAMERA2 YOLO SEG ######################
        SetEnvironmentVariable('CUDA_VISIBLE_DEVICES', '0'),
        Node(
            package='camera_pkg',
            executable='yolo_seg',  
            name='yolo_seg1',
            namespace='cam1',
            output='log',
            parameters=[
                # cpu or cuda:0
                {'device': yolo_device},
                {'model_path': cam1_model_path},
                {'threshold': 0.5},

                # segmentation 수행할 state 목록
                {'allowed_states': ['ALIGN_TO_SPACE', 'PARKING_IN']}
            ],
            remappings=[
                ('image_raw', '/cam1/image_raw'),  
                ('detections', '/cam1/detections'),
                ('seg_vis', '/cam1/seg_vis')  
            ]
        ),

        #################### CAMERA2 detection ######################
        Node(
            package="camera_pkg",           
            executable="parking_rear_detect",
            name="parking_rear_detect",
            output="log",
            parameters=[{
                "image_topic": "/cam1/image_raw",
                "detection_topic": "/cam1/detections",
                "viz_topic": "/rear_viz",
            }],
        ),

        #################### 주차 로직 ######################
         
        Node(
            package='decision_making_pkg',
            executable='motion_parking',
            name='motion_parking',
            output='screen'
        ),

        #################### CONTROL ######################
        # Node(
        #     package='control_pkg',  
        #     executable='parking_control',  
        #     name='control_node',
        #     output='screen'
        # ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='log'
        ),

        
    ])
