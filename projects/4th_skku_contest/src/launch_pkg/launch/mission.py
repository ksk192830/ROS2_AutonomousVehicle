import os
import shutil

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node


def resolve_yolo_device():
    """Allow an override while still running on systems without CUDA."""
    forced = os.environ.get('E2E_YOLO_DEVICE')
    if forced:
        return forced
    return 'cuda:0' if shutil.which('nvidia-smi') else 'cpu'

def generate_launch_description():
    camera_pkg_share = get_package_share_directory('camera_pkg')
    launch_pkg_share = get_package_share_directory('launch_pkg')
    cam0_model_path = os.path.join(camera_pkg_share, 'model', 'track.engine')
    cam0_image_dir = os.path.join(camera_pkg_share, 'lib', 'traffic_light')
    rviz_config = os.path.join(launch_pkg_share, 'rviz', 'mission.rviz')
    yolo_device = resolve_yolo_device()

    return LaunchDescription([
        #################### CAMERA1(LANE) ######################
        
        Node(
            package='camera_pkg',
            executable='image',  # 실행할 노드 파일명
            name='cam0',
            namespace='cam0',
            parameters=[
                {'data_source': 'camera'},
                {'fallback_to_images': True},
                {'cam_num': 2}, # 포트 확인   ls /dev/video*
                {'img_dir': cam0_image_dir},
                {'pub_topic': '/cam0/image_raw'},
                {'window_name': 'Raw 0'},
                {'show_image': False},
                {'timer_period': 0.03},  # float형으로, fps조절 
                # {'timer_period': 0.1},
            ]
        ),
        
        #################### CAMERA1 ENGINE SEG ######################
        SetEnvironmentVariable('CUDA_VISIBLE_DEVICES', '0'),
        Node(
            package='camera_pkg',
            executable='engine_seg',
            name='engine_seg0',
            namespace='cam0',
            output='screen',
            parameters=[
                {'device': yolo_device},
                {'model_path': cam0_model_path},
                {'threshold': 0.5},
                {'class_names': ['background', 'car', 'crosswalk', 'lane1', 'lane2', 'traffic_light', 'car']}
            ],
            remappings=[
                ('image_raw', '/cam0/image_raw'),  # 카메라의 image_raw 토픽과 매핑
                ('detections', '/cam0/detections'),  # YOLO가 감지한 결과를 detections에 퍼블리시
                ('seg_vis', '/cam0/seg_vis')
            ]
        ),

        #################### LANE DETECT ######################
        Node(
            package='camera_pkg',
            executable='lane',  
            name='lane_detector_cam0',
            namespace='cam0',
            parameters=[
                {'camera_topic': '/cam0/image_raw'},
                {'detection_topic': '/cam0/detections'},
                {'seg_vis': '/cam0/seg_vis'}
            ],
            output='screen'
        ),

        #################### TRAFFIC LIGHT DETECT ######################
        Node(
            package='camera_pkg',
            executable='traffic_light',
            name='traffic_light',
            output='screen',
            parameters=[]
        ),

        #################### OBSTACLE DETECT ######################
        Node(
            package='camera_pkg',
            executable='obstacle',
            name='obstacle',
            output='screen',
            parameters=[]
        ),

        #################### CROSS WALK DETECT ######################
        Node(
            package='camera_pkg',
            executable='cross_walk_detect',  
            name='cross_walk',
            output='screen',
            parameters=[{
                'image_topic': '/cam0/image_raw',
                'detection_topic': '/cam0/detections',
                'mask_topic': '/cam0/cross_walk_mask',
                'result_topic': 'cross_walk_result',
            }],
        ),
        
        #################### MOTION ######################
        Node(
            package='decision_making_pkg',  
            executable='motion_mission',  
            name='motion_mission',
            output='screen'
        ),
        
        #################### CONTROL ######################
        # Node(
        #     package='control_pkg',  
        #     executable='control',  
        #     name='control_node',
        #     output='screen'
        # )

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen'
        ),
        
    ])
