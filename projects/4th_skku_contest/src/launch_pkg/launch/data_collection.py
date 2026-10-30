from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os
from pathlib import Path


# ===== 이미지 저장 경로 설정 =====
# None이면 원본 camera_pkg/camera_pkg/lib 경로를 자동으로 찾습니다.
# 다른 위치에 저장하려면 절대 경로를 입력하세요.
# 예: SAVE_ROOT_DIR = '~/Pictures/driving_data'
SAVE_ROOT_DIR = None

CAM0_SAVE_FOLDER = 'parking_front_new'
CAM1_SAVE_FOLDER = 'parking_BEV2'
# ================================


def find_camera_source_lib():
    """Find camera_pkg/camera_pkg/lib in the source workspace."""
    if SAVE_ROOT_DIR:
        save_root = Path(SAVE_ROOT_DIR).expanduser().resolve()
        save_root.mkdir(parents=True, exist_ok=True)
        return str(save_root)

    override = os.environ.get('CAMERA_PKG_SOURCE_LIB')
    if override:
        source_lib = Path(override).expanduser().resolve()
        if source_lib.is_dir():
            return str(source_lib)
        raise RuntimeError(
            f'CAMERA_PKG_SOURCE_LIB is not a directory: {source_lib}'
        )

    camera_pkg_share = Path(get_package_share_directory('camera_pkg')).resolve()
    search_roots = [
        Path(__file__).resolve(),
        camera_pkg_share,
        Path.cwd().resolve(),
    ]

    for root in search_roots:
        for parent in (root, *root.parents):
            candidate = parent / 'src' / 'camera_pkg' / 'camera_pkg' / 'lib'
            if candidate.is_dir():
                return str(candidate)

            candidate = parent / 'camera_pkg' / 'camera_pkg' / 'lib'
            if candidate.is_dir():
                return str(candidate)

    # Keep data captures outside the version-controlled source tree.
    fallback = Path.home() / 'ros2_av_captures' / '4th_skku'
    fallback.mkdir(parents=True, exist_ok=True)
    return str(fallback)


def generate_launch_description():
    camera_source_lib = find_camera_source_lib()
    cam0_save_dir = os.path.join(camera_source_lib, CAM0_SAVE_FOLDER)
    cam1_save_dir = os.path.join(camera_source_lib, CAM1_SAVE_FOLDER)

    return LaunchDescription([
        #################### CAMERA1(전방) ######################
        Node(
            package='camera_pkg',
            executable='image',
            name='cam0',
            namespace='cam0',
            parameters=[
                {'data_source': 'camera'},
                {'cam_num': 2},
                {'pub_topic': '/cam0/image_raw'},
                {'window_name': 'Raw 0'},
                {'show_image': False},
                {'timer_period': 0.5},
            ]
        ),

        #################### IMAGE SAVER (전방) ######################
        DeclareLaunchArgument(
            'cam0_save_dir',
            default_value=cam0_save_dir,
            description='Directory to save images from cam0'
        ),
        DeclareLaunchArgument(
            'cam0_sub_topic',
            default_value='/cam0/image_raw',
            description='Image topic to subscribe for saving (cam0)'
        ),

        Node(
            package='camera_pkg',
            executable='image_saver',
            name='image_saver_cam0',
            namespace='cam0',
            output='screen',
            parameters=[
                {'save_dir': LaunchConfiguration('cam0_save_dir')},
                {'sub_topic': LaunchConfiguration('cam0_sub_topic')},
            ]
        ),

        #################### CAMERA2(후방) ######################
        Node(
            package='camera_pkg',
            executable='image',
            name='cam1',
            namespace='cam1',
            parameters=[
                {'data_source': 'camera'},
                {'rotate_mode': 1},
                {'cam_num': 4},
                {'pub_topic': '/cam1/image_raw'},
                {'window_name': 'Raw 1'},
                {'show_image': False},
                {'timer_period': 0.5},
            ]
        ),

        #################### IMAGE SAVER (후방) ######################
        DeclareLaunchArgument(
            'cam1_save_dir',
            default_value=cam1_save_dir,
            description='Directory to save images from cam1'
        ),
        DeclareLaunchArgument(
            'cam1_sub_topic',
            default_value='/cam1/image_raw',
            description='Image topic to subscribe for saving (cam1)'
        ),

        Node(
            package='camera_pkg',
            executable='image_saver',
            name='image_saver_cam1',
            namespace='cam1',
            output='screen',
            parameters=[
                {'save_dir': LaunchConfiguration('cam1_save_dir')},
                {'sub_topic': LaunchConfiguration('cam1_sub_topic')},
            ]
        ),
    ])
