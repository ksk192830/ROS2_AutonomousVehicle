import torch
import numpy as np
from simulation_pkg import *


class Config: # 자주 바뀌는 항목들
    
    # "driving", "mission", "parking"
    MODE = "driving"  

    # 주행 관련 설정
    SPEED = 50    # 속도
    MAX_SPEED = 3.0  # 최대 속도 (m/s)

    
    # LIDAR 설정 (모드에 따라 동적으로 바뀌는 부분)
    LIDAR_SETTINGS = {
        "driving": {
            "lidar_start_angle": 0,  # 원하는 각도 범위의 시작 값
            "lidar_end_angle": 30,   # 원하는 각도 범위의 끝 값
            "lidar_range_min": 1.0,  # 원하는 거리 범위의 최소값 [m]
            "lidar_range_max": 12.0  # 원하는 거리 범위의 최대값 [m]
        },
        "mission": {
            "lidar_start_angle": 0,
            "lidar_end_angle": 30,
            "lidar_range_min": 1.0,
            "lidar_range_max": 12.0
        },
        "parking": {
            "lidar_start_angle": 0,
            "lidar_end_angle": 30,
            "lidar_range_min": 1.0,
            "lidar_range_max": 12.0
        }
    }
    
    # 차량 제어 관련 설정
    VEHICLE_CONTROL_SETTINGS = {
        "driving": {
            "steering": -1,   # 좌/우로 움직이는지 확인, 반대로 동작하면 -1
            "direction": 1   # 앞/뒤로 움직이는지 확인, 반대로 동작하면 -1
        },
        "mission": {
            "steering": 1,   
            "direction": 1 
        },
        "parking": {
            "steering": -1,
            "direction": 1
        }
        
    }
    
    @classmethod
    def get_lidar_settings(cls):
        return cls.LIDAR_SETTINGS.get(cls.MODE, cls.LIDAR_SETTINGS["driving"])
    
    @classmethod
    def get_vehicle_control_settings(cls):
        return cls.VEHICLE_CONTROL_SETTINGS.get(cls.MODE, cls.VEHICLE_CONTROL_SETTINGS["driving"])

    @classmethod
    def get_debug_setting(cls, setting_name):
        return cls.DEBUG_SETTINGS.get(setting_name, False)

    
# 카메라 설정
REAL_CAM = "image_raw"               # 실제 차량 이미지 
#SIM_CAM = "/camera/image_raw"        # 시뮬 전방 이미지
SIM_CAM = "image_raw"
SIM_CAM2 = "/rear_camera/image_raw"  # 시뮬 후방 이미지 camera_back
SIM_CAM3 = "/cam0/image_raw"   # 시뮬 위성 이미지

# lib 기본 제공 함수
camera_perception_func_lib = get_pyc("camera_perception_func_lib.cpython-310.pyc")
#lidar_perception_func_lib = get_pyc("lidar_perception_func_lib.cpython-310.pyc")
decision_making_func_lib = get_pyc("decision_making_func_lib.cpython-310.pyc")
#convert_arduino_msg = get_pyc("convert_protocol_lib.cpython-310.pyc")
#control_motor = get_pyc("control_motor_lib.cpython-310.pyc")

class SimulationSenderSettings:
    # simulation_sender_node
    
    MOTION_PLANNER_TOPIC = "/motion_command"
    GAZEBO_CONTROL_TOPIC = "/cmd_vel"
    
    # 주차는 전부 양수/ 주행은 +-
    VEHICLE_SETTINGS = Config.get_vehicle_control_settings()
    STEERING = VEHICLE_SETTINGS["steering"]
    DIRECTION = VEHICLE_SETTINGS["direction"]
    
    MAX_SPEED = Config.MAX_SPEED


# class RecordingSettings:
#     # video_recording_node
    
#     FPS = 30
#     IMAGE_SIZE = (640, 480)
    
#     # 어떤 시점의 영상을 녹화할 건지(전방/후방/위성)
#     RECORD_VIEW1 = SIM_CAM
#     RECORD_VIEW2 = SIM_CAM3    
    
#     # 비디오 파일의 저장 이름
#     RECORD_CAR = basic.get_data("car_view.mp4")
#     RECORD_UPPER = basic.get_data("top_view.mp4")
    