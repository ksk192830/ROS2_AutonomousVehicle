#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from interfaces_pkg.msg import LaneInfo, MotionCommand, CrossWalk
import re


class MotionNode(Node):
    def __init__(self):
        super().__init__('motion_mission')

        # #################### tuning parameters ######################
        # 신호등 튜닝
        self.traffic_light_stop_threshold = 5  # 적색 신호 정지 판정 연속 프레임 수
        self.traffic_light_start_threshold = 5  # 녹색 신호 출발 판정 연속 프레임 수

        # 횡단보도 튜닝
        self.cross_walk_height_threshold = 375  # 정지 판단용 횡단보도 최소 높이(px)

        # 차선 변경 튜닝
        self.front_vehicle_height = 330  # 전방 차량으로 판단할 최소 높이(px)
        self.lane_change_completion_position_threshold = 300  # 차선 변경 완료 허용 위치 오차(px)

        self.lane1_change_steering = -8  # 1차선 변경 시 조향 명령값
        self.lane2_change_steering = 5  # 2차선 변경 시 조향 명령값

        # 1차선 주행 튜닝
        self.lane1_angle_weight = 0.75  # 차선 조향각 반영 가중치
        self.lane1_position_weight = 0.05  # 차량 횡방향 위치 오차 반영 가중치
        self.lane1_normal_speed = 255  # 차선 유지 주행 속도
        self.lane1_lane_change_speed = 255  # 차선 변경 중 주행 속도

        # 2차선 주행 튜닝
        self.lane2_angle_weight = 0.7  # 차선 조향각 반영 가중치
        self.lane2_position_weight = 0.05  # 차량 횡방향 위치 오차 반영 가중치
        self.lane2_normal_speed = 255  # 차선 유지 주행 속도
        self.lane2_lane_change_speed = 255  # 차선 변경 중 주행 속도

        # 더이상 수정할 필요 없는 것들
        self.steering_angle_range = 50.0  # 입력 조향각 정규화 범위
        self.steering_output_range = 10.0  # 정규화된 조향각의 출력 범위
        self.position_adjust_limit = 3.0  # 차량 위치 보정값 절댓값 상한
        self.steering_limit = 10.0  # 최종 조향 명령 절댓값 상한

        self.lane2_obstacle_lane_change_threshold = 5  # 2차선 장애물 판정 연속 프레임 수
        self.camera_lane_change_threshold = 3  # 카메라 장애물 판정 연속 프레임 수

        # #################################################################

        # State
        self.traffic_light_stop_counter = 0
        self.traffic_light_start_counter = 0
        self.current_lane = 2
        self.target_lane = 2
        self.is_changing_lane = False
        self.cross_walk_found = False
        self.cross_walk_height = None
        self.obstacle_detected = False
        self.obstacle_height = 0.0
        self.traffic_light_detected = False
        self.traffic_light_color = ''
        self.camera_ready = {
            'lane': False,
            'traffic_light': False,
            'obstacle': False,
            'cross_walk': False,
        }
        self.camera_pipeline_ready = False
        self.last_waiting_for = None
        self.lane2_obstacle_lane_change_executed = False
        self.lane2_obstacle_lane_change_counter = 0
        self.camera_lane_change_counter = 0
        self.stop_state = False

        # Publisher and subscriptions
        self.motion_pub = self.create_publisher(MotionCommand, 'motion_command', 10)
        self.create_subscription(LaneInfo, '/cam0/lane_info', self.lane_info_callback, 10)
        self.create_subscription(String, 'traffic_light_result', self.traffic_callback, 10)
        self.create_subscription(String, 'obstacle_result', self.obstacle_callback, 10)
        self.create_subscription(CrossWalk, 'cross_walk_result', self.cross_walk_callback, 10)

    def obstacle_callback(self, msg: String):
        if self.is_changing_lane:
            return
        
        # 카메라 기반 장애물 메시지 파싱
        match = re.match(r'Detected:\s*(\w+),\s*Area:\s*([\d.]+),\s*Height:\s*([\d.]+)', msg.data)

        if not match:
            return
        self.camera_ready['obstacle'] = True
        self.obstacle_detected = (match.group(1).lower() == 'true')
        # area = float(match.group(2))
        self.obstacle_height = float(match.group(3))
        # self.get_logger().info(f"[Obstacle] detected={obstacle_detected}, area={area}, lane={self.current_lane}")

    def cross_walk_callback(self, msg: CrossWalk):
        if self.is_changing_lane :
            return
        self.camera_ready['cross_walk'] = True
        self.cross_walk_found = msg.found
        self.cross_walk_height = msg.height 

        

    def get_lane_config(self):  
        if self.current_lane == 1:
            return (self.lane1_angle_weight,
                    self.lane1_position_weight,
                    self.lane1_normal_speed,
                    self.lane1_lane_change_speed)
        else :
            return (self.lane2_angle_weight,
                    self.lane2_position_weight,
                    self.lane2_normal_speed,
                    self.lane2_lane_change_speed)
    
    def traffic_callback(self, msg):
        match = re.match(r'Detected:\s*(\w+),\s*Area:\s*([\d.]+)(?:,\s*Color:\s*(\w+))?', msg.data)
        # self.get_logger().info(msg.data)
        if not match:
            return

        self.camera_ready['traffic_light'] = True
        self.traffic_light_detected = match.group(1).lower() == 'true'
        self.traffic_light_color = (match.group(3) or '').lower()



    def lane_info_callback(self, msg: LaneInfo):
        self.camera_ready['lane'] = True

        if not self.camera_pipeline_ready:
            waiting_for = tuple(
                name for name, ready in self.camera_ready.items() if not ready
            )
            if waiting_for:
                if waiting_for != self.last_waiting_for:
                    self.get_logger().info(
                        "Waiting for camera pipeline: " + ", ".join(waiting_for)
                    )
                    self.last_waiting_for = waiting_for
                return

            self.camera_pipeline_ready = True
            self.get_logger().info(
                "Camera pipeline ready; motion command publishing enabled"
            )

        # Update for obstacle steering calculation
        self.current_lane = msg.lane_num
        steering_angle = msg.steering_angle
        vehicle_position_x = msg.vehicle_position_x
        
        angle_weight, position_weight,normal_speed, lane_change_speed =  self.get_lane_config()
        
        cmd = MotionCommand()
        mapped = (
            steering_angle
            / self.steering_angle_range
            * self.steering_output_range
            * angle_weight
        )
        adjust = - vehicle_position_x * position_weight
        adjust = max(
            -self.position_adjust_limit,
            min(adjust, self.position_adjust_limit),
        )
        steering_value = max(
            -self.steering_limit,
            min(mapped + adjust, self.steering_limit),
        )

        



        # ================= 목표를 2차선에서  1차선 변경 ================
        if (
            self.current_lane == 2
            and not self.lane2_obstacle_lane_change_executed
            and not self.is_changing_lane
        ):
            if self.obstacle_detected:
                self.lane2_obstacle_lane_change_counter += 1
                self.get_logger().info(
                    "Front obstacle detected for lane 2→1 change: "
                    f"{self.lane2_obstacle_lane_change_counter}/"
                    f"{self.lane2_obstacle_lane_change_threshold}"
                )
            else:
                self.lane2_obstacle_lane_change_counter = 0

            if (
                self.lane2_obstacle_lane_change_counter
                >= self.lane2_obstacle_lane_change_threshold
            ):
                self.target_lane = 1
                self.is_changing_lane = True
                self.lane2_obstacle_lane_change_executed = True
                self.lane2_obstacle_lane_change_counter = 0
                return
        # ==========================================================

        # ================= 목표를 1차선에서  2차선 변경 ================
        if self.current_lane == 1 and self.obstacle_detected and not  self.is_changing_lane :
            if self.obstacle_height > self.front_vehicle_height:
                self.camera_lane_change_counter += 1
            else:
                self.camera_lane_change_counter = 0
            
            if self.camera_lane_change_counter >= self.camera_lane_change_threshold:
                self.target_lane = 2
                self.is_changing_lane = True
                self.get_logger().info(
                    f"{self.camera_lane_change_threshold}회 연속 감지 "
                    f"🚧🚧🚧🚧🚧🚧🚧🚧🚧"
                )
                return
        # ==========================================================
        if self.target_lane != self.current_lane :
            self.is_changing_lane = True
             

        # Lane change logic
        if self.is_changing_lane:
            self.get_logger().info(f"🔄🔄\n")
            cmd.steering = (
                self.lane1_change_steering
                if self.target_lane == 1
                else self.lane2_change_steering
            )
            cmd.left_speed = lane_change_speed
            cmd.right_speed = lane_change_speed


            # Lane change 상태 해제
            if (
                self.current_lane == self.target_lane
                and abs(vehicle_position_x)
                <= self.lane_change_completion_position_threshold
            ):
                self.is_changing_lane = False
                self.get_logger().info(f"✅ Lane change complete: now on lane {self.current_lane}")
         
        # ==================================== 신호등 정지 로직 ====================================

        elif self.traffic_light_detected and self.cross_walk_found  and self.traffic_light_color == 'red' :
            if self.cross_walk_height > self.cross_walk_height_threshold:
                self.traffic_light_stop_counter += 1
            else: 
                self.traffic_light_stop_counter = 0

            if self.traffic_light_stop_counter > self.traffic_light_stop_threshold :
                self.stop_state = True
                self.get_logger().info("🛑 Red light detected: stopping vehicle")

            cmd.left_speed = normal_speed
            cmd.right_speed = normal_speed
            cmd.steering = int(steering_value)

        elif self.traffic_light_detected and self.cross_walk_found:
            if self.traffic_light_color == 'green':
                self.traffic_light_start_counter += 1
            else: 
                self.traffic_light_start_counter = 0
            
            if self.traffic_light_start_counter > self.traffic_light_start_threshold :
                self.stop_state = False
                self.get_logger().info("🟢 Red light cleared: resuming")
            
            cmd.left_speed = normal_speed
            cmd.right_speed = normal_speed
            cmd.steering = int(steering_value)

        elif self.traffic_light_detected:
            cmd.left_speed = normal_speed
            cmd.right_speed = normal_speed
            cmd.steering = int(steering_value)
        # ================================================================================================  

    
        else:     
            cmd.left_speed = normal_speed
            cmd.right_speed = normal_speed
            cmd.steering = int(steering_value)
            # self.get_logger().info(f"mapped: {mapped:.1f}, adjust: {adjust:.1f}, 현재 steer: {steering_value:.1f}")

        if self.stop_state :
            cmd.left_speed = 0
            cmd.right_speed = 0
            cmd.steering = 0

        self.motion_pub.publish(cmd)
    
def main(args=None):
    rclpy.init(args=args)
    node = MotionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('MotionNode interrupted')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
