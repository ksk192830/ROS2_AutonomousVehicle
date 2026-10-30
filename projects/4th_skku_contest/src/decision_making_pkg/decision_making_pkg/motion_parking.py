#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from interfaces_pkg.msg import ParkingSpace, MotionCommand, ParkingLot, EndLine, OutLine, State

import math

class ParkingNode(Node):

    def __init__(self):
        super().__init__("motion_parking")

        # 전방 주차공간 탐색 튜닝
        self.front_parking_space_missing_required_num = 5  # 전면 주차공간 미검출 필요 횟수

        # 전방 주차선 추종 튜닝
        self.parking_line_position_weight = 135  # 주차선 위치 오차 반영 가중치
        self.parking_line_angle_weight = 0.5  # 주차선 각도 오차 반영 가중치

        # 후방 주차공간 정렬 튜닝
        self.parking_space_position_weight = 14  # 주차공간 위치 오차 반영 가중치
        self.parking_space_angle_weight = 16  # 주차공간 각도 오차 반영 가중치
        self.parking_space_required_num = 3  # 주차 진입에 필요한 연속 정렬 횟수
        self.parking_in_yaw_threshold = 0.65  # 주차 진입 판단 최대 yaw

        # 후방 종료선 튜닝
        self.end_line_angle_weight = -50  # 후진 중 종료선 각도 오차 반영 가중치
        self.end_line_stop_y = 1250  # 최종 정지를 시작할 종료선 y 좌표(px)
        self.end_line_stering = 1000  # 종료선 기반 조향을 시작할 y 좌표(px)

        # 출차선 추종 튜닝
        self.out_line_position_weight = 0.5  # 출차선 위치 오차 반영 가중치
        self.out_line_angle_weight = 10  # 출차선 각도 오차 반영 가중치
        self.out_line_bottom_threshold = 0.9  # 출차선이 화면 하단에 도달했다고 판단할 y 좌표(0~1)
        self.final_straight_duration = 2.5  # 출차선 통과 후 직진 유지 시간(s)
        self.final_straight_speed = 200  # 마지막 직진 주행 속도

        # 상태 변수
        self.front_parking_space_missing_num = 0
        self.parking_space_found_num = 0
        self.parking_line_found = False
        self.front_parking_space_found = False
        self.parking_space_found = False
        self.end_line_found = False
        self.out_line_found = False
        self.state = "SEARCHING_SPACE"
        # self.state = "ALIGN_TO_SPACE"
        self.enter_stopped_time = None
        self.final_straight_start_time = None

        # Publisher, subscriptions and timer
        self.cmd_pub = self.create_publisher(MotionCommand, "/motion_command", 10)
        self.state_pub = self.create_publisher(State, "/motion_state", 10)
        self.create_subscription(ParkingLot, "/front_parking_line", self.parking_line_callback, 10)
        self.create_subscription(ParkingSpace, "/front_parking_space", self.front_parking_space_callback, 10)
        self.create_subscription(OutLine, "/front_out_line", self.out_line_callback, 10)
        self.create_subscription(ParkingSpace, "/rear_parking_space", self.parking_space_callback, 10)
        self.create_subscription(EndLine, "/rear_end_line", self.end_line_callback, 10)
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().debug("ParkingNode initialized with MotionCommand(steering, left_speed, right_speed).")
    

    # -------------------------
    # Callbacks
    # -------------------------
    def parking_line_callback(self, msg: ParkingLot):
        self.parking_line_found = msg.found
        if self.parking_line_found:
            self.parking_line_x = msg.x
            self.parking_line_yaw = msg.yaw

    def front_parking_space_callback(self, msg: ParkingSpace):
        self.front_parking_space_found = msg.found

        if self.state != "SEARCHING_SPACE":
            return

        if self.front_parking_space_found:
            self.front_parking_space_missing_num = 0
        else:
            self.front_parking_space_missing_num += 1

    def out_line_callback(self, msg: OutLine):
        self.out_line_found = msg.found
        if self.out_line_found:
            self.out_line_x = msg.x
            self.out_line_y = msg.y
            self.out_line_yaw = msg.yaw

    def parking_space_callback(self, msg: ParkingSpace):
        self.parking_space_found = msg.found
        if self.parking_space_found:
            self.parking_space_x = msg.x
            self.parking_space_yaw = msg.yaw
    
    def end_line_callback(self, msg: EndLine):
        self.end_line_found = msg.found
        if self.end_line_found:
            self.end_line_x = msg.x
            self.end_line_y = msg.y
            self.end_line_yaw = msg.yaw

    # -------------------------
    # Main control loop
    # -------------------------
    def control_loop(self):
        self.publish_state()
        if self.state == "SEARCHING_SPACE":
            self.update_searching()

        elif self.state == "ALIGN_TO_SPACE":
            self.update_alignment()

        elif self.state == "PARKING_IN":
            self.update_parking()

        elif self.state == "STOPPED":
            self.get_logger().debug("STOPPED")
            self.update_stopped()

        elif self.state == "ESCAPE":
            self.get_logger().debug("ESCAPE")
            self.update_escape()
        
        elif self.state == "TURN_RIGHT":
            self.get_logger().debug("TURN_RIGHT")
            self.turn_right()

        elif self.state == "FINAL_STRAIGHT":
            self.get_logger().debug("FINAL_STRAIGHT")
            self.update_final_straight()

        elif self.state == "FINISHED":
            self.update_finished()

    # -------------------------
    # State behaviors
    # -------------------------
    def update_searching(self):
        """1) 공간 찾기 전: 직진"""
        if self.parking_line_found:
            steer_position = (self.parking_line_x- 0.7) * self.parking_line_position_weight
            if self.parking_line_yaw < 0:
                yaw = 10 / 0.57 * (self.parking_line_yaw + 1.57)
            else: 
                yaw = 10 / 0.57 * (self.parking_line_yaw - 1.57)

            steer_angle = yaw * self.parking_line_angle_weight
            
            mapped_steering = steer_position + steer_angle
            mapped_speed = 100
            
            # self.get_logger().info(f"\nposition: {steer_position:.1f}\nangle: {steer_angle:.1f}\n최종 steer: {mapped_steering:.1f}\n\n\n")
            
            self.publish_cmd(steering=int(mapped_steering), speed=int(mapped_speed))
        
        if self.front_parking_space_missing_num >= self.front_parking_space_missing_required_num:
            self.get_logger().info(
                "Front parking_space missing "
                f"{self.front_parking_space_missing_num}/"
                f"{self.front_parking_space_missing_required_num} → ALIGN_TO_SPACE"
            )
            self.state = "ALIGN_TO_SPACE"
            return

        self.get_logger().debug(
            "Waiting for front parking_space missing count: "
            f"{self.front_parking_space_missing_num}/"
            f"{self.front_parking_space_missing_required_num}"
        )
        
    def update_alignment(self):
        """2) 주차공간 충분히 잡힐 때까지 좌회전"""
        # self.get_logger().info(f"🚧🚧🚧🚧🚧🚧🚧🚧🚧")
        self.publish_cmd(steering=-10, speed=100)

        if self.parking_space_found:
            self.get_logger().debug(
                f"Parking center yaw: {self.parking_space_yaw:.3f} rad "
                f"({math.degrees(self.parking_space_yaw):.1f} deg), "
                f"threshold: {self.parking_in_yaw_threshold:.3f} rad, "
                f"count: {self.parking_space_found_num}/"
                f"{self.parking_space_required_num + 1}"
            )
        else:
            self.get_logger().debug("Parking center line not found")

        if (
            self.parking_space_found
            and self.parking_space_yaw >= self.parking_in_yaw_threshold
        ):
            self.parking_space_found_num += 1
            if self.parking_space_found_num > self.parking_space_required_num:
                self.get_logger().info(
                    f"Parking center yaw {self.parking_space_yaw:.3f} rad "
                    f"<= {self.parking_in_yaw_threshold:.3f} rad → PARKING_IN"
                )
                self.state = "PARKING_IN"
        else:
            self.parking_space_found_num = 0

    def update_parking(self):
        """3) 후진하여 주차"""
        
        if self.end_line_found:
            self.get_logger().debug(f"end line y={self.end_line_y:.1f}, yaw={self.end_line_yaw:.3f}")
            if self.end_line_y >= self.end_line_stering:

                mapped_steering = self.end_line_yaw * self.end_line_angle_weight
                mapped_steering = max(min(mapped_steering, 10.0), -10.0)
                
                mapped_speed = -100
                self.get_logger().debug(f"parking steer={mapped_steering:.1f}")
                self.publish_cmd(steering=int(mapped_steering), speed=int(mapped_speed))

                if self.end_line_y >= self.end_line_stop_y:
                    self.state = "STOPPED"
                    return
                return
            
        elif not self.parking_space_found: # 인식 못하면 다시 좌회전 전진
            self.publish_cmd(steering=-10, speed=100)
            return
        
        steer_position = (self.parking_space_x - 0.5) * self.parking_space_position_weight
        steer_angle = self.parking_space_yaw * self.parking_space_angle_weight

        steer_position = -steer_position 
        yaw_target = math.pi / 2.0
        yaw_error = self.parking_space_yaw - yaw_target

        # 각도 wrap (-pi ~ pi)
        yaw_error = math.atan2(math.sin(yaw_error), math.cos(yaw_error))

        steer_angle = yaw_error * self.parking_space_angle_weight
        steer_angle = -steer_angle  # 후진 기준 보정

        # --- 3) Steering 합성 ---
        mapped_steering = steer_position + steer_angle
        mapped_steering = max(min(mapped_steering, 10.0), -10.0)
        self.get_logger().debug(
            f"parking position={steer_position:.1f}, angle={steer_angle:.1f}, steer={mapped_steering:.1f}"
        )

        mapped_speed = -100
        
        self.publish_cmd(steering=int(mapped_steering), speed=int(mapped_speed))


    def update_stopped(self):
        """4) 최종 정지"""
        self.publish_cmd(steering=0, speed=0)
        
        # 상태 진입 시간이 없다면 지금 시간 기록
        if self.enter_stopped_time is None:
            self.enter_stopped_time = self.get_clock().now()
            return

        # 경과시간 계산
        elapsed = (self.get_clock().now() - self.enter_stopped_time).nanoseconds / 1e9

        # 3초 지났는지 확인
        if elapsed >= 3.0:
            self.get_logger().info("정지 3초 완료.")
            self.enter_stopped_time = self.get_clock().now()
            self.state = "ESCAPE"
        else:
            self.get_logger().debug(f"정지 유지 중... {elapsed:.1f}s / 3.0s")

    def update_escape(self):
        """5) 주차선 밟지 않게 직진"""
        self.publish_cmd(steering=0, speed=200)
        
        # 경과시간 계산
        elapsed = (self.get_clock().now() - self.enter_stopped_time).nanoseconds / 1e9

        # 3초 지났는지 확인
        if elapsed >= 1.5:
            self.get_logger().info("전진 3초 완료.")
            self.enter_stopped_time = None
            self.state = "TURN_RIGHT"
        else:
            self.get_logger().debug(f"전진 유지 중... {elapsed:.1f}s / 3.0s")

    def turn_right(self):
        """6) out_line을 따라 주행"""
        if self.out_line_found:
            if self.out_line_y >= self.out_line_bottom_threshold:
                self.get_logger().info(
                    f"Out line reached bottom ({self.out_line_y:.2f}) "
                    "→ FINAL_STRAIGHT"
                )
                self.final_straight_start_time = self.get_clock().now()
                self.state = "FINAL_STRAIGHT"
                self.publish_cmd(steering=0, speed=self.final_straight_speed)
                return

            steer_position = (self.out_line_x - 0.5) * self.out_line_position_weight
            steer_angle = self.out_line_yaw * self.out_line_angle_weight
            
            mapped_steering = steer_position + steer_angle
            mapped_steering = max(min(mapped_steering, 10.0), -10.0)
            mapped_speed = 200
            
            self.get_logger().debug(
                f"turn_right position={steer_position:.1f}, angle={steer_angle:.1f}, steer={mapped_steering:.1f}"
            )
            self.publish_cmd(steering=int(mapped_steering), speed=int(mapped_speed))
        
        else:
            self.publish_cmd(steering=10, speed=200)

    def update_final_straight(self):
        """7) out_line 통과 후 일정 시간 직진"""
        if self.final_straight_start_time is None:
            self.final_straight_start_time = self.get_clock().now()

        elapsed = (
            self.get_clock().now() - self.final_straight_start_time
        ).nanoseconds / 1e9

        if elapsed >= self.final_straight_duration:
            self.get_logger().info("Final straight complete → FINISHED")
            self.state = "FINISHED"
            self.publish_cmd(steering=0, speed=0)
            return

        self.publish_cmd(steering=0, speed=self.final_straight_speed)

    def update_finished(self):
        """8) 주차 미션 종료 후 완전 정지"""
        self.publish_cmd(steering=0, speed=0)

    # -------------------------
    # MotionCommand 생성
    # -------------------------
    def publish_cmd(self, steering: int, speed: int):
        """
        steering: int32
        speed: int32  → left_speed = speed, right_speed = speed
        """
        msg = MotionCommand()
        msg.steering = int(steering)
        msg.left_speed = int(speed)
        msg.right_speed = int(speed)

        self.cmd_pub.publish(msg)

    # -------------------------
    # Current State 생성
    # -------------------------
    def publish_state(self):
        msg = State()
        msg.state = self.state
        self.state_pub.publish(msg)    

def main(args=None):
    rclpy.init(args=args)
    node = ParkingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
