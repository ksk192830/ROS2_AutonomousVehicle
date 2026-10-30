"""③ 주차(Parking) 판단 노드 — launch: final_parking.py

LiDAR만으로 동작하는 8단계 후진 주차 FSM.
직진하며 우측의 주차 차량 2대를 지나친 뒤, 두 차량 사이 빈 자리에
후진 주차하고(좌/우 거리 차로 조향 보정) 잠시 정지 후 우회전으로 탈출한다.

상태 흐름:
  SEARCH → FIRST_STOP → REVERSE_RIGHT → REVERSE_STRAIGHT
        → REVERSE_PAUSE → ADJUST_FORWARD → GO_OUT_TURN → GO_OUT_STRAIGHT

LiDAR 각도 기준 (rplidar, ranges 배열 비율로 인덱싱):
  95~105°  = 차량 우측  /  265~275° = 차량 좌측
모든 감지는 3회 연속(detect/clear count)일 때만 상태 전이 → 노이즈 억제.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from interfaces_pkg.msg import MotionCommand
from enum import Enum

class ParkingState(Enum):
    SEARCH = 1                     # 차량 탐색
    FIRST_STOP = 2                 # 첫 번째 정지
    REVERSE_RIGHT = 3              # 바퀴 오른쪽 최대 조향 후 후진
    REVERSE_STRAIGHT = 4           # 바퀴 정방향 후 직진 후진
    REVERSE_PAUSE = 5              # 후진 완료 후 3초 정지
    ADJUST_FORWARD = 6             # 전진 조정 (필요 시)
    GO_OUT_TURN = 7                # 탈출 회전
    GO_OUT_STRAIGHT = 8            # 탈출 직진

class ParkingNode(Node):
    def __init__(self):
        super().__init__('motion_parking')
        self.state = ParkingState.SEARCH # 시작 상태
        self.state_start_time = None

        # -------------------------------------
        # --- 시작위치에 따라 초기값을 꼭 조정할 것 ---
        self.init_steer = 7 
        # -------------------------------------

        self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.motion_pub = self.create_publisher(MotionCommand, 'motion_command', 10)

        # 장애물 관련 상태 변수
        self.obs_detect_count = 0
        self.obs_clear_count = 0
        self.obstacle_state = 'no_obstacle'  # or 'passed_first', 'between', 'passed_second'

        self.left_obs_detect_count = 0
        self.left_obs_clear_count = 0
        self.left_obstacle_state = 'no_obstacle'  # or 'passing' or 'out'
        self.right_obs_detect_count = 0
        self.right_obs_clear_count = 0
        self.right_obstacle_state = 'no_obstacle'  # or 'passing' or 'out'

    def lidar_callback(self, msg: LaserScan):
        """LiDAR 스캔마다 현재 상태의 핸들러로 분기 (FSM 디스패처)."""
        if self.state == ParkingState.SEARCH:
            self.handle_search(msg)
        elif self.state == ParkingState.FIRST_STOP:
            self.handle_first_stop(msg)
        elif self.state == ParkingState.REVERSE_RIGHT:
            self.handle_reverse_right(msg)
        elif self.state == ParkingState.REVERSE_STRAIGHT:
            self.handle_reverse_straight(msg)
        elif self.state == ParkingState.REVERSE_PAUSE:
            self.handle_reverse_pause(msg)
        elif self.state == ParkingState.ADJUST_FORWARD:
            self.handle_adjust_forward(msg)
        elif self.state == ParkingState.GO_OUT_TURN:
            self.handle_go_out_turn(msg)
        elif self.state == ParkingState.GO_OUT_STRAIGHT:
            self.handle_go_out_straight(msg)


    def handle_search(self, msg: LaserScan):
        """[SEARCH] 직진하며 우측 LiDAR로 주차 차량 감지.

        감지 상태를 no_obstacle → passed_first → between → passed_second로 진행시켜
        '두 번째 주차 차량을 완전히 지나친 순간' 정지하고 FIRST_STOP으로 전이.
        """
        # 265~275도 범위 평균 거리 계산
        angle_min_idx = int(len(msg.ranges) * (95 / 360))
        angle_max_idx = int(len(msg.ranges) * (105 / 360))
        section = msg.ranges[angle_min_idx:angle_max_idx]
        filtered = [r for r in section if 0.0 < r < float('inf')]
        avg_distance = sum(filtered) / len(filtered) if filtered else float('inf')

        # 조금 가까운 곳에서 시작했을 땐 장애물과의 거리가 1.4m정도였고
        # 조금 멀리서 시작하면 1.6-1.7m정도로 감지됨. 이에 따라서 후진 스피드를 다르게 주는 것도 고려.
        if avg_distance <= 2.0:
            self.get_logger().info(f"🚧 265~275도 범위에서 물체 감지! 거리: {avg_distance:.2f}m")

            self.obs_detect_count += 1
            self.obs_clear_count = 0
        else:
            self.obs_clear_count += 1
            self.obs_detect_count = 0

        if self.obstacle_state == 'no_obstacle' and self.obs_detect_count >= 3:
            self.obstacle_state = 'passed_first'
            self.get_logger().info("🚧 첫 번째 장애물 감지 완료")

        elif self.obstacle_state == 'passed_first' and self.obs_clear_count >= 3:
            self.obstacle_state = 'between'
            self.get_logger().info("🟩 첫 번째와 두 번째 장애물 사이")

        elif self.obstacle_state == 'between' and self.obs_detect_count >= 3:
            self.obstacle_state = 'passed_second'
            self.get_logger().info("🚧 두 번째 장애물 감지 완료")
            # self.get_logger().info(f"time: {self.get_clock().now().to_msg().sec} s")

        elif self.obstacle_state == 'passed_second' and self.obs_clear_count >= 3:
            self.get_logger().info("🟩 두 번째 장애물 통과 - 정지")
            self.publish_motion_command(0, 0, self.init_steer)
            self.state = ParkingState.FIRST_STOP
            # self.get_logger().info(f"time: {self.get_clock().now().to_msg().sec} s")
            
            self.state_start_time = self.get_clock().now()

        else:
            # 장애물 통과 중 직진
            self.publish_motion_command(100, 100, -1) # 가변저항값 조정으로 인해 -1 조향을 줘야 직진을 한다.

    def handle_first_stop(self, msg: LaserScan):
        """[FIRST_STOP] 정지 상태에서 조향을 우측 최대로 꺾어두고 잠시 대기 후 후진 시작."""
        self.publish_motion_command(0, 0, 30)
        # 첫 번째 정지 상태에서 1초 후 REVERSE_RIGHT 상태로 전이
        if (self.get_clock().now() - self.state_start_time).nanoseconds >= 2e9:
            self.state_start_time = None
            self.get_logger().info("🔁 1초 정지 완료 → REVERSE_RIGHT 시작")
            self.state = ParkingState.REVERSE_RIGHT
        
    def handle_reverse_right(self, msg: LaserScan):
        """[REVERSE_RIGHT] 우측 조향 후진으로 주차 칸에 진입.

        좌/우 양쪽에서 동시에 주차 차량이 감지되면(둘 다 'passing')
        차체가 두 차량 사이에 들어온 것이므로 REVERSE_STRAIGHT로 전이.
        """
        # 오른쪽으로 후진
        self.publish_motion_command(-70, -30, self.init_steer)
        self.get_logger().info("🔁 오른쪽으로 후진 중...")

        #-------------------------------------------------------
        # 라이다의 좌우에 장애물이 잡히는지를 통해
        # REVERSE_STRAIGHT 상태로의 전이 여부를 판단

        # 왼쪽 장애물 탐지
        left_angle_min_idx = int(len(msg.ranges) * (265 / 360))
        left_angle_max_idx = int(len(msg.ranges) * (275 / 360))
        left_section = msg.ranges[left_angle_min_idx:left_angle_max_idx]
        left_filtered = [r for r in left_section if 0.0 < r < float('inf')]
        left_avg_distance = sum(left_filtered) / len(left_filtered) if left_filtered else float('inf')

        if left_avg_distance <= 1.0:
            self.get_logger().info(f"🚧 왼쪽에서 물체 감지! 거리: {left_avg_distance:.2f}m")

            self.left_obs_detect_count += 1
            self.left_obs_clear_count = 0
        else:
            self.left_obs_clear_count += 1
            self.left_obs_detect_count = 0

        # ---
        if self.left_obstacle_state == 'no_obstacle' and self.left_obs_detect_count >= 3:
            self.left_obstacle_state = 'passing'
            self.get_logger().info("🚧 좌측 주차 차량 탐지 완료 - 지나는 중")

        # elif self.left_obstacle_state == 'passing' and self.left_obs_clear_count >= 3:
        #     self.left_obstacle_state = 'out'
        #     self.get_logger().info("🟩 좌측 주차 차량 탐지 안됨")


        # --------------------------------------------------------
        # 오른쪽 장애물 탐지
        right_angle_min_idx = int(len(msg.ranges) * (95 / 360))
        right_angle_max_idx = int(len(msg.ranges) * (105 / 360))
        right_section = msg.ranges[right_angle_min_idx:right_angle_max_idx]
        right_filtered = [r for r in right_section if 0.0 < r < float('inf')]
        right_avg_distance = sum(right_filtered) / len(right_filtered) if right_filtered else float('inf')

        if right_avg_distance <= 1.0:
            self.get_logger().info(f"🚧 오른쪽에서 물체 감지! 거리: {right_avg_distance:.2f}m")

            self.right_obs_detect_count += 1
            self.right_obs_clear_count = 0
        else:
            self.right_obs_clear_count += 1
            self.right_obs_detect_count = 0

        # ---
        if self.right_obstacle_state == 'no_obstacle' and self.right_obs_detect_count >= 3:
            self.right_obstacle_state = 'passing'
            self.get_logger().info("🚧 우측 주차 차량 탐지 완료 - 지나는 중")

        # elif self.right_obstacle_state == 'passing' and self.right_obs_clear_count >= 3:
        #     self.right_obstacle_state = 'out'
        #     self.get_logger().info("🟩 우측 주차 차량 탐지 안됨")

        # REVERSE_STRAIGHT 상태 전이
        if self.left_obstacle_state == 'passing' and self.right_obstacle_state == 'passing':
            self.state = ParkingState.REVERSE_STRAIGHT

    def handle_reverse_straight(self, msg: LaserScan):
        """[REVERSE_STRAIGHT] 좌/우 차량 거리 차이로 조향을 보정하며 직진 후진.

        왼쪽에 치우치면 +3(우측), 오른쪽에 치우치면 -3(좌측)으로 미세 조정.
        좌/우 차량이 모두 시야에서 사라지면(둘 다 'out') 주차 완료로 판단.
        """
        # 똑바로 후진
        # self.publish_motion_command(-70, -70, -1) # 김수경표 좌우거리 차 조정을 안쓴다면
        steering = -1
        self.get_logger().info("🔁 똑바로 후진 중...")

        #-------------------------------------------------------
        # 라이다의 좌우에 장애물이 잡히는지를 통해
        # 주차 완료를 확힌 -> 양쪽 차량 간 거리 대략 1.3m

        # 왼쪽 장애물 탐지
        left_angle_min_idx = int(len(msg.ranges) * (265 / 360))
        left_angle_max_idx = int(len(msg.ranges) * (275 / 360))
        left_section = msg.ranges[left_angle_min_idx:left_angle_max_idx]
        left_filtered = [r for r in left_section if 0.0 < r < float('inf')]
        left_avg_distance = sum(left_filtered) / len(left_filtered) if left_filtered else float('inf')

        if left_avg_distance <= 1.0:
            self.get_logger().info(f"🚧 왼쪽에서 물체 감지! 거리: {left_avg_distance:.2f}m")

            self.left_obs_detect_count += 1
            self.left_obs_clear_count = 0
        else:
            self.left_obs_clear_count += 1
            self.left_obs_detect_count = 0

        # ---

        if self.left_obstacle_state == 'passing' and self.left_obs_clear_count >= 3:
            self.left_obstacle_state = 'out'
            self.get_logger().info("🟩 좌측 주차 차량 탐지 안됨")

        # --------------------------------------------------------
        # 오른쪽 장애물 탐지
        right_angle_min_idx = int(len(msg.ranges) * (95 / 360))
        right_angle_max_idx = int(len(msg.ranges) * (105 / 360))
        right_section = msg.ranges[right_angle_min_idx:right_angle_max_idx]
        right_filtered = [r for r in right_section if 0.0 < r < float('inf')]
        right_avg_distance = sum(right_filtered) / len(right_filtered) if right_filtered else float('inf')

        if right_avg_distance <= 1.0:
            self.get_logger().info(f"🚧 오른쪽에서 물체 감지! 거리: {right_avg_distance:.2f}m")

            self.right_obs_detect_count += 1
            self.right_obs_clear_count = 0
        else:
            self.right_obs_clear_count += 1
            self.right_obs_detect_count = 0

        # ---
        if self.right_obstacle_state == 'passing' and self.right_obs_clear_count >= 3:
            self.right_obstacle_state = 'out'
            self.get_logger().info("🟩 우측 주차 차량 탐지 안됨")

        #양쪽 차량 거리에 따라 조향 조절 
        if abs(left_avg_distance - right_avg_distance) < 0.4:
            steering = int(left_avg_distance - right_avg_distance)
        elif left_avg_distance < right_avg_distance:
            self.get_logger().info(f"✨ 왼쪽에 치우쳐짐 - 핸들 우측으로 꺾기! 거리 차이: {left_avg_distance - right_avg_distance:.2f}m")
            steering = 3
        else:
            self.get_logger().info(f"💡 오른쪽에 치우쳐짐 - 핸들 좌측으로 꺾기! 거리 차이: {left_avg_distance - right_avg_distance:.2f}m")
            steering = -3
        self.publish_motion_command(-70, -70, steering)

        # REVERSE_PAUSE 상태 전이
        if self.left_obstacle_state == 'out' and self.right_obstacle_state == 'out':
            self.state_start_time = self.get_clock().now()
            self.state = ParkingState.REVERSE_PAUSE

    def handle_reverse_pause(self, msg: LaserScan):
        """[REVERSE_PAUSE] 주차 완료 상태로 정지 유지 (심사 확인용 대기) 후 탈출 시작."""
        # 후진 완료 후 3초 정지
        self.publish_motion_command(0, 0, 0)
        self.get_logger().info("🔁 후진 완료 - 3초 정지 중...")

        if (self.get_clock().now() - self.state_start_time).nanoseconds >= 4e9:
            self.state_start_time = self.get_clock().now()
            self.get_logger().info("🔁 3초 정지 완료 → ADJUST_FORWARD 시작")
            self.state = ParkingState.ADJUST_FORWARD

    def handle_adjust_forward(self, msg: LaserScan):
        """[ADJUST_FORWARD] 전진으로 칸에서 빠져나오다가, 우측 차량을 벗어나면 우회전 시작."""
        # 탈출 직진
        self.publish_motion_command(70, 70, 1) # 그냥 직진할거면 -1. 우회전을 좀더 주려고 일단 1을 줘봄.
        self.get_logger().info("🔁 탈출 직진 중...")

        # --------------------------------------------------------
        # 오른쪽 장애물 탐지
        right_angle_min_idx = int(len(msg.ranges) * (95 / 360))
        right_angle_max_idx = int(len(msg.ranges) * (105 / 360))
        right_section = msg.ranges[right_angle_min_idx:right_angle_max_idx]
        right_filtered = [r for r in right_section if 0.0 < r < float('inf')]
        right_avg_distance = sum(right_filtered) / len(right_filtered) if right_filtered else float('inf')

        if right_avg_distance <= 1.0:
            self.get_logger().info(f"🚧 오른쪽에서 물체 감지! 거리: {right_avg_distance:.2f}m")

            self.right_obs_detect_count += 1
            self.right_obs_clear_count = 0
        else:
            self.right_obs_clear_count += 1
            self.right_obs_detect_count = 0

        # ---
        if (self.get_clock().now() - self.state_start_time).nanoseconds >= 3e9 and self.right_obstacle_state == 'out' and self.right_obs_clear_count >= 3:
            self.state_start_time = self.get_clock().now()
            self.right_obstacle_state = 'passing'
            self.state = ParkingState.GO_OUT_TURN
            self.get_logger().info("🟩 우측 주차 차량 탐지 안됨 - 우회전 시작")

    def handle_go_out_turn(self, msg: LaserScan):
        """[GO_OUT_TURN] 우회전으로 주행 방향 정렬 → 우측이 비면 직진 탈출로 전이."""
        # 탈출 우회전
        self.publish_motion_command(50, 20, 30)
        self.get_logger().info("🔁 탈출 우회전 중...")

        # --------------------------------------------------------
        # 오른쪽 장애물 탐지
        right_angle_min_idx = int(len(msg.ranges) * (95 / 360))
        right_angle_max_idx = int(len(msg.ranges) * (105 / 360))
        right_section = msg.ranges[right_angle_min_idx:right_angle_max_idx]
        right_filtered = [r for r in right_section if 0.0 < r < float('inf')]
        right_avg_distance = sum(right_filtered) / len(right_filtered) if right_filtered else float('inf')

        if right_avg_distance <= 1.5:
            self.get_logger().info(f"🚧 오른쪽에서 물체 감지! 거리: {right_avg_distance:.2f}m")

            self.right_obs_detect_count += 1
            self.right_obs_clear_count = 0
        else:
            self.right_obs_clear_count += 1
            self.right_obs_detect_count = 0

        # ---
        if (self.get_clock().now() - self.state_start_time).nanoseconds >= 25e9 and self.right_obstacle_state == 'passing' and self.right_obs_clear_count >= 3:
            self.right_obstacle_state = 'out'
            self.state = ParkingState.GO_OUT_STRAIGHT
            self.get_logger().info("🟩 우측 주차 차량 탐지 안됨 - 직진 시작")

    def handle_go_out_straight(self, msg: LaserScan):
        """[GO_OUT_STRAIGHT] 최종 상태: 직진으로 주차 구역 이탈 (미션 종료)."""
        # 탈출 직진
        self.publish_motion_command(70, 70, -1)

    def publish_motion_command(self, left_speed: int, right_speed: int, steering: int):
        cmd = MotionCommand()
        cmd.left_speed = left_speed
        cmd.right_speed = right_speed
        cmd.steering = steering
        self.motion_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = ParkingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()