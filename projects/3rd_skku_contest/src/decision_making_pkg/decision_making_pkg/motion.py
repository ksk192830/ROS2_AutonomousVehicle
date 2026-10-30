"""① 기록주행(Time Trial) 판단 노드 — launch: final.py

주행 로직:
  - 기본: LaneInfo(중앙선 기울기 + 횡방향 오프셋)로 조향을 계산해 최고 속도(255)로 차선 유지
  - 차선 변경: cam1 신호등이 threshold 면적 이상으로 required_count(3)회 연속 감지되면
    목표 차선을 토글(1↔2)하고, 신호등이 시야에서 사라질 때까지(clear 100회) 재감지를 막는다.
    즉, 신호등 위치를 "차선을 바꿔야 하는 구간"의 랜드마크로 사용.
  - 조향식: steering = (angle/50*10)*angle_weight + (-position_x)*position_weight, [-10,10] 클램프

발행: motion_command (MotionCommand) → control 노드가 시리얼로 아두이노에 전달
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from interfaces_pkg.msg import LaneInfo, MotionCommand
import re

class MotionNode(Node):
    def __init__(self):
        super().__init__('motion')

        # Subscriptions
        self.create_subscription(LaneInfo, '/cam0/lane_info', self.lane_info_callback, 10)
        self.create_subscription(String, 'traffic_light_result', self.traffic_callback, 10)

        # Publisher
        self.motion_pub = self.create_publisher(MotionCommand, 'motion_command', 10)

        # 신호등 감지 설정
        self.lane1_area_threshold = 11000
        self.lane2_area_threshold =  12000  # lane2에서 1
        self.required_count = 3

        # 상태 변수
        self.is_changing_lane = False
        self.wait_for_traffic_clear = False
        self.current_lane = None
        self.target_lane = 2

        # 신호등 감지 카운터
        self.detect_counter = 0
        self.clear_counter = 0

        # lane 1 설정
        self.lane1_angle_weight = 0.7
        self.lane1_position_weight = 0.05
        self.lane1_normal_speed =255
        self.lane1_lane_change_speed = 255

        # lane 2 설정
        self.lane2_angle_weight = 0.7
        self.lane2_position_weight = 0.05
        self.lane2_normal_speed = 255
        self.lane2_lane_change_speed = 255

    def get_lane_config(self):
        """현재 차선(1/2)에 맞는 (조향 각도 가중치, 위치 가중치, 주행 속도, 차선변경 속도) 반환."""
        if self.current_lane == 1:
            return (self.lane1_angle_weight,
                    self.lane1_position_weight,
                    self.lane1_normal_speed,
                    self.lane1_lane_change_speed)
        elif self.current_lane == 2:
            return (self.lane2_angle_weight,
                    self.lane2_position_weight,
                    self.lane2_normal_speed,
                    self.lane2_lane_change_speed)
        else:
            # 기본값 (차선 정보 없을 경우)
            return (0.6, 0.05, 255, 100)

    def traffic_callback(self, msg):
        """신호등 감지 결과 파싱 → 연속 3회 감지 시 목표 차선 토글.

        기록주행에서는 신호등 '색'은 무시하고, 신호등이 가까이(면적 큼) 보였다는
        사실만으로 차선 변경 구간에 진입했다고 판단한다.
        변경 후에는 신호등이 완전히 사라질 때까지(clear 100회) 중복 트리거를 막는다.
        """
        match = re.match(r'Detected:\s*(\w+),\s*Area:\s*([\d.]+)(?:,\s*Color:\s*(\w+))?', msg.data)
        if not match:
            return

        detected = match.group(1).lower() == 'true'
        area = float(match.group(2))
        # color = match.group(3)  # 색 정보 무시!
        

        # 현재 lane에 맞는 threshold 선택
        if self.current_lane == 1:
            threshold = self.lane1_area_threshold
        else :
            threshold = self.lane2_area_threshold

        # 신호등이 감지 됐으며 크기가 클 때
        if detected and area > threshold:
            self.clear_counter = 0

            self.get_logger().info("🚦🚦🚦🚦🚦🚦🚦🚦🚦🚦🚦🚦🚦🚦🚦🚦🚦🚦🚦🚦🚦🚦🚦🚦🚦🚦")

            if self.wait_for_traffic_clear:
                return

            # 신호등 감지 시
            self.detect_counter += 1
            # self.get_logger().info(f"🔎 Traffic detected {self.detect_counter}")

            if self.detect_counter >= self.required_count:
                # 목표 차선 변경
                self.target_lane = 1 if self.target_lane == 2 else 2
                self.wait_for_traffic_clear = True
                
        
        # 신호등 미감지
        else:
            self.detect_counter = 0

            if self.wait_for_traffic_clear:
                self.clear_counter += 1
                self.get_logger().info(f"🟤 Traffic not detected {self.clear_counter}")

                if self.clear_counter >= 100:
                    self.wait_for_traffic_clear = False
                    self.clear_counter = 0
                    
                    #신호등 다시 감지 시작
                    self.get_logger().info("🟢🟢🟢🟢🟢🟢🟢🟢🟢🟢🟢🟢🟢🟢🟢🟢🟢🟢🟢🟢🟢")

    def lane_info_callback(self, msg):
        """차선 정보 수신 → 조향/속도 계산 후 MotionCommand 발행.

        - 차선 변경 중: 고정 조향(-7 또는 +7)으로 목표 차선까지 이동,
          목표 차선 도달 + 중앙 근접(|position_x| ≤ 100) 시 변경 완료 처리
        - 평상시: 각도·오프셋 가중합으로 조향을 [-10,10] 범위에서 계산
        """
        self.current_lane = msg.lane_num
        steering_angle = msg.steering_angle
        vehicle_position_x = msg.vehicle_position_x

        # 차선 변경을 위한 상태 관리
        if self.current_lane != self.target_lane:
            self.is_changing_lane = True

        if self.is_changing_lane and self.current_lane == self.target_lane and abs(vehicle_position_x) <= 100:
            self.is_changing_lane = False
            self.get_logger().info(f"✅ Lane change complete: now on lane {self.current_lane}, centered in lane")

        # 현재 차선 기준 설정값 가져오기
        angle_weight, position_weight, normal_speed, lane_change_speed = self.get_lane_config()

        cmd = MotionCommand()

        # 차선 변경 중일 때
        if self.is_changing_lane:
            steering_value = -7 if self.target_lane == 1 else 7
            cmd.left_speed = lane_change_speed
            cmd.right_speed = lane_change_speed
            self.get_logger().info(f"🔁 Changing lane...")
        else:
            # 차선 변경이 완료되면 차선 유지
            mapped = (steering_angle / 50.0) * 10.0 * angle_weight
            adjust = -vehicle_position_x * position_weight
            steering_value = max(-10, min(mapped + adjust, 10))
            cmd.left_speed = normal_speed
            cmd.right_speed = normal_speed

            # lane1에서 좌회전 시 왼쪽 바퀴 속도 줄이기
            if self.current_lane == 1 and steering_value < -8:
                cmd.left_speed = 255


        cmd.steering = int(steering_value)
        self.motion_pub.publish(cmd)

        if self.current_lane != self.target_lane:
            self.get_logger().info(f"🎯 현재 차선: {self.current_lane}, 🎯목표 차선: {self.target_lane}")

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
