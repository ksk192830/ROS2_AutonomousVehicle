import rclpy
from rclpy.node import Node
from interfaces_pkg.msg import LaneInfo, MotionCommand

class MotionNode(Node):
    def __init__(self):
        super().__init__('motion')

        # 1차선 주행 튜닝
        self.lane1_angle_weight = 0.75  # 차선 조향각 반영 가중치
        self.lane1_position_weight = 0.05  # 차량 횡방향 위치 오차 반영 가중치
        self.lane1_normal_speed = 255  # 차선 유지 주행 속도

        # 2차선 주행 튜닝
        self.lane2_angle_weight = 0.7  # 차선 조향각 반영 가중치
        self.lane2_position_weight = 0.05  # 차량 횡방향 위치 오차 반영 가중치
        self.lane2_normal_speed = 255  # 차선 유지 주행 속도

        # 차선 정보가 없을 때 사용할 기본 주행 튜닝
        self.default_angle_weight = 0.7  # 기본 차선 조향각 반영 가중치
        self.default_position_weight = 0.05  # 기본 횡방향 위치 오차 반영 가중치
        self.default_normal_speed = 255  # 기본 주행 속도

        # 조향 튜닝
        self.steering_angle_range = 50.0  # 입력 조향각 정규화 범위
        self.steering_output_range = 10.0  # 정규화된 조향각의 출력 범위
        self.position_adjust_limit = 3.0  # 차량 위치 보정값 절댓값 상한
        self.steering_limit = 10.0  # 최종 조향 명령 절댓값 상한

        # 상태 변수
        self.current_lane = None

        # Publisher and subscription
        self.motion_pub = self.create_publisher(MotionCommand, 'motion_command', 10)
        self.create_subscription(LaneInfo, '/cam0/lane_info', self.lane_info_callback, 10)

    def get_lane_config(self):
        if self.current_lane == 1:
            return (self.lane1_angle_weight,
                    self.lane1_position_weight,
                    self.lane1_normal_speed)
        elif self.current_lane == 2:
            return (self.lane2_angle_weight,
                    self.lane2_position_weight,
                    self.lane2_normal_speed)
        else:
            # 기본값 (차선 정보 없을 경우)
            return (self.default_angle_weight,
                    self.default_position_weight,
                    self.default_normal_speed)

    def lane_info_callback(self, msg):
        self.current_lane = msg.lane_num
        steering_angle = msg.steering_angle
        vehicle_position_x = msg.vehicle_position_x
        self.get_logger().info(f"vehicle_position: {vehicle_position_x} ")

        # 현재 차선 기준 설정값 가져오기
        angle_weight, position_weight, normal_speed = self.get_lane_config()

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
        cmd.left_speed = normal_speed
        cmd.right_speed = normal_speed
        self.get_logger().info(f"mapped: {mapped:.1f}, adjust: {adjust:.1f}, 현재 steer: {steering_value:.1f}\n\n")

        cmd.steering = int(steering_value)
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
