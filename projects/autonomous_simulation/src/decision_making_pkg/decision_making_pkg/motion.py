import re
import time
import json

import cv2
import numpy as np
import rclpy
from interfaces_pkg.msg import DetectionArray, LaneInfo, MotionCommand
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from std_msgs.msg import Bool, Int32, String


class MotionNode(Node):
    """Lane following with commanded targets and camera obstacle avoidance."""

    def __init__(self):
        super().__init__('motion')

        self.declare_parameter('camera_width', 480.0)
        self.declare_parameter('obstacle_center_fraction', 0.20)
        # Lane2 keeps the early trigger that has already been validated.
        self.declare_parameter('obstacle_trigger_y', 105.0)
        self.declare_parameter('obstacle_trigger_height', 28.0)
        # Start preparing before the next car reaches the side safety envelope.
        # Require both position and size to reject tiny, distant detections.
        self.declare_parameter('lane1_obstacle_trigger_y', 100.0)
        self.declare_parameter('lane1_obstacle_trigger_height', 30.0)
        # Kept separately configurable for narrower forward camera views.
        self.declare_parameter('lane1_center_trigger_fraction', 0.16)
        self.declare_parameter('lane1_center_trigger_y', 100.0)
        self.declare_parameter('lane1_center_trigger_height', 30.0)
        self.declare_parameter('obstacle_confirm_frames', 2)
        self.declare_parameter('obstacle_lane_margin', 18.0)
        self.declare_parameter('obstacle_clear_frames', 6)
        self.declare_parameter('obstacle_avoidance_cooldown_sec', 1.0)
        self.declare_parameter('lane_change_complete_error', 38.0)
        self.declare_parameter('lane_change_confirm_frames', 5)
        self.declare_parameter('exhibition_safety_enabled', False)
        self.safety_enabled = bool(self.get_parameter('exhibition_safety_enabled').value)
        self.safety_at = 0.0
        self.safety = {}

        self.camera_width = float(self.get_parameter('camera_width').value)
        self.obstacle_center_fraction = float(
            self.get_parameter('obstacle_center_fraction').value
        )
        self.obstacle_trigger_y = float(
            self.get_parameter('obstacle_trigger_y').value
        )
        self.obstacle_trigger_height = float(
            self.get_parameter('obstacle_trigger_height').value
        )
        self.lane1_obstacle_trigger_y = float(
            self.get_parameter('lane1_obstacle_trigger_y').value
        )
        self.lane1_obstacle_trigger_height = float(
            self.get_parameter('lane1_obstacle_trigger_height').value
        )
        self.lane1_center_trigger_fraction = float(
            self.get_parameter('lane1_center_trigger_fraction').value
        )
        self.lane1_center_trigger_y = float(
            self.get_parameter('lane1_center_trigger_y').value
        )
        self.lane1_center_trigger_height = float(
            self.get_parameter('lane1_center_trigger_height').value
        )
        self.obstacle_confirm_frames = int(
            self.get_parameter('obstacle_confirm_frames').value
        )
        self.obstacle_lane_margin = float(
            self.get_parameter('obstacle_lane_margin').value
        )
        self.obstacle_clear_frames = int(
            self.get_parameter('obstacle_clear_frames').value
        )
        self.obstacle_avoidance_cooldown_sec = float(
            self.get_parameter('obstacle_avoidance_cooldown_sec').value
        )
        self.lane_change_complete_error = float(
            self.get_parameter('lane_change_complete_error').value
        )
        self.lane_change_confirm_frames = int(
            self.get_parameter('lane_change_confirm_frames').value
        )

        self.create_subscription(
            LaneInfo, '/cam0/lane_info', self.lane_info_callback, 10
        )
        self.create_subscription(
            String, 'traffic_light_result', self.traffic_callback, 10
        )
        self.create_subscription(
            DetectionArray,
            '/cam0/detections',
            self.detections_callback,
            QoSProfile(
                reliability=QoSReliabilityPolicy.RELIABLE,
                history=QoSHistoryPolicy.KEEP_LAST,
                durability=QoSDurabilityPolicy.VOLATILE,
                depth=1,
            ),
        )
        self.create_subscription(
            String, '/exhibition/command', self.command_callback, 10
        )

        self.motion_pub = self.create_publisher(
            MotionCommand, 'motion_command', 10
        )
        target_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            depth=1,
        )
        self.target_lane_pub = self.create_publisher(
            Int32, '/target_lane', target_qos
        )
        self.lane_change_pub = self.create_publisher(
            Bool, '/lane_change_active', target_qos
        )
        self.create_subscription(String, '/exhibition/safety', self.safety_callback, target_qos)

        self.current_lane = None
        self.last_valid_lane_at = 0.0
        self.create_timer(0.1, self._check_lane_timeout)
        self.target_lane = 2
        self.is_changing_lane = False
        self.last_published_lane_change = None
        self.change_complete_counter = 0

        self.obstacle_detect_counter = 0
        self.obstacle_clear_counter = 0
        self.obstacle_latched = False
        self.obstacle_cooldown_until = 0.0

        # Existing traffic-trigger state is retained; the very high thresholds
        # in this scenario keep it inactive unless deliberately configured.
        self.lane1_area_threshold = 9999999999
        self.lane2_area_threshold = 9999999
        self.required_count = 3
        self.detect_counter = 0
        self.clear_counter = 0
        self.wait_for_traffic_clear = False

        self.lane1_angle_weight = 0.75
        self.lane1_position_weight = 0.05
        self.lane1_normal_speed = 255
        self.lane1_lane_change_speed = 255

        self.lane2_angle_weight = 0.77
        self.lane2_position_weight = 0.035
        self.lane2_normal_speed = 255
        self.lane2_lane_change_speed = 255

        self.rear_offtrack_gain = 6.0
        self.rear_offtrack_max_offset = 30.0
        self.rear_offtrack_min_steer = 1.0

        # TRANSIENT_LOCAL keeps this initial target available for a lane node
        # that starts a little later in the launch sequence.
        self._publish_target_lane()
        self._publish_lane_change_state(force=True)

    def _publish_target_lane(self):
        self.target_lane_pub.publish(Int32(data=int(self.target_lane)))

    def _publish_lane_change_state(self, force=False):
        active = bool(self.is_changing_lane)
        if force or active != self.last_published_lane_change:
            self.lane_change_pub.publish(Bool(data=active))
            self.last_published_lane_change = active

    def safety_callback(self, msg):
        try:
            value = json.loads(msg.data)
            if not isinstance(value.get('lane_blocked'), bool) or not isinstance(value.get('stop'), bool):
                return
        except (ValueError, AttributeError):
            return
        self.safety, self.safety_at = value, time.monotonic()
        if value['stop'] and self.safety_enabled:
            self.motion_pub.publish(MotionCommand())

    def _safety_blocks(self, key):
        return self.safety_enabled and (time.monotonic() - self.safety_at > .5 or self.safety.get(key) is not False)

    def _set_target_lane(self, lane, reason, force=False, automatic=False):
        lane = int(lane)
        if lane not in (1, 2):
            return False
        guard = f'auto_lane{lane}_blocked' if automatic else f'manual_lane{lane}_blocked'
        if not force and (self.is_changing_lane or self._safety_blocks(guard)):
            return False
        changed = lane != self.target_lane
        self.target_lane = lane
        self.change_complete_counter = 0
        if self.current_lane is None or self.current_lane != lane or changed:
            self.is_changing_lane = True
        self._publish_lane_change_state()
        self._publish_target_lane()
        self.get_logger().info(
            f'Target lane {lane} requested ({reason}); changing={self.is_changing_lane}'
        )
        return True

    def command_callback(self, msg: String):
        command = msg.data.strip().lower()
        if command in ('demo_start', 'demo_reset'):
            self._set_target_lane(2, 'exhibition reset', force=True)
            self.obstacle_latched = False
            self.obstacle_detect_counter = 0
            self.obstacle_clear_counter = 0
            self.obstacle_cooldown_until = 0.0
            self.wait_for_traffic_clear = False
            self.detect_counter = 0
            self.clear_counter = 0
            return
        if command != 'lane_toggle':
            return
        base_lane = self.target_lane if self.target_lane in (1, 2) else 2
        if not self._set_target_lane(1 if base_lane == 2 else 2, 'manual command'):
            return
        # A manual command is explicit, so it also releases an old obstacle
        # latch. A still-present obstacle must be confirmed again before the
        # automatic logic can override the new target.
        self.obstacle_latched = False
        self.obstacle_detect_counter = 0
        self.obstacle_cooldown_until = 0.0

    def detections_callback(self, msg: DetectionArray):
        if self.is_changing_lane:
            return

        half_corridor = self.camera_width * self.obstacle_center_fraction
        center_x = self.camera_width * 0.5
        lane_polygons = {1: [], 2: []}
        cars = []
        for det in msg.detections:
            if det.class_name in ('lane1', 'lane2'):
                polygon = np.asarray(
                    [[point.x, point.y] for point in det.mask.data],
                    dtype=np.float32,
                )
                if polygon.shape[0] >= 3:
                    lane_index = 1 if det.class_name == 'lane1' else 2
                    lane_polygons[lane_index].append(polygon)
            elif det.class_name == 'car':
                cars.append(det)

        candidates = []
        for det in cars:
            cx = float(det.bbox.center.position.x)
            cy = float(det.bbox.center.position.y)
            height = float(det.bbox.size.y)
            # Classify the road immediately under the car. Points inside its
            # elevated body/side silhouette can be closer to the wrong lane.
            road_y = cy + height * .5 + self.camera_width / 120.0
            test_points = ((cx, road_y),)

            def best_lane_distance(lane):
                polygons = lane_polygons[lane]
                if not polygons:
                    return None
                return max(
                    cv2.pointPolygonTest(polygon, point, True)
                    for polygon in polygons
                    for point in test_points
                )

            target_distance = best_lane_distance(self.target_lane)
            other_lane = 1 if self.target_lane == 2 else 2
            other_distance = best_lane_distance(other_lane)

            if target_distance is not None:
                in_target_lane = target_distance >= -self.obstacle_lane_margin
                if other_distance is not None:
                    in_target_lane = (
                        in_target_lane and target_distance >= other_distance
                    )
            elif other_distance is not None:
                # A car can hide most of the narrow lane1 mask. If only the
                # opposite lane remains segmented, being clearly outside it is
                # still strong evidence that the car occupies our target lane.
                in_target_lane = other_distance < -self.obstacle_lane_margin
            else:
                # Fall back only when segmentation supplied no lane geometry.
                in_target_lane = (
                    center_x - half_corridor <= cx <= center_x + half_corridor
                )

            if in_target_lane:
                candidates.append((cy, float(det.bbox.size.y), cx))

        if candidates:
            nearest_y, nearest_height, nearest_x = max(
                candidates, key=lambda item: item[0] + item[1] * 0.5
            )
        else:
            nearest_y, nearest_height, nearest_x = 0.0, 0.0, center_x
        if self.target_lane == 1:
            close_lane1_obstacle = (
                nearest_y >= self.lane1_obstacle_trigger_y
                and nearest_height >= self.lane1_obstacle_trigger_height
            )
            centered_lane1_obstacle = (
                abs(nearest_x - center_x)
                <= self.camera_width * self.lane1_center_trigger_fraction
                and nearest_y >= self.lane1_center_trigger_y
                and nearest_height >= self.lane1_center_trigger_height
            )
            obstacle_ahead = close_lane1_obstacle or centered_lane1_obstacle
        else:
            obstacle_ahead = (
                nearest_y >= self.obstacle_trigger_y
                or nearest_height >= self.obstacle_trigger_height
            )

        if obstacle_ahead:
            self.obstacle_detect_counter += 1
            self.obstacle_clear_counter = 0
            if (
                self.obstacle_detect_counter >= self.obstacle_confirm_frames
                and not self.obstacle_latched
                and time.monotonic() >= self.obstacle_cooldown_until
            ):
                next_lane = 1 if self.target_lane == 2 else 2
                if self._set_target_lane(next_lane, 'front obstacle', automatic=True):
                    self.obstacle_latched = True
                    self.obstacle_cooldown_until = (
                        time.monotonic() + self.obstacle_avoidance_cooldown_sec
                    )
                self.obstacle_detect_counter = 0
        else:
            self.obstacle_detect_counter = 0
            self.obstacle_clear_counter += 1
            if (
                self.obstacle_clear_counter >= self.obstacle_clear_frames
                and time.monotonic() >= self.obstacle_cooldown_until
            ):
                self.obstacle_latched = False

    def get_lane_config(self):
        lane = self.target_lane if self.is_changing_lane else self.current_lane
        if lane == 1:
            return (
                self.lane1_angle_weight,
                self.lane1_position_weight,
                self.lane1_normal_speed,
                self.lane1_lane_change_speed,
            )
        if lane == 2:
            return (
                self.lane2_angle_weight,
                self.lane2_position_weight,
                self.lane2_normal_speed,
                self.lane2_lane_change_speed,
            )
        return (0.6, 0.05, 255, 100)

    def traffic_callback(self, msg: String):
        match = re.match(
            r'Detected:\s*(\w+),\s*Area:\s*([\d.]+)(?:,\s*Color:\s*(\w+))?',
            msg.data,
        )
        if not match:
            return

        detected = match.group(1).lower() == 'true'
        area = float(match.group(2))
        threshold = (
            self.lane1_area_threshold
            if self.current_lane == 1
            else self.lane2_area_threshold
        )

        if detected and area > threshold:
            self.clear_counter = 0
            if self.wait_for_traffic_clear:
                return
            self.detect_counter += 1
            if self.detect_counter >= self.required_count:
                accepted = self._set_target_lane(
                    1 if self.target_lane == 2 else 2,
                    'traffic mission trigger',
                )
                self.wait_for_traffic_clear = accepted
        else:
            self.detect_counter = 0
            if self.wait_for_traffic_clear:
                self.clear_counter += 1
                if self.clear_counter >= 100:
                    self.wait_for_traffic_clear = False
                    self.clear_counter = 0

    def lane_info_callback(self, msg: LaneInfo):
        if self._safety_blocks('stop'):
            self.motion_pub.publish(MotionCommand())
            return
        self.current_lane = int(msg.lane_num)
        if self.current_lane not in (1, 2):
            self.last_valid_lane_at = 0.0
            self.motion_pub.publish(MotionCommand())
            return
        self.last_valid_lane_at = time.monotonic()
        steering_angle = float(msg.steering_angle)
        vehicle_position_x = float(msg.vehicle_position_x)

        if self.current_lane != self.target_lane:
            self.is_changing_lane = True
            self._publish_lane_change_state()

        angle_weight, position_weight, normal_speed, lane_change_speed = (
            self.get_lane_config()
        )
        mapped = (steering_angle / 50.0) * 10.0 * angle_weight

        rear_offset = 0.0
        if abs(mapped) >= self.rear_offtrack_min_steer:
            rear_offset = -mapped * self.rear_offtrack_gain
            rear_offset = max(
                -self.rear_offtrack_max_offset,
                min(rear_offset, self.rear_offtrack_max_offset),
            )
        compensated_position_x = vehicle_position_x - rear_offset
        adjust = max(-4.0, min(-compensated_position_x * position_weight, 4.0))
        normal_steering = max(-10.0, min(mapped + adjust, 10.0))

        cmd = MotionCommand()
        if self.is_changing_lane:
            if self.current_lane == self.target_lane:
                # Once the target lane is visible, steer proportionally toward
                # its center instead of holding a fixed angle and overshooting.
                change_adjust = max(
                    -9.0, min(-vehicle_position_x * 0.065, 9.0)
                )
                steering_value = max(
                    -10.0, min(mapped + change_adjust, 10.0)
                )
            else:
                steering_value = -7.0 if self.target_lane == 1 else 7.0

            if (
                self.current_lane == self.target_lane
                and abs(vehicle_position_x) <= self.lane_change_complete_error
            ):
                self.change_complete_counter += 1
            else:
                self.change_complete_counter = 0

            if self.change_complete_counter >= self.lane_change_confirm_frames:
                self.is_changing_lane = False
                self.change_complete_counter = 0
                # The new lane can already contain the next obstacle. Do not
                # require it to disappear before allowing another avoidance.
                self.obstacle_latched = False
                self.obstacle_detect_counter = 0
                self.obstacle_cooldown_until = max(
                    self.obstacle_cooldown_until, time.monotonic() + .35)
                self._publish_lane_change_state()
                self.get_logger().info(
                    f'Lane change complete: centered in lane {self.target_lane}'
                )

            cmd.left_speed = lane_change_speed
            cmd.right_speed = lane_change_speed
        else:
            steering_value = normal_steering
            cmd.left_speed = normal_speed
            cmd.right_speed = normal_speed

        cmd.steering = int(round(steering_value))
        self.motion_pub.publish(cmd)

    def _check_lane_timeout(self):
        if self._safety_blocks('stop') or time.monotonic() - self.last_valid_lane_at > 0.5:
            self.motion_pub.publish(MotionCommand())


def main(args=None):
    rclpy.init(args=args)
    node = MotionNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        node.get_logger().info('MotionNode interrupted')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
