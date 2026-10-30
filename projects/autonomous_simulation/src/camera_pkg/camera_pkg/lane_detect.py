import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    QoSDurabilityPolicy,
)
from sensor_msgs.msg import Image
from interfaces_pkg.msg import DetectionArray, LaneInfo
from cv_bridge import CvBridge
from std_msgs.msg import Bool, Int32, String
import json
import time
from camera_pkg.frame_sync import FramePairs, stamp_key
from camera_pkg.lane_geometry import extract_lane, control_from_geometry
import cv2
import numpy as np


# Exact OpenCV BGR equivalents of pth_seg.CLASS_COLORS_RGB for lane1/lane2.
# Keeping the values and alpha identical makes the camera and BEV stages read
# as the same segmentation after the perspective transform.
VIZ_LANE1 = (255, 150, 35)
VIZ_LANE2 = (185, 70, 235)
VIZ_MASK_ALPHA = 0.38
VIZ_SELECTED = (167, 226, 76)
VIZ_POINT = (245, 248, 252)
VIZ_DARK_HALO = (12, 22, 32)


class LaneDetector(Node):
    def __init__(self):
        super().__init__("lane_detector")

        # Parameters
        self.declare_parameter("camera_topic", "image_raw")
        self.declare_parameter("detection_topic", "detections")
        self.declare_parameter("persp_src", [0.25, 1.0, 0.75, 1.0, 0.62, 0.6, 0.38, 0.6])
        self.declare_parameter("persp_dst", [0.3, 1.0, 0.7, 1.0, 0.7, 0.0, 0.3, 0.0])
        # Keep the control signal resolution-independent. The original tuning
        # used a 480 px-wide camera; exhibition rendering can now be sharper.
        self.declare_parameter("control_reference_width", 480.0)
        self.declare_parameter("lock_selected_lane", True)

        cam_topic = self.get_parameter("camera_topic").value
        det_topic = self.get_parameter("detection_topic").value

        image_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1,
        )
        reliable_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1,
        )

        self.bridge = CvBridge()

        self.frame_pairs = FramePairs()
        self.stream_generation = 0
        self.last_source_at = None
        self.frame_size = None

        # Subscribers (no message_filters)
        self.det_sub = self.create_subscription(
            DetectionArray, det_topic, self.det_callback, reliable_qos
        )
        self.img_sub = self.create_subscription(
            Image, cam_topic, self.image_callback, image_qos
        )
        self.target_lane_sub = self.create_subscription(
            Int32,
            "/target_lane",
            self.target_lane_callback,
            QoSProfile(
                reliability=QoSReliabilityPolicy.RELIABLE,
                history=QoSHistoryPolicy.KEEP_LAST,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                depth=1,
            ),
        )
        self.lane_change_sub = self.create_subscription(
            Bool,
            "/lane_change_active",
            self.lane_change_callback,
            QoSProfile(
                reliability=QoSReliabilityPolicy.RELIABLE,
                history=QoSHistoryPolicy.KEEP_LAST,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                depth=1,
            ),
        )

        # Publishers
        self.viz_pub = self.create_publisher(Image, "lane_viz", image_qos)
        self.bev_pub = self.create_publisher(Image, "bev_viz", image_qos)
        self.lane_info_pub = self.create_publisher(LaneInfo, "lane_info", reliable_qos)

        self.geometry_pub = self.create_publisher(String, "lane_geometry", reliable_qos)
        self.CONTROL_REFERENCE_WIDTH = max(
            1.0, float(self.get_parameter("control_reference_width").value))
        self.LOCK_SELECTED_LANE = bool(self.get_parameter("lock_selected_lane").value)
        self.lane_width_estimates = {}
        self.WIDTH_MAX_AGE_SEC = 1.0

        # Last selected lane (1 or 2)
        self.last_lane = None
        # A commanded target overrides nearest-lane selection. This lets the
        # controller follow the destination lane center throughout a lane
        # change instead of remaining locked to the lane being left.
        self.requested_lane = None
        self.lane_change_active = False

        self.create_timer(.1, self.check_timeout)
        self.get_logger().info(f"LaneDetector initialized: {cam_topic}, {det_topic}")

    def det_callback(self, det_msg: DetectionArray):
        self.accept_frame(det_msg, image=False)

    def image_callback(self, img_msg: Image):
        self.accept_frame(img_msg, image=True)

    def accept_frame(self, message, image):
        pair = self.frame_pairs.add(message, time.monotonic(), image=image)
        if self.stream_generation != self.frame_pairs.generation:
            self.stream_generation = self.frame_pairs.generation
            self.lane_width_estimates.clear()
            self.last_lane = None
            self.last_source_at = None
            self.publish_fallback()
        if pair is not None:
            source, detection, received = pair
            self.last_source_at = received
            self.process(source, detection)

    def check_timeout(self):
        if self.last_source_at is None or time.monotonic() - self.last_source_at > self.frame_pairs.max_age:
            self.lane_width_estimates.clear()
            self.publish_fallback()
            self.publish_geometry(None, None, 'stale')

    def target_lane_callback(self, msg: Int32):
        lane = int(msg.data)
        if lane not in (1, 2):
            self.get_logger().warn(f"Ignoring invalid target lane: {lane}")
            return
        self.requested_lane = lane

    def lane_change_callback(self, msg: Bool):
        self.lane_change_active = bool(msg.data)

    def publish_fallback(self):
        # Zero means invalid observation. The motion node stops on this value.
        self.lane_info_pub.publish(LaneInfo())

    def publish_geometry(self, geometry, lane, reason='', valid=False, source_stamp=0, control_lane=None):
        observed = geometry.observed_rows if geometry else 0
        inferred = geometry.inferred_rows if geometry else 0
        quality = 'inferred' if inferred else ('observed' if observed else 'unavailable')
        self.geometry_pub.publish(String(data=json.dumps({
            'quality': quality, 'observed_rows': observed, 'inferred_rows': inferred,
            'lane': lane, 'control_valid': valid, 'reason': reason,
            'changing_lane': self.lane_change_active, 'source_stamp': source_stamp,
            'control_lane': control_lane,
        })))

    @staticmethod
    def draw_center(frame, segment):
        # Solid = two measured edges; dashed = reconstructed from one edge.
        # Never join disjoint segments or draw a remembered line from old frames.
        points = segment[:, :2].astype(np.int32)
        inferred = segment[:, 2].astype(bool)
        runs = []
        start = None
        for i in range(len(points)):
            visible = not inferred[i] or int(points[i, 1]) % 18 < 11
            if visible and start is None:
                start = i
            if start is not None and (not visible or i == len(points) - 1):
                end = i if not visible else i + 1
                if end - start >= 2:
                    runs.append(points[start:end])
                start = None
        for curve in runs:
            cv2.polylines(frame, [curve], False, VIZ_DARK_HALO, 7, cv2.LINE_AA)
            cv2.polylines(frame, [curve], False, VIZ_SELECTED, 4, cv2.LINE_AA)

    def process(self, img_msg: Image, det_msg: DetectionArray):
        try:
            frame = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
            h, w = frame.shape[:2]
            if self.frame_size != (w, h):
                self.frame_size = (w, h)
                self.lane_width_estimates.clear()
            masks = {lane: np.zeros((h, w), np.uint8) for lane in (1, 2)}
            for det in det_msg.detections:
                if det.class_name not in ("lane1", "lane2"):
                    continue
                pts = np.asarray([[p.x, p.y] for p in det.mask.data], dtype=float)
                if len(pts) < 3 or not np.isfinite(pts).all():
                    continue
                # Mask coordinates refer to their declared source resolution.
                pts *= [w / float(det.mask.width or w), h / float(det.mask.height or h)]
                pts = np.rint(pts).astype(np.int32)
                pts[:, 0] = np.clip(pts[:, 0], 0, w - 1)
                pts[:, 1] = np.clip(pts[:, 1], 0, h - 1)
                cv2.fillPoly(masks[int(det.class_name[-1])], [pts], 255)
            src = np.asarray(self.get_parameter("persp_src").value, np.float32).reshape(4, 2) * [w, h]
            dst = np.asarray(self.get_parameter("persp_dst").value, np.float32).reshape(4, 2) * [w, h]
            matrix = cv2.getPerspectiveTransform(src.astype(np.float32), dst.astype(np.float32))
            road = cv2.warpPerspective(frame, matrix, (w, h), flags=cv2.INTER_LINEAR)
            valid = cv2.warpPerspective(np.ones((h, w), np.uint8), matrix, (w, h),
                                        flags=cv2.INTER_NEAREST).astype(bool)
            kernel_size = max(3, int(round(w / 160)) | 1)
            kernel = np.ones((kernel_size, kernel_size), np.uint8)
            geometry, infos, warped = {}, {}, {}
            now = time.monotonic()
            for lane in (1, 2):
                mask = cv2.warpPerspective(masks[lane], matrix, (w, h), flags=cv2.INTER_NEAREST)
                mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
                mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
                warped[lane] = mask
                width, measured_at = self.lane_width_estimates.get(lane, (None, 0))
                if now - measured_at > self.WIDTH_MAX_AGE_SEC:
                    width = None
                result = extract_lane(mask, valid, width)
                if result.measured_width is not None:
                    self.lane_width_estimates[lane] = (result.measured_width, now)
                geometry[lane] = result
                control = control_from_geometry(result, w, h, lane)
                if control is not None:
                    infos[lane] = control

            selected = None
            # Follow the target when it can be measured. During an explicit
            # lane change, retain the current lane's valid observation until
            # the destination enters view: MotionNode uses the lane number to
            # distinguish acquisition from following the destination center.
            if self.requested_lane in (1, 2):
                selected = self.requested_lane if self.requested_lane in infos else None
                if selected is None and self.lane_change_active and infos:
                    selected = self.last_lane if self.last_lane in infos else min(infos, key=lambda lane: abs(infos[lane][1]))
            elif self.LOCK_SELECTED_LANE and self.last_lane in infos:
                selected = self.last_lane
            elif infos:
                selected = min(infos, key=lambda lane: abs(infos[lane][1]))
            if selected is None:
                self.publish_fallback()
            else:
                angle, offset = infos[selected]
                msg = LaneInfo()
                msg.steering_angle = angle
                msg.lane_num = selected
                msg.vehicle_position_x = int(round(offset * self.CONTROL_REFERENCE_WIDTH / w))
                self.lane_info_pub.publish(msg)
                self.last_lane = selected
            display_lane = (self.requested_lane if self.lane_change_active else selected) or self.requested_lane or self.last_lane or 2
            result = geometry[display_lane]
            reason = '' if selected else ('insufficient_near_field' if result.segments else 'missing_edges_or_width')
            if selected is not None and selected != display_lane:
                reason = 'acquiring_target'
            self.publish_geometry(result, display_lane, reason, selected is not None, stamp_key(img_msg), selected)

            overlay = road.copy()
            overlay[warped[1] > 0] = VIZ_LANE1
            overlay[warped[2] > 0] = VIZ_LANE2
            bev_stage = cv2.addWeighted(road, 1 - VIZ_MASK_ALPHA, overlay, VIZ_MASK_ALPHA, 0)
            bev_stage[~valid] = (bev_stage[~valid] * .18).astype(np.uint8)
            bev_msg = self.bridge.cv2_to_imgmsg(bev_stage, "bgr8")
            bev_msg.header = img_msg.header
            self.bev_pub.publish(bev_msg)
            fitting = (road * .72).astype(np.uint8)
            fitting[~valid] = (fitting[~valid] * .16).astype(np.uint8)
            # Even during a lane change this is the destination lane center,
            # not a planned vehicle trajectory. The UI explicitly names it.
            for segment in result.segments:
                self.draw_center(fitting, segment)
            for edge in (result.left, result.right):
                for x, y in edge[::max(1, len(edge) // 22)]:
                    cv2.circle(fitting, (int(x), int(y)), 2, VIZ_POINT, -1, cv2.LINE_AA)
            msg = self.bridge.cv2_to_imgmsg(fitting, "bgr8")
            msg.header = img_msg.header
            self.viz_pub.publish(msg)
        except Exception as exc:
            self.get_logger().error(f"LaneDetector exception: {type(exc).__name__}: {exc}")
            self.publish_fallback()
            self.publish_geometry(None, self.requested_lane, 'processing_error')


def main(args=None):
    rclpy.init(args=args)
    node = LaneDetector()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        node.get_logger().info("Interrupted")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
