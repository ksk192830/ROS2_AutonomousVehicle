"""정지선 감지 노드 (StopLineNode).

YOLO 검출 중 'stop_line' 클래스 존재 여부를 로그로만 출력하는 실험용 노드.
최종 대회 launch에는 포함되지 않았다 (신호등 정지는 traffic_light 면적 기준으로 대체).
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from interfaces_pkg.msg import DetectionArray
from message_filters import Subscriber, ApproximateTimeSynchronizer


class StopLineNode(Node):
    def __init__(self):
        super().__init__('stop_line')

        self.bridge = CvBridge()

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )

        self.image_sub = Subscriber(self, Image, '/cam0/image_raw', qos_profile=qos)
        self.detection_sub = Subscriber(self, DetectionArray, '/cam0/detections', qos_profile=qos)

        self.ts = ApproximateTimeSynchronizer(
            [self.image_sub, self.detection_sub],
            queue_size=10,
            slop=0.5
        )
        self.ts.registerCallback(self.sync_callback)

    def sync_callback(self, img_msg, det_msg):
        image = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')

        found_stop_line = False
        for det in det_msg.detections:
            if det.class_name == 'stop_line':
                found_stop_line = True
                break

        if found_stop_line:
            self.get_logger().info("🟥 Stop line detected!")
        


def main(args=None):
    rclpy.init(args=args)
    node = StopLineNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
