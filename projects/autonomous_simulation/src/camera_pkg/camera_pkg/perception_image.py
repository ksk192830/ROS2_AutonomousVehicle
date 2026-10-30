"""Keep inference small while the exhibition displays the full camera image."""
import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image


def resize_image(message, bridge, width, height):
    frame = bridge.imgmsg_to_cv2(message, 'bgr8')
    resized = cv2.resize(frame, (width, height), interpolation=cv2.INTER_AREA)
    result = bridge.cv2_to_imgmsg(resized, 'bgr8')
    # The exact source stamp is required by lane_detect.FramePairs. Never
    # stamp the resized image with the processing time or alter the FOV.
    result.header = message.header
    return result


class SourceRateGate:
    """Sample camera timestamps without increasing inference work with display FPS."""
    def __init__(self, rate):
        if rate <= 0:
            raise ValueError('Perception rate must be positive')
        self.period = max(1, round(1e9 / rate))
        self.next_at = None
        self.last_stamp = None

    def accept(self, stamp):
        if self.last_stamp is not None and stamp < self.last_stamp:
            self.next_at = None  # Simulator clock reset.
        self.last_stamp = stamp
        if self.next_at is None:
            self.next_at = stamp + self.period
            return True
        if stamp < self.next_at:
            return False
        # Keep cadence after jitter or dropped frames without publishing a burst.
        self.next_at += ((stamp - self.next_at) // self.period + 1) * self.period
        return True


class PerceptionImage(Node):
    def __init__(self):
        super().__init__('perception_image')
        self.declare_parameter('width', 480)
        self.declare_parameter('height', 270)
        self.declare_parameter('max_rate', 20.0)
        self.width = int(self.get_parameter('width').value)
        self.height = int(self.get_parameter('height').value)
        if self.width < 1 or self.height < 1:
            raise ValueError('Perception image dimensions must be positive')
        self.rate_gate = SourceRateGate(float(self.get_parameter('max_rate').value))
        cv2.setNumThreads(1)
        self.bridge = CvBridge()
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.publisher = self.create_publisher(Image, '/cam0/perception/image_raw', qos)
        self.create_subscription(Image, '/cam0/image_raw', self.on_image, qos)

    def on_image(self, message):
        stamp = message.header.stamp.sec * 10**9 + message.header.stamp.nanosec
        if not self.rate_gate.accept(stamp):
            return
        self.publisher.publish(resize_image(message, self.bridge, self.width, self.height))


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionImage()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
