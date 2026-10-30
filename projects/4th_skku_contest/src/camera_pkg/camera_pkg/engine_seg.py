import os

import cv2
import numpy as np
import rclpy
import torch
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from sensor_msgs.msg import Image

from interfaces_pkg.msg import (
    BoundingBox2D,
    Detection,
    DetectionArray,
    Mask,
    Point2D,
    State,
)


IMAGENET_MEAN = (0.485, 0.456, 0.406)
IMAGENET_STD = (0.229, 0.224, 0.225)
REQUIRED_OUTPUTS = {"dets", "labels", "masks"}


def color_for_id(idx: int) -> tuple:
    hue = (idx * 37) % 180
    hsv = np.uint8([[[hue, 255, 255]]])
    bgr = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)[0, 0]
    return int(bgr[0]), int(bgr[1]), int(bgr[2])


class TensorRTEngine:
    """Run the static RF-DETR TensorRT segmentation engine."""

    def __init__(self, engine_path: str, device: str):
        try:
            import tensorrt as trt
        except ImportError as exc:
            raise RuntimeError("TensorRT Python package is not available") from exc

        if not torch.cuda.is_available():
            raise RuntimeError("TensorRT requires an available CUDA GPU")
        if not device.startswith("cuda"):
            raise RuntimeError(f"TensorRT does not support device '{device}'")

        self.trt = trt
        self.device = torch.device(device)
        torch.cuda.set_device(self.device)

        self.logger = trt.Logger(trt.Logger.ERROR)
        self.runtime = trt.Runtime(self.logger)
        with open(engine_path, "rb") as engine_file:
            self.engine = self.runtime.deserialize_cuda_engine(engine_file.read())
        if self.engine is None:
            raise RuntimeError(f"Could not deserialize TensorRT engine: {engine_path}")

        self.context = self.engine.create_execution_context()
        if self.context is None:
            raise RuntimeError("Could not create TensorRT execution context")

        self.input_name = None
        self.input = None
        self.outputs = {}

        for index in range(self.engine.num_io_tensors):
            name = self.engine.get_tensor_name(index)
            shape = tuple(self.engine.get_tensor_shape(name))
            if any(dimension < 0 for dimension in shape):
                raise RuntimeError(f"Dynamic tensor is not supported: {name}={shape}")

            numpy_dtype = np.dtype(trt.nptype(self.engine.get_tensor_dtype(name)))
            torch_dtype = torch.from_numpy(np.empty((), dtype=numpy_dtype)).dtype
            tensor = torch.empty(shape, dtype=torch_dtype, device=self.device)
            self.context.set_tensor_address(name, tensor.data_ptr())

            if self.engine.get_tensor_mode(name) == trt.TensorIOMode.INPUT:
                if self.input_name is not None:
                    raise RuntimeError("The engine must have exactly one input")
                self.input_name = name
                self.input = tensor
            else:
                self.outputs[name] = tensor

        if self.input is None or not REQUIRED_OUTPUTS.issubset(self.outputs):
            raise RuntimeError(
                "Expected one input and dets/labels/masks outputs, got "
                f"input={self.input_name}, outputs={sorted(self.outputs)}"
            )
        if self.input.ndim != 4 or self.input.shape[0] != 1:
            raise RuntimeError(f"Expected static NCHW batch 1, got {self.input.shape}")

        self.input_height = int(self.input.shape[2])
        self.input_width = int(self.input.shape[3])
        self.mean = torch.tensor(
            IMAGENET_MEAN, dtype=self.input.dtype, device=self.device
        ).view(3, 1, 1)
        self.std = torch.tensor(
            IMAGENET_STD, dtype=self.input.dtype, device=self.device
        ).view(3, 1, 1)

    def predict(self, bgr_image: np.ndarray, threshold: float):
        height, width = bgr_image.shape[:2]
        resized = cv2.resize(
            bgr_image, (self.input_width, self.input_height),
            interpolation=cv2.INTER_LINEAR,
        )
        rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
        tensor = torch.from_numpy(np.ascontiguousarray(rgb.transpose(2, 0, 1)))
        tensor = tensor.to(device=self.device, dtype=self.input.dtype) / 255.0
        tensor = (tensor - self.mean) / self.std
        self.input.copy_(tensor.unsqueeze(0))

        stream = torch.cuda.current_stream(self.device)
        if not self.context.execute_async_v3(stream.cuda_stream):
            raise RuntimeError("TensorRT execute_async_v3 failed")
        stream.synchronize()

        boxes = self.outputs["dets"][0]
        logits = self.outputs["labels"][0]
        masks = self.outputs["masks"][0]

        probabilities = logits.sigmoid()
        scores, class_ids = torch.max(probabilities, dim=1)
        count = min(300, scores.numel())
        scores, query_indexes = torch.topk(scores, count)
        class_ids = class_ids[query_indexes]
        keep = scores > threshold

        scores = scores[keep]
        query_indexes = query_indexes[keep]
        class_ids = class_ids[keep]
        selected_boxes = boxes[query_indexes]
        selected_masks = masks[query_indexes]

        cx, cy, box_width, box_height = selected_boxes.unbind(1)
        xyxy = torch.stack(
            (
                (cx - box_width / 2.0) * width,
                (cy - box_height / 2.0) * height,
                (cx + box_width / 2.0) * width,
                (cy + box_height / 2.0) * height,
            ),
            dim=1,
        )

        return (
            xyxy.float().cpu().numpy(),
            scores.float().cpu().numpy(),
            class_ids.cpu().numpy(),
            selected_masks.float().cpu().numpy(),
        )


class EngineSegNode(Node):
    def __init__(self):
        super().__init__("engine_seg")

        default_model_path = os.path.join(
            get_package_share_directory("camera_pkg"),
            "model",
            "parking_front.engine",
        )
        self.declare_parameter("model_path", default_model_path)
        self.declare_parameter("device", "cuda:0")
        self.declare_parameter("threshold", 0.3)
        self.declare_parameter("mask_fill", True)
        self.declare_parameter("mask_alpha", 0.25)
        self.declare_parameter("color_by", "class")
        self.declare_parameter("allowed_states", ["__ALL__"])
        self.declare_parameter(
            "class_names",
            ["front", "out_line", "parking_lot", "parking_space"],
        )

        self.model_path = self.get_parameter("model_path").value
        self.device = self.get_parameter("device").value
        self.threshold = float(self.get_parameter("threshold").value)
        self.mask_fill = bool(self.get_parameter("mask_fill").value)
        self.mask_alpha = float(self.get_parameter("mask_alpha").value)
        self.color_by = self.get_parameter("color_by").value
        self.allowed_states = list(self.get_parameter("allowed_states").value)
        self.class_names = self._read_class_names()
        self.current_state = None
        self.bridge = CvBridge()
        self.model = None

        self.get_logger().info(
            f"engine_seg config: model_path={self.model_path}, "
            f"class_names={self.class_names}"
        )

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1,
        )
        self.publisher = self.create_publisher(DetectionArray, "detections", qos)
        self.subscription = self.create_subscription(
            Image, "image_raw", self.image_cb, qos
        )
        self.vis_pub = self.create_publisher(Image, "seg_vis", qos)
        self.state_sub = self.create_subscription(
            State, "/motion_state", self.state_cb, qos
        )

        if not os.path.exists(self.model_path):
            self.get_logger().error(f"Model not found: {self.model_path}")
            return

        try:
            self.model = TensorRTEngine(self.model_path, self.device)
            self.get_logger().info(
                f"TensorRT model loaded from {self.model_path} "
                f"(input={self.model.input_name}, "
                f"size={self.model.input_width}x{self.model.input_height})"
            )
        except Exception as exc:
            self.get_logger().error(f"Failed to load TensorRT model: {exc}")

    def _read_class_names(self):
        value = self.get_parameter("class_names").value
        if isinstance(value, str):
            names = [name.strip() for name in value.split(",") if name.strip()]
        else:
            names = [str(name) for name in value]

        if (
            os.path.basename(str(self.get_parameter("model_path").value)) == "track.engine"
            and names == ["front", "out_line", "parking_lot", "parking_space"]
        ):
            names = [
                "background",
                "car",
                "crosswalk",
                "lane1",
                "lane2",
                "traffic_light",
                "car",
            ]

        return names

    def state_cb(self, msg: State):
        self.current_state = msg.state

    def _publish_empty(self, msg: Image, cv_image: np.ndarray):
        detections_msg = DetectionArray()
        detections_msg.header = msg.header
        self.publisher.publish(detections_msg)
        vis_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
        vis_msg.header = msg.header
        self.vis_pub.publish(vis_msg)

    def _class_name(self, class_id: int) -> str:
        if 0 <= class_id < len(self.class_names):
            return self.class_names[class_id]
        return f"class_{class_id}"

    @staticmethod
    def _mask_polygon(mask_logits: np.ndarray, width: int, height: int):
        mask = cv2.resize(
            mask_logits, (width, height), interpolation=cv2.INTER_LINEAR
        )
        binary = (mask > 0.0).astype(np.uint8)
        contours, _ = cv2.findContours(
            binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        if not contours:
            return None
        contour = max(contours, key=cv2.contourArea)
        if len(contour) < 3:
            return None
        return contour.reshape(-1, 2)

    def image_cb(self, msg: Image):
        if self.model is None:
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().error(f"Image conversion failed: {exc}")
            return

        if (
            self.current_state is not None
            and "__ALL__" not in self.allowed_states
            and self.current_state not in self.allowed_states
        ):
            self._publish_empty(msg, cv_image)
            return

        try:
            boxes, scores, class_ids, mask_logits = self.model.predict(
                cv_image, self.threshold
            )
            height, width = cv_image.shape[:2]
            detections_msg = DetectionArray()
            detections_msg.header = msg.header
            vis = cv_image.copy()

            for index, (box, score, class_id, raw_mask) in enumerate(
                zip(boxes, scores, class_ids, mask_logits)
            ):
                class_id = int(class_id)
                class_name = self._class_name(class_id)
                x1, y1, x2, y2 = box
                x1 = float(np.clip(x1, 0, width - 1))
                y1 = float(np.clip(y1, 0, height - 1))
                x2 = float(np.clip(x2, 0, width - 1))
                y2 = float(np.clip(y2, 0, height - 1))
                polygon = self._mask_polygon(raw_mask, width, height)

                det = Detection()
                det.class_id = class_id
                det.class_name = class_name
                det.score = float(score)
                det.bbox = BoundingBox2D()
                det.bbox.center.position.x = (x1 + x2) / 2.0
                det.bbox.center.position.y = (y1 + y2) / 2.0
                det.bbox.size.x = max(0.0, x2 - x1)
                det.bbox.size.y = max(0.0, y2 - y1)

                if polygon is not None:
                    mask_msg = Mask()
                    mask_msg.width = width
                    mask_msg.height = height
                    for point_x, point_y in polygon:
                        point = Point2D()
                        point.x = float(point_x)
                        point.y = float(point_y)
                        mask_msg.data.append(point)
                    det.mask = mask_msg

                detections_msg.detections.append(det)

                color_index = index if self.color_by == "instance" else class_id
                color = color_for_id(color_index)
                cv2.rectangle(
                    vis, (int(x1), int(y1)), (int(x2), int(y2)), color, 2
                )
                label = f"{class_name}:{float(score):.2f}"
                (text_width, text_height), _ = cv2.getTextSize(
                    label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1
                )
                top = max(0, int(y1) - text_height - 8)
                cv2.rectangle(
                    vis,
                    (int(x1), top),
                    (int(x1) + text_width + 4, int(y1)),
                    color,
                    -1,
                )
                cv2.putText(
                    vis,
                    label,
                    (int(x1) + 2, max(0, int(y1) - 5)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 0, 0),
                    1,
                    cv2.LINE_AA,
                )

                if polygon is not None:
                    points = polygon.astype(np.int32).reshape((-1, 1, 2))
                    if self.mask_fill:
                        overlay = vis.copy()
                        cv2.fillPoly(overlay, [points], color)
                        vis = cv2.addWeighted(
                            overlay,
                            self.mask_alpha,
                            vis,
                            1.0 - self.mask_alpha,
                            0,
                        )
                    cv2.polylines(
                        vis, [points], isClosed=True, color=color, thickness=2
                    )

            self.publisher.publish(detections_msg)
            vis_msg = self.bridge.cv2_to_imgmsg(vis, encoding="bgr8")
            vis_msg.header = msg.header
            self.vis_pub.publish(vis_msg)
        except Exception as exc:
            self.get_logger().error(
                f"TensorRT inference failed: {type(exc).__name__}: {exc}",
                throttle_duration_sec=2.0,
            )


def main(args=None):
    rclpy.init(args=args)
    node = EngineSegNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
