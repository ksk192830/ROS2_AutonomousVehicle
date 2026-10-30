"""GPU-first RF-DETR segmentation node for the general driving mission."""

from __future__ import annotations

import math
import json
import os
import time
import warnings

import cv2
import numpy as np
import rclpy
import torch
import torch.nn.functional as torch_f
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from sensor_msgs.msg import Image
from std_msgs.msg import String

from interfaces_pkg.msg import (
    BoundingBox2D,
    Detection,
    DetectionArray,
    Mask,
    Point2D,
    State,
)


# The class order is embedded in general_mission(with_crosswalk).pth. Keeping
# the same names is important because lane_detect consumes lane1 and lane2.
DEFAULT_CLASS_NAMES = [
    "driv-car-Road-fPZR-lane2-lane2-road-object",
    "car",
    "crosswalk",
    "lane1",
    "lane2",
    "traffic_light",
]

# RGB colors used by the GPU overlay. Lane colors match the exhibition BEV.
CLASS_COLORS_RGB = [
    (48, 150, 235),
    (245, 82, 76),
    (250, 205, 55),
    (35, 150, 255),
    (235, 70, 185),
    (80, 225, 115),
]


class PthSegNode(Node):
    """Run RF-DETR segmentation while keeping the image pipeline on CUDA."""

    def __init__(self) -> None:
        super().__init__("pth_seg")

        default_model = os.path.join(
            os.path.dirname(__file__),
            "model",
            "general_mission_with_crosswalk.pth",
        )
        self.declare_parameter("model_path", default_model)
        self.declare_parameter("device", "cuda:0")
        self.declare_parameter("threshold", 0.35)
        self.declare_parameter("use_half", True)
        self.declare_parameter("max_det", 20)
        self.declare_parameter("mask_alpha", 0.38)
        self.declare_parameter("min_mask_area", 36.0)
        self.declare_parameter("polygon_epsilon", 0.0025)
        self.declare_parameter("polygon_max_points", 180)
        self.declare_parameter("max_contours_per_detection", 4)
        self.declare_parameter("warmup_runs", 2)
        self.declare_parameter("cpu_threads", 1)
        self.declare_parameter("allowed_states", ["__ALL__"])
        self.declare_parameter('exhibition_role', '')
        self.exhibition_role = str(self.get_parameter('exhibition_role').value)
        self.exhibition_active = self.exhibition_role != 'opponent'
        if self.exhibition_role:
            self.create_subscription(String, '/race/status', self.exhibition_callback,
                QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL))

        self.model_path = str(self.get_parameter("model_path").value)
        requested_device = str(self.get_parameter("device").value)
        self.threshold = float(self.get_parameter("threshold").value)
        self.use_half = bool(self.get_parameter("use_half").value)
        self.max_det = max(1, int(self.get_parameter("max_det").value))
        self.mask_alpha = float(np.clip(self.get_parameter("mask_alpha").value, 0.0, 1.0))
        self.min_mask_area = max(1.0, float(self.get_parameter("min_mask_area").value))
        self.polygon_epsilon = max(0.0, float(self.get_parameter("polygon_epsilon").value))
        self.polygon_max_points = max(8, int(self.get_parameter("polygon_max_points").value))
        self.max_contours = max(
            1, int(self.get_parameter("max_contours_per_detection").value)
        )
        self.warmup_runs = max(0, int(self.get_parameter("warmup_runs").value))
        self.cpu_threads = max(1, int(self.get_parameter("cpu_threads").value))
        self.allowed_states = list(self.get_parameter("allowed_states").value)
        self.current_state = None

        # This node has no CPU neural-network path when CUDA is active. Capping
        # the libraries' host pools prevents tiny contour/dispatch work from
        # waking many cores and competing with Gazebo.
        cv2.setNumThreads(self.cpu_threads)
        torch.set_num_threads(self.cpu_threads)
        try:
            torch.set_num_interop_threads(1)
        except RuntimeError:
            # Another imported PyTorch component may already have initialized
            # the inter-op pool; intra-op capping above is still effective.
            pass

        if requested_device.startswith("cuda") and not torch.cuda.is_available():
            self.get_logger().warning(
                f"CUDA device {requested_device!r} is unavailable; using CPU."
            )
            requested_device = "cpu"
        self.device = torch.device(requested_device)
        self.use_half = self.use_half and self.device.type == "cuda"
        self.inference_dtype = torch.float16 if self.use_half else torch.float32

        if self.device.type == "cuda":
            torch.backends.cudnn.benchmark = True
            torch.backends.cuda.matmul.allow_tf32 = True
            torch.backends.cudnn.allow_tf32 = True
            torch.set_float32_matmul_precision("high")

        self.bridge = CvBridge()
        self._host_frame = None
        self._host_frame_shape = None
        self._frames = 0
        self._last_stats_time = time.perf_counter()
        self._latency_ema_ms = None

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
        self.detection_pub = self.create_publisher(
            DetectionArray, "detections", reliable_qos
        )
        self.vis_pub = self.create_publisher(Image, "seg_vis", image_qos)
        self.image_sub = self.create_subscription(
            Image, "image_raw", self.image_callback, image_qos
        )
        self.state_sub = self.create_subscription(
            State, "/motion_state", self.state_callback, reliable_qos
        )

        self.runtime = None
        self.core_model = None
        self.class_names = list(DEFAULT_CLASS_NAMES)
        self._load_model()

    def _load_model(self) -> None:
        if not os.path.isfile(self.model_path):
            raise FileNotFoundError(f"RF-DETR checkpoint not found: {self.model_path}")

        # RF-DETR 1.8.1 emits deprecation warnings for legacy aliases while it
        # resolves this checkpoint. They do not indicate an inference problem.
        with warnings.catch_warnings():
            warnings.filterwarnings("ignore", category=FutureWarning, module="rfdetr")
            from rfdetr import RFDETR

            self.runtime = RFDETR.from_checkpoint(
                self.model_path,
                device=str(self.device),
                num_classes=len(DEFAULT_CLASS_NAMES),
            )

        checkpoint_names = list(self.runtime.class_names)
        if len(checkpoint_names) == len(DEFAULT_CLASS_NAMES):
            self.class_names = checkpoint_names
        else:
            self.get_logger().warning(
                "Checkpoint class metadata is missing or has an unexpected length; "
                "using the configured general-mission class order."
            )

        self.resolution = int(self.runtime.model.resolution)
        self.core_model = self.runtime.model.model
        self.core_model.eval()
        self.core_model.export()
        self.core_model.requires_grad_(False)
        self.core_model.to(device=self.device, dtype=self.inference_dtype)

        self.mean = torch.tensor(
            self.runtime.means,
            device=self.device,
            dtype=self.inference_dtype,
        ).view(1, 3, 1, 1)
        self.std = torch.tensor(
            self.runtime.stds,
            device=self.device,
            dtype=self.inference_dtype,
        ).view(1, 3, 1, 1)
        self.color_table = torch.tensor(
            CLASS_COLORS_RGB,
            device=self.device,
            dtype=torch.float32,
        )

        self._warm_up()
        self.get_logger().info(
            "RF-DETR segmentation ready: "
            f"device={self.device}, dtype={self.inference_dtype}, "
            f"resolution={self.resolution}, classes={self.class_names}"
        )

    def _warm_up(self) -> None:
        if self.warmup_runs == 0:
            return
        dummy = torch.zeros(
            (1, 3, self.resolution, self.resolution),
            device=self.device,
            dtype=self.inference_dtype,
        )
        with torch.inference_mode():
            for _ in range(self.warmup_runs):
                outputs = self._prediction_dict(self.core_model(dummy))
                self._postprocess_selected(
                    outputs,
                    self.resolution,
                    self.resolution,
                )
        if self.device.type == "cuda":
            torch.cuda.synchronize(self.device)

    @staticmethod
    def _prediction_dict(outputs):
        if not isinstance(outputs, tuple):
            return outputs
        predictions = {
            "pred_boxes": outputs[0],
            "pred_logits": outputs[1],
        }
        if len(outputs) >= 3:
            predictions["pred_masks"] = outputs[2]
        return predictions

    def state_callback(self, msg: State) -> None:
        self.current_state = msg.state

    def _state_is_allowed(self) -> bool:
        if self.current_state is None:
            return True
        return "__ALL__" in self.allowed_states or self.current_state in self.allowed_states

    def _publish_passthrough(self, msg: Image) -> None:
        detections = DetectionArray()
        detections.header = msg.header
        self.detection_pub.publish(detections)
        self.vis_pub.publish(msg)

    def _copy_frame_to_device(self, frame: np.ndarray) -> torch.Tensor:
        frame = np.ascontiguousarray(frame)
        source = torch.from_numpy(frame)
        if self.device.type != "cuda":
            return source.to(self.device)

        if self._host_frame_shape != frame.shape:
            self._host_frame = torch.empty(
                frame.shape,
                dtype=torch.uint8,
                pin_memory=True,
            )
            self._host_frame_shape = frame.shape
        self._host_frame.copy_(source)
        return self._host_frame.to(self.device, non_blocking=True)

    def _preprocess(self, bgr_gpu: torch.Tensor) -> torch.Tensor:
        # Channel conversion, uint8 scaling, resize, and normalization all run
        # on the GPU. Only cv_bridge and the pinned host copy remain on CPU.
        image = bgr_gpu.flip(-1).permute(2, 0, 1).unsqueeze(0)
        image = image.to(dtype=self.inference_dtype).mul_(1.0 / 255.0)
        image = torch_f.interpolate(
            image,
            size=(self.resolution, self.resolution),
            mode="bilinear",
            align_corners=False,
            antialias=True,
        )
        return image.sub_(self.mean).div_(self.std)

    def _postprocess_selected(
        self,
        predictions: dict,
        height: int,
        width: int,
    ) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        """Filter query/class scores before expanding masks to camera size.

        RF-DETR's general postprocessor first enlarges its top 300 masks and
        only then lets the caller apply a threshold. A driving frame normally
        retains two lane masks, so selecting on CUDA first removes most of the
        memory traffic and interpolation work.
        """
        logits = predictions["pred_logits"][0]
        flat_scores = logits.sigmoid().flatten()
        candidate_indexes = torch.nonzero(
            flat_scores > self.threshold,
            as_tuple=False,
        ).flatten()
        if candidate_indexes.numel() == 0:
            return (
                flat_scores.new_empty((0,)),
                torch.empty((0,), device=self.device, dtype=torch.long),
                torch.empty(
                    (0, height, width),
                    device=self.device,
                    dtype=torch.bool,
                ),
            )

        candidate_scores = flat_scores[candidate_indexes]
        keep_count = min(self.max_det, candidate_scores.numel())
        scores, order = torch.topk(candidate_scores, keep_count, sorted=True)
        selected = candidate_indexes[order]
        class_count = logits.shape[-1]
        query_indexes = torch.div(selected, class_count, rounding_mode="floor")
        labels = selected.remainder(class_count)

        mask_logits = predictions.get("pred_masks")
        if mask_logits is None:
            masks = torch.empty(
                (0, height, width),
                device=self.device,
                dtype=torch.bool,
            )
        else:
            masks = mask_logits[0, query_indexes].unsqueeze(1)
            masks = torch_f.interpolate(
                masks,
                size=(height, width),
                mode="bilinear",
                align_corners=False,
            )[:, 0] > 0.0
        return scores, labels, masks

    def _gpu_visualization(
        self,
        bgr_gpu: torch.Tensor,
        scores: torch.Tensor,
        labels: torch.Tensor,
        masks: torch.Tensor,
    ) -> np.ndarray:
        if masks.numel() == 0:
            return bgr_gpu.cpu().numpy().copy()

        priority = masks.to(dtype=torch.float32) * scores.float().view(-1, 1, 1)
        winner = priority.argmax(dim=0)
        valid = masks.any(dim=0)
        color_ids = labels.long().clamp_(0, self.color_table.shape[0] - 1)
        colors = self.color_table[color_ids][winner]
        rgb = bgr_gpu.flip(-1).to(dtype=torch.float32)
        blended = rgb.mul(1.0 - self.mask_alpha).add_(colors, alpha=self.mask_alpha)
        rgb = torch.where(valid.unsqueeze(-1), blended, rgb)
        return rgb.flip(-1).clamp_(0, 255).to(torch.uint8).cpu().numpy()

    def _simplify_contour(self, contour: np.ndarray) -> np.ndarray | None:
        perimeter = cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(
            contour,
            self.polygon_epsilon * perimeter,
            True,
        ).reshape(-1, 2)
        if approx.shape[0] < 3:
            return None
        if approx.shape[0] > self.polygon_max_points:
            stride = math.ceil(approx.shape[0] / self.polygon_max_points)
            approx = approx[::stride]
        return approx

    def _append_detection(
        self,
        output: DetectionArray,
        class_id: int,
        score: float,
        polygon: np.ndarray,
        width: int,
        height: int,
    ) -> None:
        detection = Detection()
        detection.class_id = class_id
        detection.class_name = (
            self.class_names[class_id]
            if 0 <= class_id < len(self.class_names)
            else f"class_{class_id}"
        )
        detection.score = score

        x, y, box_w, box_h = cv2.boundingRect(polygon.reshape(-1, 1, 2))
        detection.bbox = BoundingBox2D()
        detection.bbox.center.position.x = float(x + box_w * 0.5)
        detection.bbox.center.position.y = float(y + box_h * 0.5)
        detection.bbox.size.x = float(box_w)
        detection.bbox.size.y = float(box_h)

        mask = Mask()
        mask.width = width
        mask.height = height
        for x_coord, y_coord in polygon:
            point = Point2D()
            point.x = float(x_coord)
            point.y = float(y_coord)
            mask.data.append(point)
        detection.mask = mask
        output.detections.append(detection)

    def _messages_and_outlines(
        self,
        header,
        scores: torch.Tensor,
        labels: torch.Tensor,
        masks: torch.Tensor,
        width: int,
        height: int,
    ) -> tuple[DetectionArray, list[tuple[np.ndarray, int]]]:
        output = DetectionArray()
        output.header = header
        outlines: list[tuple[np.ndarray, int]] = []
        if masks.numel() == 0:
            return output, outlines

        scores_cpu = scores.float().cpu().numpy()
        labels_cpu = labels.long().cpu().numpy()
        masks_cpu = masks.to(torch.uint8).cpu().numpy()

        for mask_array, label, score in zip(
            masks_cpu,
            labels_cpu,
            scores_cpu,
            strict=True,
        ):
            contours, _ = cv2.findContours(
                mask_array,
                cv2.RETR_EXTERNAL,
                cv2.CHAIN_APPROX_SIMPLE,
            )
            contours = sorted(contours, key=cv2.contourArea, reverse=True)
            emitted = 0
            for contour in contours:
                if cv2.contourArea(contour) < self.min_mask_area:
                    continue
                polygon = self._simplify_contour(contour)
                if polygon is None:
                    continue
                class_id = int(label)
                self._append_detection(
                    output,
                    class_id,
                    float(score),
                    polygon,
                    width,
                    height,
                )
                outlines.append((polygon, class_id))
                emitted += 1
                if emitted >= self.max_contours:
                    break
        return output, outlines

    def _draw_outlines(
        self,
        visualization: np.ndarray,
        outlines: list[tuple[np.ndarray, int]],
    ) -> None:
        for polygon, class_id in outlines:
            rgb = CLASS_COLORS_RGB[class_id % len(CLASS_COLORS_RGB)]
            bgr = (rgb[2], rgb[1], rgb[0])
            cv2.polylines(
                visualization,
                [polygon.reshape(-1, 1, 2)],
                isClosed=True,
                color=bgr,
                thickness=2,
                lineType=cv2.LINE_AA,
            )

    def exhibition_callback(self, msg):
        try:
            status = json.loads(msg.data)
            duel = status.get('duel') or {}
            racing = (status.get('session') or {}).get('phase') in ('CHALLENGE', 'RESULTS')
            self.exhibition_active = racing if self.exhibition_role == 'opponent' else not racing
        except (ValueError, TypeError):
            return

    def image_callback(self, msg: Image) -> None:
        if not self.exhibition_active:
            return
        if self.core_model is None:
            return
        if not self._state_is_allowed():
            self._publish_passthrough(msg)
            return

        started = time.perf_counter()
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            height, width = frame.shape[:2]
            bgr_gpu = self._copy_frame_to_device(frame)

            with torch.inference_mode():
                inputs = self._preprocess(bgr_gpu)
                predictions = self._prediction_dict(self.core_model(inputs))
                scores, labels, masks = self._postprocess_selected(
                    predictions,
                    height,
                    width,
                )
                visualization = self._gpu_visualization(
                    bgr_gpu,
                    scores,
                    labels,
                    masks,
                )

            detection_msg, outlines = self._messages_and_outlines(
                msg.header,
                scores,
                labels,
                masks,
                width,
                height,
            )
            self._draw_outlines(visualization, outlines)
            self.detection_pub.publish(detection_msg)

            vis_msg = self.bridge.cv2_to_imgmsg(visualization, encoding="bgr8")
            vis_msg.header = msg.header
            self.vis_pub.publish(vis_msg)
            self._update_stats(started)
        except Exception as error:  # Keep the camera graph alive on one bad frame.
            self.get_logger().error(f"RF-DETR inference failed: {error}")

    def _update_stats(self, started: float) -> None:
        elapsed_ms = (time.perf_counter() - started) * 1000.0
        if self._latency_ema_ms is None:
            self._latency_ema_ms = elapsed_ms
        else:
            self._latency_ema_ms = 0.9 * self._latency_ema_ms + 0.1 * elapsed_ms
        self._frames += 1

        now = time.perf_counter()
        interval = now - self._last_stats_time
        if interval >= 5.0:
            fps = self._frames / interval
            self.get_logger().info(
                f"RF-DETR throughput={fps:.1f} FPS, "
                f"latency_ema={self._latency_ema_ms:.1f} ms"
            )
            self._frames = 0
            self._last_stats_time = now


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PthSegNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
