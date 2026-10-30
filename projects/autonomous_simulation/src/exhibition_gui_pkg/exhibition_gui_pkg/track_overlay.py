"""Track-boundary extraction and display-only top-camera overlay."""

import os
from typing import List, Optional

import cv2
import numpy as np
from ament_index_python.packages import get_package_share_directory


class TrackBoundaryOverlay:
    """Extract the road ring from the source texture and draw its two edges."""

    TRACK_WIDTH_METERS = 16.0
    TRACK_HEIGHT_METERS = 12.0
    OUTER_BOUNDARY_MARGIN_METERS = 0.25
    INNER_BOUNDARY_MARGIN_METERS = 0.05

    def __init__(self, texture_path: Optional[str] = None):
        if texture_path is None:
            texture_path = os.path.join(
                get_package_share_directory('simulation_pkg'),
                'models',
                'race_track',
                'materials',
                'textures',
                'track.png',
            )
        self.texture_path = texture_path
        self.normalized_contours: List[np.ndarray] = []
        self._extract_boundaries()

    @property
    def available(self) -> bool:
        return len(self.normalized_contours) == 2

    @classmethod
    def world_to_normalized(cls, x: float, y: float) -> tuple:
        """Map a Gazebo ground-plane position into the top-camera image."""
        return (
            0.5 - float(y) / cls.TRACK_WIDTH_METERS,
            0.5 - float(x) / cls.TRACK_HEIGHT_METERS,
        )

    def contains_world_point(self, x: float, y: float) -> bool:
        """Return whether a vehicle centre is inside the displayed red ring."""
        if not self.available:
            return True
        point = self.world_to_normalized(x, y)
        outer = self.normalized_contours[0].astype(np.float32)
        inner = self.normalized_contours[1].astype(np.float32)
        inside_outer = cv2.pointPolygonTest(outer, point, False) >= 0.0
        outside_inner = cv2.pointPolygonTest(inner, point, False) <= 0.0
        return bool(inside_outer and outside_inner)

    @staticmethod
    def _smooth_closed_contour(
        contour: np.ndarray,
        sample_count: int,
    ) -> np.ndarray:
        """Arc-length resample and circularly smooth a closed pixel contour."""
        points = contour.reshape(-1, 2).astype(np.float32)
        if len(points) < 4:
            return contour.astype(np.float32)
        closed = np.vstack((points, points[0]))
        lengths = np.linalg.norm(np.diff(closed, axis=0), axis=1)
        cumulative = np.concatenate(([0.0], np.cumsum(lengths)))
        valid = np.concatenate(([True], np.diff(cumulative) > 1e-6))
        closed = closed[valid]
        cumulative = cumulative[valid]
        if cumulative[-1] <= 1e-6:
            return contour.astype(np.float32)

        targets = np.linspace(
            0.0,
            cumulative[-1],
            num=sample_count,
            endpoint=False,
            dtype=np.float32,
        )
        sampled = np.column_stack(
            (
                np.interp(targets, cumulative, closed[:, 0]),
                np.interp(targets, cumulative, closed[:, 1]),
            )
        ).astype(np.float32)

        offsets = np.arange(-5, 6, dtype=np.int32)
        weights = np.exp(-0.5 * (offsets.astype(np.float32) / 2.2) ** 2)
        weights /= weights.sum()
        smoothed = np.zeros_like(sampled)
        for offset, weight in zip(offsets, weights):
            smoothed += np.roll(sampled, int(offset), axis=0) * weight
        return smoothed.reshape(-1, 1, 2)

    @staticmethod
    def _snap_to_painted_edge(
        contour: np.ndarray,
        white_mask: np.ndarray,
        search_radius: float = 18.0,
    ) -> np.ndarray:
        """Move each contour point to the centre of the nearby white edge.

        The asphalt mask only tells us which side of the painted line the road
        is on.  Sampling perpendicular to the contour lets us use the actual
        solid white paint as the final geometric reference, including through
        tight curves.
        """
        points = contour.reshape(-1, 2).astype(np.float32)
        if len(points) < 7:
            return contour

        height, width = white_mask.shape
        offsets = np.linspace(
            -search_radius,
            search_radius,
            num=int(search_radius * 4.0) + 1,
            dtype=np.float32,
        )
        snapped = points.copy()
        for index, point in enumerate(points):
            tangent = points[(index + 3) % len(points)] - points[(index - 3) % len(points)]
            tangent_length = float(np.linalg.norm(tangent))
            if tangent_length <= 1e-6:
                continue
            normal = np.array([-tangent[1], tangent[0]], dtype=np.float32) / tangent_length
            samples = point[None, :] + offsets[:, None] * normal[None, :]
            sample_x = np.clip(np.rint(samples[:, 0]).astype(np.int32), 0, width - 1)
            sample_y = np.clip(np.rint(samples[:, 1]).astype(np.int32), 0, height - 1)
            hits = white_mask[sample_y, sample_x] > 0

            # Use the closest continuous white run, rather than an isolated
            # anti-aliased pixel or a nearby dashed lane marking.
            padded = np.pad(hits.astype(np.int8), (1, 1))
            transitions = np.diff(padded)
            starts = np.flatnonzero(transitions == 1)
            ends = np.flatnonzero(transitions == -1)
            candidates = []
            for start, end in zip(starts, ends):
                if end - start < 3:
                    continue
                centre_offset = float(offsets[start:end].mean())
                candidates.append((abs(centre_offset), centre_offset))
            if candidates:
                _, centre_offset = min(candidates, key=lambda item: item[0])
                snapped[index] = point + centre_offset * normal

        # Suppress one-pixel texture aliasing without pulling the line away
        # from its measured location on bends.
        snapped = (
            0.2 * np.roll(snapped, 1, axis=0)
            + 0.6 * snapped
            + 0.2 * np.roll(snapped, -1, axis=0)
        )
        return snapped.reshape(-1, 1, 2)

    @staticmethod
    def _offset_into_non_road(
        contour: np.ndarray,
        road_mask: np.ndarray,
        distance_pixels: float,
    ) -> np.ndarray:
        """Expand the usable track by moving an edge away from the asphalt.

        The same rule works for both contours: the outer line moves into the
        grass and the inner line moves into the infield. A local road-mask
        sample selects the correct normal direction even on tight bends.
        """
        points = contour.reshape(-1, 2).astype(np.float32)
        if len(points) < 7 or distance_pixels <= 0.0:
            return contour

        height, width = road_mask.shape
        probe_distance = max(8.0, distance_pixels + 4.0)

        def road_score(position: np.ndarray) -> float:
            x = int(np.clip(round(float(position[0])), 1, width - 2))
            y = int(np.clip(round(float(position[1])), 1, height - 2))
            return float(np.mean(road_mask[y - 1:y + 2, x - 1:x + 2] > 0))

        expanded = points.copy()
        for index, point in enumerate(points):
            tangent = points[(index + 3) % len(points)] - points[(index - 3) % len(points)]
            tangent_length = float(np.linalg.norm(tangent))
            if tangent_length <= 1e-6:
                continue
            normal = np.array([-tangent[1], tangent[0]], dtype=np.float32) / tangent_length
            plus_score = road_score(point + probe_distance * normal)
            minus_score = road_score(point - probe_distance * normal)
            non_road_direction = normal if plus_score < minus_score else -normal
            expanded[index] = point + distance_pixels * non_road_direction
        return expanded.reshape(-1, 1, 2)

    def _extract_boundaries(self) -> None:
        texture = cv2.imread(self.texture_path, cv2.IMREAD_COLOR)
        if texture is None:
            return

        # The top camera sees the complete 16 x 12 m texture rotated by 180°.
        texture = cv2.rotate(texture, cv2.ROTATE_180)
        channels = texture.astype(np.int16)
        gray = channels.mean(axis=2)
        channel_range = channels.max(axis=2) - channels.min(axis=2)

        # Asphalt is the large, dark, low-saturation ring. Selecting it from
        # the static source texture avoids cars and lighting in the live view.
        mask = (
            (gray >= 35.0)
            & (gray <= 108.0)
            & (channel_range <= 28)
        ).astype(np.uint8) * 255
        mask = cv2.morphologyEx(
            mask,
            cv2.MORPH_CLOSE,
            cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (15, 15)),
        )

        count, labels, stats, _ = cv2.connectedComponentsWithStats(mask, 8)
        if count <= 1:
            return
        largest_label = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
        road = np.where(labels == largest_label, 255, 0).astype(np.uint8)

        # Solid boundary paint in the texture. Other white road markings are
        # ignored later because every search is local and perpendicular to the
        # already identified road edge.
        white_mask = (
            (channels.min(axis=2) >= 165)
            & (channel_range <= 38)
        ).astype(np.uint8) * 255

        contours, hierarchy = cv2.findContours(
            road,
            cv2.RETR_CCOMP,
            cv2.CHAIN_APPROX_SIMPLE,
        )
        if not contours or hierarchy is None:
            return
        hierarchy = hierarchy[0]
        outer_index = max(
            range(len(contours)),
            key=lambda index: cv2.contourArea(contours[index])
            if hierarchy[index][3] == -1 else -1.0,
        )
        hole_indexes = [
            index
            for index in range(len(contours))
            if hierarchy[index][3] == outer_index
        ]
        if not hole_indexes:
            return
        inner_index = max(hole_indexes, key=lambda index: cv2.contourArea(contours[index]))

        height, width = road.shape
        scale = np.array([[[float(width), float(height)]]], dtype=np.float32)
        pixels_per_meter = 0.5 * (
            width / self.TRACK_WIDTH_METERS
            + height / self.TRACK_HEIGHT_METERS
        )
        selected = []
        for contour_number, index in enumerate((outer_index, inner_index)):
            contour = self._smooth_closed_contour(
                contours[index],
                sample_count=512 if contour_number == 0 else 448,
            )
            contour = self._snap_to_painted_edge(contour, white_mask)
            margin_meters = (
                self.OUTER_BOUNDARY_MARGIN_METERS
                if contour_number == 0
                else self.INNER_BOUNDARY_MARGIN_METERS
            )
            contour = self._offset_into_non_road(
                contour,
                road,
                margin_meters * pixels_per_meter,
            )
            selected.append(contour / scale)
        self.normalized_contours = selected

    def draw(self, frame: np.ndarray) -> np.ndarray:
        if not self.available or frame is None or frame.size == 0:
            return frame
        height, width = frame.shape[:2]
        scale = np.array([[[float(width), float(height)]]], dtype=np.float32)
        contours = [
            np.rint(contour * scale).astype(np.int32)
            for contour in self.normalized_contours
        ]
        cv2.polylines(
            frame,
            contours,
            isClosed=True,
            color=(0, 0, 255),
            thickness=max(2, int(round(width / 240.0))),
            lineType=cv2.LINE_AA,
        )
        return frame
