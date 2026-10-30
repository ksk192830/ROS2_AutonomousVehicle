"""Lane centers supported by current mask edges, shared by display and control.

Image/warp borders are not physical lane boundaries. A clipped boundary may
only be reconstructed with a measured width; it must never become a midpoint
with the image border or an unconstrained polynomial beyond the observations.
"""
from dataclasses import dataclass
from typing import Optional

import cv2
import numpy as np


@dataclass
class LaneGeometry:
    segments: list
    left: np.ndarray
    right: np.ndarray
    measured_width: Optional[float]

    @property
    def observed_rows(self):
        return sum(int(np.count_nonzero(s[:, 2] == 0)) for s in self.segments)

    @property
    def inferred_rows(self):
        return sum(int(np.count_nonzero(s[:, 2] != 0)) for s in self.segments)


def smooth(values, window=9):
    window = min(window, len(values) if len(values) % 2 else len(values) - 1)
    if window < 3:
        return values.copy()
    rows = np.lib.stride_tricks.sliding_window_view(
        np.pad(values, (window // 2, window // 2), mode='edge'), window)
    return np.median(rows, axis=1)


def extract_lane(mask, valid, width_hint=None):
    """Return ordered [x, y, inferred] segments without extrapolating endpoints.

    Width hints are lane-specific and expire in the caller. Current paired
    edges take priority, including widths above the old erroneous 45% cap.
    Separate islands on a row never get merged into one wide lane.
    """
    h, w = mask.shape
    active = ((mask > 0) & valid).astype(np.uint8)
    count, labels, stats, _ = cv2.connectedComponentsWithStats(active, 8)
    if count > 1:
        areas = stats[1:, cv2.CC_STAT_AREA]
        retained = np.flatnonzero(areas >= max(12, areas.max() * .15)) + 1
        active = np.isin(labels, retained)
    safe = cv2.erode(valid.astype(np.uint8), np.ones((3, 3), np.uint8)).astype(bool)
    rows, left, right = [], [], []
    for y in np.flatnonzero(active.any(axis=1)):
        changes = np.diff(np.r_[False, active[y], False].astype(np.int8))
        starts = np.flatnonzero(changes == 1)
        ends = np.flatnonzero(changes == -1) - 1
        index = int(np.argmax(ends - starts))
        if ends[index] - starts[index] < max(3, w * .015):
            continue
        rows.append(y)
        left.append(starts[index])
        right.append(ends[index])
    ys = np.asarray(rows, dtype=int)
    left = np.asarray(left, dtype=int)
    right = np.asarray(right, dtype=int)
    lv = (left > 1) & safe[ys, left]
    rv = (right < w - 2) & safe[ys, right]
    widths = right - left
    paired = lv & rv & (widths >= max(8, .04 * w)) & (widths <= .85 * w)
    measured = None
    if np.count_nonzero(paired) >= 6:
        samples = widths[paired]
        median = np.median(samples)
        stable = np.abs(samples - median) <= max(6, 3 * np.median(np.abs(samples - median)))
        if np.count_nonzero(stable) >= 6:
            measured = float(np.median(samples[stable]))
    centers = np.full(len(ys), np.nan)
    inferred = np.zeros(len(ys), dtype=bool)
    centers[paired] = (left[paired] + right[paired]) * .5
    if measured is not None:
        # Follow the locally observed width where available; at a clipped end
        # hold the closest measured width, rather than inventing a long curve.
        profile = np.interp(ys, ys[paired], smooth(widths[paired].astype(float)))
    elif width_hint is not None and .04 * w <= width_hint <= .85 * w:
        profile = np.full(len(ys), width_hint)
    else:
        profile = None
    if profile is not None:
        left_only, right_only = lv & ~rv, rv & ~lv
        centers[left_only] = left[left_only] + profile[left_only] * .5
        centers[right_only] = right[right_only] - profile[right_only] * .5
        inferred = left_only | right_only
    keep = np.isfinite(centers) & (centers >= 0) & (centers < w)
    keep_indices = np.flatnonzero(keep)
    if len(keep_indices):
        keep[keep_indices] &= valid[ys[keep_indices], centers[keep_indices].astype(int)]
    points = np.column_stack((centers[keep], ys[keep], inferred[keep])).astype(float)
    segments = []
    if len(points):
        # Interpolate only small internal gaps. A long occlusion or a spatial
        # jump ends the line; drawing through it would imply false evidence.
        delta = np.diff(points[:, :2], axis=0)
        breaks = np.flatnonzero((delta[:, 1] > max(4, h * .04)) |
                               (np.abs(delta[:, 0]) > np.maximum(w * .06, delta[:, 1] * 2))) + 1
        for part in np.split(points, breaks):
            if len(part) < 5 or np.ptp(part[:, 1]) < h * .05:
                continue
            y = np.arange(part[0, 1], part[-1, 1] + 1)
            x = np.interp(y, part[:, 1], smooth(part[:, 0]))
            guess = np.interp(y, part[:, 1], part[:, 2]) > 0
            guess |= ~np.isin(y, part[:, 1])
            segments.append(np.column_stack((x, y, guess)))
    return LaneGeometry(segments, np.column_stack((left[lv], ys[lv])),
                        np.column_stack((right[rv], ys[rv])), measured)


def control_from_geometry(geometry, width, height, lane):
    """Use the same center as the diagram, requiring coverage near the car.

    Keep the existing lane-specific lookahead region for steering tuning.
    Missing observations at the bottom are not extended into a valid command.
    """
    for segment in reversed(geometry.segments):
        if segment[-1, 1] < height - 1 - max(3, height * .025):
            continue
        upper = .995 if lane == 1 else .9
        region = segment[(segment[:, 1] >= height * .8) & (segment[:, 1] <= height * upper)]
        if len(region) < 5 or np.ptp(region[:, 1]) < height * .06:
            continue
        slope = np.polyfit(region[:, 1], region[:, 0], 1)[0]
        # Normalize both dimensions so steering stays invariant at other sizes.
        slope *= (480.0 / width) / (270.0 / height)
        return int(np.degrees(np.arctan2(-slope, 1))), width * .5 - segment[-1, 0]
    return None
