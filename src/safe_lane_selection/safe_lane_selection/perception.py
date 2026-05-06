from dataclasses import dataclass, field
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import cv2
import numpy as np


@dataclass
class LaneBoundary:
    x_bottom: float
    x_top: float
    angle_deg: float
    confidence: float
    points: List[Tuple[int, int, int, int]] = field(default_factory=list)


@dataclass
class LaneModel:
    detected: bool
    boundaries: List[LaneBoundary] = field(default_factory=list)
    lane_centers: List[float] = field(default_factory=list)
    # Stable/logical ego-lane id. This is allowed to come from localization,
    # because the per-frame visual lane-center index is not a physical lane id.
    current_lane: int = -1
    # Index into lane_centers used by the visual controller for this frame.
    target_lane_index: int = -1
    # Smoothed image x-coordinate that the controller should follow. This is
    # more stable than target_lane_index because the index changes whenever a
    # detected lane center appears/disappears in the current frame.
    target_center_x: float = 0.0
    lateral_error: float = 0.0
    orientation_error_deg: float = 0.0
    confidence: float = 0.0


@dataclass
class DetectedObject:
    kind: str
    object_id: str
    center_x: float
    center_y: float
    width: float
    height: float
    area: float
    confidence: float
    lane_index: int = -1

    @property
    def near_score(self) -> float:
        # Bottom-of-image and large objects should dominate safety selection.
        return float(np.clip(0.55 * self.center_y + 10.0 * self.area, 0.0, 1.8))


@dataclass
class PerceptionResult:
    lane: LaneModel
    objects: List[DetectedObject]


class MultiLaneDetector:
    """Detect lane boundaries and estimate lane centers/orientation from a camera frame."""

    def __init__(
        self,
        roi_top: float = 0.43,
        min_line_length: int = 35,
        max_line_gap: int = 18,
        cluster_px: int = 42,
        min_lane_width: float = 0.12,
        max_lane_width: float = 0.45,
        image_center_x: float = 0.5,
        ego_track_alpha: float = 0.30,
        ego_track_max_jump: float = 0.18,
    ) -> None:
        self.roi_top = float(np.clip(roi_top, 0.05, 0.9))
        self.min_line_length = int(min_line_length)
        self.max_line_gap = int(max_line_gap)
        self.cluster_px = int(cluster_px)
        self.min_lane_width = min_lane_width
        self.max_lane_width = max_lane_width
        # Camera-frame x-coordinate of the robot's longitudinal axis. Used to
        # decide which detected lane is the ego-lane regardless of camera yaw.
        self.image_center_x = float(image_center_x)
        # Temporal tracking: smooth the ego-lane image x across frames so that
        # the chosen lane index stays stable even if Hough finds slightly
        # different boundary sets each frame. Closer to 1 = trust new frame
        # more, closer to 0 = stickier history.
        self.ego_track_alpha = float(np.clip(ego_track_alpha, 0.05, 1.0))
        # Maximum jump in normalized image x that we accept frame-to-frame as
        # "still the same lane". Larger jumps are interpreted as a real lane
        # change candidate and require confirmation.
        self.ego_track_max_jump = float(ego_track_max_jump)
        self.tracked_ego_x: Optional[float] = None
        self.tracked_ego_lane: int = -1
        self.tracked_lane_count: int = 0
        self.last_mask: Optional[np.ndarray] = None

    def detect(self, frame_bgr: np.ndarray) -> LaneModel:
        height, width = frame_bgr.shape[:2]
        y0 = int(height * self.roi_top)
        roi = frame_bgr[y0:, :]
        mask = self._lane_mask(roi)
        self.last_mask = mask

        lines = cv2.HoughLinesP(
            mask,
            rho=1,
            theta=np.pi / 180,
            threshold=35,
            minLineLength=self.min_line_length,
            maxLineGap=self.max_line_gap,
        )
        if lines is None:
            return LaneModel(False)

        candidates: List[Tuple[float, float, float, float, Tuple[int, int, int, int]]] = []
        roi_h = roi.shape[0]
        for raw_line in lines[:, 0, :]:
            x1, y1, x2, y2 = [int(value) for value in raw_line]
            dx = x2 - x1
            dy = y2 - y1
            length = float(np.hypot(dx, dy))
            if length < self.min_line_length:
                continue
            # Allow more horizontal lines: lane edges close to the camera (e.g.
            # the yellow border immediately next to the robot) appear with a
            # large dx/dy ratio in the image plane.
            if abs(dy) < max(6, abs(dx) * 0.20):
                continue

            # Use the segment's actual bottom-most and top-most points instead
            # of linear extrapolation. Short segments near the vanishing point
            # have unstable extrapolations: a couple of pixels of dx amplify to
            # wild x_bot values when projected to the ROI bottom, which
            # collapses the clustering and produces bogus boundaries clustered
            # off-frame.
            if y1 >= y2:
                x_bottom = float(x1)
                x_top = float(x2)
            else:
                x_bottom = float(x2)
                x_top = float(x1)
            # Keep only segments whose visible bottom point lies inside the
            # frame (with a small margin for noise).
            if not (-0.05 * width <= x_bottom <= 1.05 * width):
                continue

            # Positive angle means the boundary leans right toward the bottom.
            angle_deg = float(np.degrees(np.arctan2(dx, abs(dy))))
            candidates.append((x_bottom, x_top, angle_deg, length, (x1, y1 + y0, x2, y2 + y0)))

        if not candidates:
            return LaneModel(False)

        boundaries = self._cluster_boundaries(candidates, width)
        lane_centers = self._lane_centers(boundaries)
        if not lane_centers:
            return LaneModel(False, boundaries=boundaries)

        # Pick the ego-lane using temporal continuity: prefer the lane closest
        # to the smoothed historical position. Without history (first frame),
        # fall back to the camera-bias-corrected image center.
        reference_x = (
            self.tracked_ego_x if self.tracked_ego_x is not None else self.image_center_x
        )
        distances = [abs(center - reference_x) for center in lane_centers]
        current_lane = int(np.argmin(distances))
        target_center = lane_centers[current_lane]
        # Reject huge frame-to-frame jumps unless we genuinely lost lock: if
        # the closest detected lane is far from the tracked position and we
        # already had a stable estimate, keep the old reference and let the
        # next frames either confirm or drift toward the new value.
        if (
            self.tracked_ego_x is not None
            and abs(target_center - self.tracked_ego_x) > self.ego_track_max_jump
        ):
            target_center = self.tracked_ego_x
            current_lane = int(
                np.argmin([abs(center - self.tracked_ego_x) for center in lane_centers])
            )
        # EMA smoothing on the ego-lane image x.
        if self.tracked_ego_x is None:
            self.tracked_ego_x = float(lane_centers[current_lane])
        else:
            self.tracked_ego_x = (
                self.ego_track_alpha * float(lane_centers[current_lane])
                + (1.0 - self.ego_track_alpha) * self.tracked_ego_x
            )
        # Replace the lane center we expose downstream with the smoothed value
        # so the controller sees a stable target and the debug overlay no
        # longer jitters across L0/L1/L2.
        smoothed_center = float(np.clip(self.tracked_ego_x, 0.0, 1.0))
        lane_centers[current_lane] = smoothed_center
        target_center = smoothed_center
        lateral_error = target_center - self.image_center_x
        orientation_error = self._lane_orientation(boundaries, target_center)
        confidence = float(np.clip(len(boundaries) / 5.0 + sum(b.confidence for b in boundaries) / 8.0, 0.0, 1.0))

        return LaneModel(
            detected=True,
            boundaries=boundaries,
            lane_centers=lane_centers,
            current_lane=current_lane,
            target_lane_index=current_lane,
            target_center_x=target_center,
            lateral_error=lateral_error,
            orientation_error_deg=orientation_error,
            confidence=confidence,
        )

    @staticmethod
    def _x_at_y(x1: int, y1: int, x2: int, y2: int, y: int) -> Optional[float]:
        if y2 == y1:
            return None
        ratio = (y - y1) / float(y2 - y1)
        return x1 + ratio * (x2 - x1)

    def _lane_mask(self, roi: np.ndarray) -> np.ndarray:
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        white = cv2.inRange(hsv, (0, 0, 145), (179, 95, 255))
        yellow = cv2.inRange(hsv, (15, 65, 90), (45, 255, 255))
        mask = cv2.bitwise_or(white, yellow)
        mask = cv2.GaussianBlur(mask, (5, 5), 0)
        _, mask = cv2.threshold(mask, 80, 255, cv2.THRESH_BINARY)
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        return mask

    def _cluster_boundaries(
        self,
        candidates: Sequence[Tuple[float, float, float, float, Tuple[int, int, int, int]]],
        image_width: int,
    ) -> List[LaneBoundary]:
        sorted_candidates = sorted(candidates, key=lambda item: item[0])
        clusters: List[List[Tuple[float, float, float, float, Tuple[int, int, int, int]]]] = []
        for candidate in sorted_candidates:
            if not clusters:
                clusters.append([candidate])
                continue
            cluster_mean = float(np.mean([item[0] for item in clusters[-1]]))
            if abs(candidate[0] - cluster_mean) <= self.cluster_px:
                clusters[-1].append(candidate)
            else:
                clusters.append([candidate])

        boundaries: List[LaneBoundary] = []
        for cluster in clusters:
            weights = np.array([item[3] for item in cluster], dtype=float)
            total = max(float(np.sum(weights)), 1.0)
            x_bottom = float(np.sum([item[0] * item[3] for item in cluster]) / total) / image_width
            x_top = float(np.sum([item[1] * item[3] for item in cluster]) / total) / image_width
            angle = float(np.sum([item[2] * item[3] for item in cluster]) / total)
            confidence = float(np.clip(total / 450.0, 0.0, 1.0))
            boundaries.append(
                LaneBoundary(
                    x_bottom=x_bottom,
                    x_top=x_top,
                    angle_deg=angle,
                    confidence=confidence,
                    points=[item[4] for item in cluster],
                )
            )

        return sorted(boundaries, key=lambda boundary: boundary.x_bottom)

    def _lane_centers(self, boundaries: Sequence[LaneBoundary]) -> List[float]:
        centers: List[float] = []
        for left, right in zip(boundaries[:-1], boundaries[1:]):
            width = right.x_bottom - left.x_bottom
            if self.min_lane_width <= width <= self.max_lane_width:
                centers.append(0.5 * (left.x_bottom + right.x_bottom))
        return centers

    @staticmethod
    def _lane_orientation(boundaries: Sequence[LaneBoundary], target_center: float) -> float:
        if not boundaries:
            return 0.0
        # Pick the boundary immediately to the left of the lane center and the
        # one immediately to the right. Their image-plane angles are opposite in
        # sign when the camera is aligned with the road, so the average gives a
        # clean orientation error. Averaging the two nearest boundaries ignored
        # the side and could pick two boundaries on the same side of the lane,
        # leading to absurd readings (e.g. 48 deg on a straight road).
        left = [b for b in boundaries if b.x_bottom < target_center]
        right = [b for b in boundaries if b.x_bottom >= target_center]
        if not left or not right:
            nearest = sorted(boundaries, key=lambda b: abs(b.x_bottom - target_center))[:2]
            return float(np.mean([b.angle_deg for b in nearest])) if nearest else 0.0
        left_b = max(left, key=lambda b: b.x_bottom)
        right_b = min(right, key=lambda b: b.x_bottom)
        return 0.5 * (left_b.angle_deg + right_b.angle_deg)


class CameraObjectDetector:
    """Detect other cars/obstacles through color segmentation and optional ArUco markers."""

    def __init__(
        self,
        min_area_ratio: float = 0.002,
        enable_aruco: bool = True,
        aruco_dictionary: str = "DICT_4X4_50",
        car_marker_ids: Iterable[int] = (21, 22, 23),
        obstacle_marker_ids: Iterable[int] = (31, 32, 33),
    ) -> None:
        self.min_area_ratio = min_area_ratio
        self.car_marker_ids = {int(marker_id) for marker_id in car_marker_ids}
        self.obstacle_marker_ids = {int(marker_id) for marker_id in obstacle_marker_ids}
        self.aruco = None
        self.aruco_dictionary = None
        self.aruco_parameters = None
        self.aruco_detector = None
        if enable_aruco and hasattr(cv2, "aruco"):
            self.aruco = cv2.aruco
            dictionary_id = getattr(self.aruco, aruco_dictionary)
            if hasattr(self.aruco, "getPredefinedDictionary"):
                self.aruco_dictionary = self.aruco.getPredefinedDictionary(dictionary_id)
            else:
                self.aruco_dictionary = self.aruco.Dictionary_get(dictionary_id)
            if hasattr(self.aruco, "DetectorParameters"):
                self.aruco_parameters = self.aruco.DetectorParameters()
            else:
                self.aruco_parameters = self.aruco.DetectorParameters_create()
            if hasattr(self.aruco, "ArucoDetector"):
                self.aruco_detector = self.aruco.ArucoDetector(self.aruco_dictionary, self.aruco_parameters)

    def detect(self, frame_bgr: np.ndarray, lane_centers: Sequence[float]) -> List[DetectedObject]:
        objects = self._detect_colored_objects(frame_bgr)
        objects.extend(self._detect_aruco_objects(frame_bgr))
        objects = self._merge_overlaps(objects)
        for detected in objects:
            detected.lane_index = self.assign_lane(detected.center_x, lane_centers)
        objects.sort(key=lambda obj: obj.near_score, reverse=True)
        return objects

    def _detect_colored_objects(self, frame_bgr: np.ndarray) -> List[DetectedObject]:
        height, width = frame_bgr.shape[:2]
        hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
        b, g, r = cv2.split(frame_bgr)
        masks: Dict[str, np.ndarray] = {}

        white_lane = cv2.inRange(hsv, (0, 0, 150), (179, 105, 255))
        yellow_lane = cv2.inRange(hsv, (14, 45, 70), (48, 255, 255))
        not_lane_paint = cv2.bitwise_not(cv2.bitwise_or(white_lane, yellow_lane))

        red_hsv_1 = cv2.inRange(hsv, (0, 45, 45), (14, 255, 255))
        red_hsv_2 = cv2.inRange(hsv, (165, 45, 45), (179, 255, 255))
        red_bgr = (
            (r.astype(np.int16) > 75)
            & (g.astype(np.int16) < r.astype(np.int16) * 0.55)
            & (b.astype(np.int16) < r.astype(np.int16) * 0.55)
        ).astype(np.uint8) * 255

        green_hsv = cv2.inRange(hsv, (40, 45, 35), (88, 255, 255))
        green_bgr = (
            (g.astype(np.int16) > 55)
            & (g.astype(np.int16) > r.astype(np.int16) + 16)
            & (g.astype(np.int16) > b.astype(np.int16) + 16)
        ).astype(np.uint8) * 255

        masks["car_red"] = cv2.bitwise_and(
            cv2.bitwise_or(cv2.bitwise_or(red_hsv_1, red_hsv_2), red_bgr),
            not_lane_paint,
        )
        masks["car_green"] = cv2.bitwise_and(cv2.bitwise_or(green_hsv, green_bgr), not_lane_paint)

        objects: List[DetectedObject] = []
        frame_area = float(height * width)
        for kind, mask in masks.items():
            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for index, contour in enumerate(contours):
                area = float(cv2.contourArea(contour))
                area_ratio = area / frame_area
                # Ignore tiny specks and very large regions such as the road
                # surface under lighting changes.
                min_area = self.min_area_ratio
                if area_ratio < min_area or area_ratio > 0.20:
                    continue
                x, y, w, h = cv2.boundingRect(contour)
                if w < 10 or h < 10:
                    continue
                touches_side = x <= 2 or (x + w) >= (width - 2)
                if touches_side and area_ratio > 0.004:
                    continue
                center_y = (y + 0.5 * h) / height
                if center_y < 0.18:
                    continue
                aspect = w / float(max(h, 1))
                if aspect > 5.0 or aspect < 0.20:
                    continue
                objects.append(
                    DetectedObject(
                        kind="car",
                        object_id=f"{kind}_{index}",
                        center_x=(x + 0.5 * w) / width,
                        center_y=center_y,
                        width=w / width,
                        height=h / height,
                        area=area_ratio,
                        confidence=float(np.clip(area_ratio / 0.03, 0.25, 1.0)),
                    )
                )
        return objects

    def _detect_aruco_objects(self, frame_bgr: np.ndarray) -> List[DetectedObject]:
        if self.aruco is None or self.aruco_dictionary is None:
            return []
        gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
        if self.aruco_detector is not None:
            corners, ids, _ = self.aruco_detector.detectMarkers(gray)
        else:
            corners, ids, _ = self.aruco.detectMarkers(
                gray, self.aruco_dictionary, parameters=self.aruco_parameters
            )
        if ids is None:
            return []

        height, width = frame_bgr.shape[:2]
        frame_area = float(height * width)
        objects: List[DetectedObject] = []
        for marker_id, marker_corners in zip(ids.flatten(), corners):
            points = marker_corners.reshape(4, 2)
            x_min, y_min = np.min(points, axis=0)
            x_max, y_max = np.max(points, axis=0)
            marker_area = max(float(cv2.contourArea(points.astype(np.float32))), 1.0) / frame_area
            kind = "marker"
            if int(marker_id) in self.car_marker_ids:
                kind = "car"
            elif int(marker_id) in self.obstacle_marker_ids:
                kind = "obstacle"
            objects.append(
                DetectedObject(
                    kind=kind,
                    object_id=f"aruco_{int(marker_id)}",
                    center_x=float(np.mean(points[:, 0]) / width),
                    center_y=float(np.mean(points[:, 1]) / height),
                    width=float((x_max - x_min) / width),
                    height=float((y_max - y_min) / height),
                    area=marker_area,
                    confidence=1.0,
                )
            )
        return objects

    @staticmethod
    def assign_lane(center_x: float, lane_centers: Sequence[float]) -> int:
        if not lane_centers:
            return -1
        distances = [abs(center_x - center) for center in lane_centers]
        lane_index = int(np.argmin(distances))
        if distances[lane_index] > 0.28:
            return -1
        return lane_index

    @staticmethod
    def _merge_overlaps(objects: Sequence[DetectedObject]) -> List[DetectedObject]:
        merged: List[DetectedObject] = []
        for obj in objects:
            duplicate = False
            for kept in merged:
                if abs(obj.center_x - kept.center_x) < 0.05 and abs(obj.center_y - kept.center_y) < 0.06:
                    if obj.confidence > kept.confidence:
                        kept.kind = obj.kind
                        kept.object_id = obj.object_id
                        kept.area = max(kept.area, obj.area)
                        kept.confidence = obj.confidence
                    duplicate = True
                    break
            if not duplicate:
                merged.append(obj)
        return merged


class SafeLaneSelector:
    """Choose the lane with lowest risk and small switching cost."""

    def __init__(
        self,
        switch_penalty: float = 0.35,
        object_lane_penalty: float = 2.5,
        adjacent_lane_penalty: float = 0.35,
        prefer_center_weight: float = 0.25,
    ) -> None:
        self.switch_penalty = switch_penalty
        self.object_lane_penalty = object_lane_penalty
        self.adjacent_lane_penalty = adjacent_lane_penalty
        self.prefer_center_weight = prefer_center_weight

    def select(
        self,
        lane: LaneModel,
        objects: Sequence[DetectedObject],
        previous_lane: int = -1,
    ) -> Tuple[int, List[float]]:
        if not lane.detected or not lane.lane_centers:
            return -1, []

        scores = [1.0 for _ in lane.lane_centers]
        for index, center in enumerate(lane.lane_centers):
            scores[index] -= self.prefer_center_weight * abs(center - 0.5)
            if previous_lane >= 0 and index != previous_lane:
                scores[index] -= self.switch_penalty

        for obj in objects:
            if obj.lane_index < 0:
                continue
            risk = obj.near_score
            scores[obj.lane_index] -= self.object_lane_penalty * risk
            for adjacent in (obj.lane_index - 1, obj.lane_index + 1):
                if 0 <= adjacent < len(scores):
                    scores[adjacent] -= self.adjacent_lane_penalty * risk

        selected = int(np.argmax(scores))
        return selected, scores


def draw_debug(
    frame_bgr: np.ndarray,
    perception: PerceptionResult,
    selected_lane: int,
    lane_scores: Sequence[float],
    extra_lines: Sequence[str] = (),
) -> np.ndarray:
    debug = frame_bgr.copy()
    height, width = debug.shape[:2]

    for boundary in perception.lane.boundaries:
        color = (255, 255, 255)
        for x1, y1, x2, y2 in boundary.points:
            cv2.line(debug, (x1, y1), (x2, y2), color, 2)
        xb = int(boundary.x_bottom * width)
        xt = int(boundary.x_top * width)
        cv2.circle(debug, (xb, height - 8), 5, (0, 255, 255), -1)
        cv2.circle(debug, (xt, int(height * 0.45)), 4, (255, 255, 0), -1)

    visual_target = perception.lane.target_lane_index
    for idx, center in enumerate(perception.lane.lane_centers):
        x = int(center * width)
        color = (0, 255, 0) if idx == visual_target else (0, 160, 255)
        cv2.line(debug, (x, int(height * 0.42)), (x, height - 1), color, 2)
        score = lane_scores[idx] if idx < len(lane_scores) else 0.0
        cv2.putText(debug, f"L{idx}:{score:.2f}", (x - 34, height - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1)

    for obj in perception.objects:
        x = int((obj.center_x - obj.width * 0.5) * width)
        y = int((obj.center_y - obj.height * 0.5) * height)
        w = int(obj.width * width)
        h = int(obj.height * height)
        if "green" in obj.object_id:
            color = (0, 255, 0)
        elif obj.kind == "car":
            color = (0, 0, 255)
        else:
            color = (180, 180, 180)
        cv2.rectangle(debug, (x, y), (x + w, y + h), color, 2)
        lane_text = f"L{obj.lane_index + 1}" if obj.lane_index >= 0 else "L?"
        cv2.putText(
            debug,
            f"{obj.kind} {lane_text} r={obj.near_score:.2f}",
            (x, max(18, y - 6)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            color,
            1,
        )

    n_bnd = len(perception.lane.boundaries)
    n_centers = len(perception.lane.lane_centers)
    bnd_xs = ",".join(f"{b.x_bottom:.2f}" for b in perception.lane.boundaries[:6])
    lane_number = perception.lane.current_lane + 1 if perception.lane.current_lane >= 0 else -1
    target_number = selected_lane + 1 if selected_lane >= 0 else -1
    status = (
        f"det={int(perception.lane.detected)} lane#={lane_number} "
        f"target#={target_number} orient={perception.lane.orientation_error_deg:.1f}deg"
    )
    cv2.putText(debug, status, (12, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (40, 255, 40), 2)
    center_xs = ",".join(f"{center:.2f}" for center in perception.lane.lane_centers[:6])
    target_x = perception.lane.target_center_x if perception.lane.detected else None
    target_text = f"{target_x:.2f}" if target_x is not None else "--"
    diag = (
        f"idx lane={perception.lane.current_lane} target={selected_lane} "
        f"bnd={n_bnd} centers={n_centers} obj={len(perception.objects)} "
        f"vis={perception.lane.target_lane_index} "
        f"target_x={target_text} x_bot=[{bnd_xs}]"
    )
    cv2.putText(debug, diag, (12, 52), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (40, 255, 40), 1)
    centers_diag = f"centers=[{center_xs}]"
    cv2.putText(debug, centers_diag, (12, 76), cv2.FONT_HERSHEY_SIMPLEX, 0.50, (40, 255, 40), 1)
    for index, line in enumerate(extra_lines[:2]):
        cv2.putText(
            debug,
            line,
            (12, 100 + index * 22),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.50,
            (40, 255, 40),
            1,
        )
    return debug
