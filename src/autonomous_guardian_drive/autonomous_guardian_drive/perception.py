from dataclasses import dataclass
from typing import Iterable, List, Optional

import cv2
import numpy as np


@dataclass
class LaneObservation:
    detected: bool
    center_x: float = 0.5
    error: float = 0.0
    confidence: float = 0.0
    pixel_count: int = 0


@dataclass
class MarkerDetection:
    marker_id: int
    center_x: float
    center_y: float
    area_ratio: float
    width_ratio: float
    height_ratio: float
    corners: np.ndarray


class LaneDetector:
    """Detect a painted lane/road guide in the lower part of a camera frame."""

    def __init__(
        self,
        mode: str = "white_or_yellow",
        roi_top: float = 0.55,
        min_pixels: int = 120,
        debug: bool = False,
    ) -> None:
        self.mode = mode
        self.roi_top = float(np.clip(roi_top, 0.05, 0.95))
        self.min_pixels = min_pixels
        self.debug = debug
        self.last_mask: Optional[np.ndarray] = None

    def detect(self, frame_bgr: np.ndarray) -> LaneObservation:
        height, width = frame_bgr.shape[:2]
        top = int(height * self.roi_top)
        roi = frame_bgr[top:height, :]

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mask = self._make_mask(hsv)
        mask = cv2.medianBlur(mask, 5)
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        self.last_mask = mask

        ys, xs = np.where(mask > 0)
        pixel_count = int(xs.size)
        if pixel_count < self.min_pixels:
            return LaneObservation(False, pixel_count=pixel_count)

        # If both lane borders are visible, the useful target is the midpoint
        # between the left and right clusters. If only a center line is visible,
        # this naturally collapses to the line centroid.
        x_low = float(np.percentile(xs, 10))
        x_high = float(np.percentile(xs, 90))
        if (x_high - x_low) > width * 0.22:
            center_px = 0.5 * (x_low + x_high)
        else:
            center_px = float(np.mean(xs))

        center_x = center_px / float(width)
        error = center_x - 0.5
        confidence = float(np.clip(pixel_count / (mask.size * 0.08), 0.0, 1.0))
        return LaneObservation(True, center_x, error, confidence, pixel_count)

    def _make_mask(self, hsv_roi: np.ndarray) -> np.ndarray:
        white = cv2.inRange(hsv_roi, (0, 0, 145), (179, 85, 255))
        yellow = cv2.inRange(hsv_roi, (15, 70, 90), (45, 255, 255))
        black = cv2.inRange(hsv_roi, (0, 0, 0), (179, 255, 80))

        if self.mode == "white":
            return white
        if self.mode == "yellow":
            return yellow
        if self.mode == "black":
            return black
        if self.mode == "white_or_yellow":
            return cv2.bitwise_or(white, yellow)
        if self.mode == "bright_or_dark":
            return cv2.bitwise_or(cv2.bitwise_or(white, yellow), black)
        return cv2.bitwise_or(white, yellow)


class ArucoDetector:
    """Small OpenCV ArUco wrapper that works across OpenCV minor versions."""

    def __init__(self, dictionary_name: str = "DICT_4X4_50") -> None:
        if not hasattr(cv2, "aruco"):
            raise RuntimeError("This OpenCV build does not include cv2.aruco")

        self.aruco = cv2.aruco
        dictionary_id = getattr(self.aruco, dictionary_name)
        if hasattr(self.aruco, "getPredefinedDictionary"):
            self.dictionary = self.aruco.getPredefinedDictionary(dictionary_id)
        else:
            self.dictionary = self.aruco.Dictionary_get(dictionary_id)

        if hasattr(self.aruco, "DetectorParameters"):
            self.parameters = self.aruco.DetectorParameters()
        else:
            self.parameters = self.aruco.DetectorParameters_create()

        self.detector = None
        if hasattr(self.aruco, "ArucoDetector"):
            self.detector = self.aruco.ArucoDetector(self.dictionary, self.parameters)

    def detect(self, frame_bgr: np.ndarray) -> List[MarkerDetection]:
        gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
        if self.detector is not None:
            corners, ids, _ = self.detector.detectMarkers(gray)
        else:
            corners, ids, _ = self.aruco.detectMarkers(
                gray, self.dictionary, parameters=self.parameters
            )

        if ids is None:
            return []

        height, width = frame_bgr.shape[:2]
        frame_area = float(height * width)
        detections: List[MarkerDetection] = []
        for marker_id, marker_corners in zip(ids.flatten(), corners):
            points = marker_corners.reshape(4, 2)
            x_min, y_min = np.min(points, axis=0)
            x_max, y_max = np.max(points, axis=0)
            marker_width = max(float(x_max - x_min), 1.0)
            marker_height = max(float(y_max - y_min), 1.0)
            area = float(cv2.contourArea(points.astype(np.float32)))
            detections.append(
                MarkerDetection(
                    marker_id=int(marker_id),
                    center_x=float(np.mean(points[:, 0]) / width),
                    center_y=float(np.mean(points[:, 1]) / height),
                    area_ratio=float(area / frame_area),
                    width_ratio=float(marker_width / width),
                    height_ratio=float(marker_height / height),
                    corners=points,
                )
            )

        detections.sort(key=lambda detection: detection.area_ratio, reverse=True)
        return detections

    @staticmethod
    def filter_ids(
        detections: Iterable[MarkerDetection],
        accepted_ids: Iterable[int],
    ) -> List[MarkerDetection]:
        accepted = {int(marker_id) for marker_id in accepted_ids}
        return [detection for detection in detections if detection.marker_id in accepted]

