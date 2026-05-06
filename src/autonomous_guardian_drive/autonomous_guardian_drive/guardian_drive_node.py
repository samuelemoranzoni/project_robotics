import csv
import json
import math
import os
from enum import Enum
from typing import Dict, Iterable, List, Optional, Tuple

import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Image, Range
from std_msgs.msg import ColorRGBA, String

from robomaster_msgs.msg import LEDEffect

from .perception import ArucoDetector, LaneDetector, LaneObservation, MarkerDetection


class DriveState(str, Enum):
    WAITING_FOR_CAMERA = "WAITING_FOR_CAMERA"
    SEARCH_LINE = "SEARCH_LINE"
    LANE_FOLLOW = "LANE_FOLLOW"
    CAUTION = "CAUTION"
    EMERGENCY_STOP = "EMERGENCY_STOP"
    PARKED = "PARKED"


class GuardianDriveNode(Node):
    """Autonomous driving demo with lane following and intruder detection."""

    def __init__(self) -> None:
        super().__init__("guardian_drive")

        self._declare_parameters()
        self.bridge = CvBridge()

        self.cmd_pub = self.create_publisher(
            Twist, self.get_parameter("cmd_vel_topic").value, 10
        )
        self.led_color_pub = self.create_publisher(
            ColorRGBA, self.get_parameter("led_color_topic").value, 10
        )
        self.led_effect_pub = self.create_publisher(
            LEDEffect, self.get_parameter("led_effect_topic").value, 10
        )
        self.events_pub = self.create_publisher(
            String, self.get_parameter("events_topic").value, 10
        )

        self.debug_enabled = bool(self.get_parameter("debug_image").value)
        self.debug_pub = None
        if self.debug_enabled:
            self.debug_pub = self.create_publisher(
                Image, self.get_parameter("debug_image_topic").value, 3
            )

        self.lane_detector = LaneDetector(
            mode=str(self.get_parameter("lane_color_mode").value),
            roi_top=float(self.get_parameter("lane_roi_top").value),
            min_pixels=int(self.get_parameter("lane_min_pixels").value),
        )
        self.aruco_detector: Optional[ArucoDetector] = None
        if bool(self.get_parameter("enable_aruco").value):
            try:
                self.aruco_detector = ArucoDetector(
                    str(self.get_parameter("aruco_dictionary").value)
                )
            except Exception as exc:
                self.get_logger().warning(f"Aruco detector disabled: {exc}")

        self.intruder_ids = self._int_list("intruder_ids")
        self.stop_sign_ids = self._int_list("stop_sign_ids")
        self.goal_ids = self._int_list("goal_ids")

        self.ranges: Dict[str, float] = {}
        self.range_topics = self._str_list("range_topics")
        self.left_range_topics = self._str_list("left_range_topics")
        self.right_range_topics = self._str_list("right_range_topics")
        for topic in self.range_topics:
            self.create_subscription(
                Range,
                topic,
                lambda msg, topic=topic: self._range_callback(topic, msg),
                10,
            )

        self.create_subscription(
            Image,
            self.get_parameter("image_topic").value,
            self._image_callback,
            3,
        )

        self.state = DriveState.WAITING_FOR_CAMERA
        self.latest_lane = LaneObservation(False)
        self.latest_markers: List[MarkerDetection] = []
        self.latest_image_time: Optional[float] = None
        self.latest_intruder_time: Optional[float] = None
        self.first_intruder_seen_time: Optional[float] = None
        self.emergency_started_time: Optional[float] = None
        self.stop_sign_until: float = 0.0
        self.last_led_state: Optional[DriveState] = None
        self.last_status_log_time = 0.0

        self.scenario_level = str(self.get_parameter("scenario_level").value)
        self.csv_writer, self.csv_file = self._open_log()

        self.control_period = float(self.get_parameter("control_period").value)
        self.timer = self.create_timer(self.control_period, self._control_loop)
        self._enter_state(DriveState.WAITING_FOR_CAMERA, "node_started")

    def _declare_parameters(self) -> None:
        self.declare_parameter("image_topic", "camera/image_color")
        self.declare_parameter("cmd_vel_topic", "cmd_vel")
        self.declare_parameter("led_color_topic", "leds/color")
        self.declare_parameter("led_effect_topic", "leds/effect")
        self.declare_parameter("events_topic", "guardian/events")
        self.declare_parameter("debug_image_topic", "guardian/debug_image")
        self.declare_parameter("debug_image", True)

        self.declare_parameter("range_topics", ["range_0", "range_1", "range_2", "range_3"])
        self.declare_parameter("left_range_topics", ["range_2", "range_3"])
        self.declare_parameter("right_range_topics", ["range_0", "range_1"])

        self.declare_parameter("lane_color_mode", "white_or_yellow")
        self.declare_parameter("lane_roi_top", 0.56)
        self.declare_parameter("lane_min_pixels", 120)
        self.declare_parameter("lane_kp", 1.55)
        self.declare_parameter("lane_confidence_slow_threshold", 0.22)

        self.declare_parameter("enable_aruco", True)
        self.declare_parameter("aruco_dictionary", "DICT_4X4_50")
        self.declare_parameter("intruder_ids", [11, 12, 13])
        self.declare_parameter("stop_sign_ids", [30])
        self.declare_parameter("goal_ids", [99])

        self.declare_parameter("base_speed", 0.22)
        self.declare_parameter("caution_speed", 0.10)
        self.declare_parameter("search_turn_speed", 0.35)
        self.declare_parameter("max_turn_speed", 1.15)
        self.declare_parameter("avoidance_gain", 0.85)

        self.declare_parameter("caution_distance", 1.00)
        self.declare_parameter("stop_distance", 0.46)
        self.declare_parameter("emergency_distance", 0.28)
        self.declare_parameter("intruder_caution_area", 0.006)
        self.declare_parameter("intruder_stop_area", 0.030)
        self.declare_parameter("goal_stop_area", 0.020)
        self.declare_parameter("stop_sign_area", 0.014)
        self.declare_parameter("stop_sign_hold_seconds", 2.0)
        self.declare_parameter("emergency_hold_seconds", 2.5)
        self.declare_parameter("image_timeout_seconds", 1.0)
        self.declare_parameter("intruder_memory_seconds", 1.2)

        self.declare_parameter("scenario_level", "manual")
        self.declare_parameter("metrics_path", "logs/guardian_drive_metrics.csv")
        self.declare_parameter("control_period", 0.10)

    def _image_callback(self, msg: Image) -> None:
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().warning(f"Could not convert camera frame: {exc}")
            return

        self.latest_image_time = self._now_seconds()
        self.latest_lane = self.lane_detector.detect(frame)
        self.latest_markers = (
            self.aruco_detector.detect(frame) if self.aruco_detector is not None else []
        )
        intruder = self._largest_marker(self.latest_markers, self.intruder_ids)
        if intruder is not None:
            self.latest_intruder_time = self.latest_image_time
            if self.first_intruder_seen_time is None:
                self.first_intruder_seen_time = self.latest_image_time
                self._publish_event(
                    "intruder_detected",
                    marker_id=intruder.marker_id,
                    marker_area=intruder.area_ratio,
                )

        if self.debug_enabled and self.debug_pub is not None:
            self._publish_debug_frame(frame)

    def _range_callback(self, topic: str, msg: Range) -> None:
        value = float(msg.range)
        if math.isnan(value) or math.isinf(value):
            value = float(msg.max_range)
        value = max(float(msg.min_range), min(float(msg.max_range), value))
        self.ranges[topic] = value

    def _control_loop(self) -> None:
        now = self._now_seconds()
        if not self._camera_ready(now):
            self._enter_state(DriveState.WAITING_FOR_CAMERA, "camera_timeout")
            self._publish_stop()
            self._write_metrics("waiting_for_camera")
            return

        min_range = self._minimum_range()
        intruder = self._largest_marker(self.latest_markers, self.intruder_ids)
        stop_sign = self._largest_marker(self.latest_markers, self.stop_sign_ids)
        goal = self._largest_marker(self.latest_markers, self.goal_ids)

        if self.state == DriveState.PARKED:
            self._publish_stop()
            self._write_metrics("parked")
            return

        if goal is not None and goal.area_ratio >= float(self.get_parameter("goal_stop_area").value):
            self._enter_state(DriveState.PARKED, "goal_marker_reached")
            self._publish_stop()
            self._write_metrics("goal_reached")
            return

        if stop_sign is not None and stop_sign.area_ratio >= float(self.get_parameter("stop_sign_area").value):
            if now >= self.stop_sign_until:
                self.stop_sign_until = now + float(self.get_parameter("stop_sign_hold_seconds").value)
                self._publish_event("stop_sign_detected", marker_id=stop_sign.marker_id)

        if now < self.stop_sign_until:
            self._enter_state(DriveState.CAUTION, "stop_sign_hold")
            self._publish_stop()
            self._write_metrics("stop_sign_hold")
            return

        emergency = self._is_emergency(min_range, intruder)
        if emergency:
            if self.state != DriveState.EMERGENCY_STOP:
                self.emergency_started_time = now
                reaction_time = None
                if self.first_intruder_seen_time is not None:
                    reaction_time = now - self.first_intruder_seen_time
                self._enter_state(
                    DriveState.EMERGENCY_STOP,
                    "emergency_stop",
                    reaction_time=reaction_time,
                )
            self._publish_stop()
            self._write_metrics("emergency_stop")
            return

        if self.state == DriveState.EMERGENCY_STOP:
            hold = float(self.get_parameter("emergency_hold_seconds").value)
            if self.emergency_started_time is not None and now - self.emergency_started_time < hold:
                self._publish_stop()
                self._write_metrics("emergency_hold")
                return
            self.first_intruder_seen_time = None
            self.emergency_started_time = None

        stop_distance = float(self.get_parameter("stop_distance").value)
        if min_range is not None and min_range < stop_distance:
            self._enter_state(DriveState.CAUTION, "obstacle_stop_distance")
            self._publish_stop()
            self._write_metrics("obstacle_stop")
            return

        if not self.latest_lane.detected:
            self._enter_state(DriveState.SEARCH_LINE, "lane_lost")
            self._publish_search_twist()
            self._write_metrics("search_line")
            return

        caution = self._is_caution(min_range, intruder, now)
        self._enter_state(
            DriveState.CAUTION if caution else DriveState.LANE_FOLLOW,
            "intruder_or_obstacle_near" if caution else "lane_visible",
        )
        self._publish_lane_twist(caution=caution)
        self._write_metrics("drive")

        if self.first_intruder_seen_time is not None and not caution:
            memory = float(self.get_parameter("intruder_memory_seconds").value)
            if self.latest_intruder_time is None or now - self.latest_intruder_time > memory:
                self.first_intruder_seen_time = None

    def _publish_lane_twist(self, caution: bool) -> None:
        speed = (
            float(self.get_parameter("caution_speed").value)
            if caution
            else float(self.get_parameter("base_speed").value)
        )
        if self.latest_lane.confidence < float(self.get_parameter("lane_confidence_slow_threshold").value):
            speed *= 0.65

        lane_turn = -float(self.get_parameter("lane_kp").value) * self.latest_lane.error
        avoidance_turn = self._avoidance_turn()
        angular = lane_turn + avoidance_turn
        max_turn = float(self.get_parameter("max_turn_speed").value)
        angular = max(-max_turn, min(max_turn, angular))

        msg = Twist()
        msg.linear.x = speed
        msg.angular.z = angular
        self.cmd_pub.publish(msg)

    def _publish_search_twist(self) -> None:
        msg = Twist()
        msg.angular.z = float(self.get_parameter("search_turn_speed").value)
        self.cmd_pub.publish(msg)

    def _publish_stop(self) -> None:
        self.cmd_pub.publish(Twist())

    def _avoidance_turn(self) -> float:
        left = self._minimum_for_topics(self.left_range_topics)
        right = self._minimum_for_topics(self.right_range_topics)
        if left is None or right is None:
            return 0.0

        caution_distance = float(self.get_parameter("caution_distance").value)
        if min(left, right) > caution_distance:
            return 0.0

        gain = float(self.get_parameter("avoidance_gain").value)
        return -gain * (right - left)

    def _is_emergency(
        self,
        min_range: Optional[float],
        intruder: Optional[MarkerDetection],
    ) -> bool:
        emergency_distance = float(self.get_parameter("emergency_distance").value)
        if min_range is not None and min_range < emergency_distance:
            return True
        if intruder is None:
            return False
        return intruder.area_ratio >= float(self.get_parameter("intruder_stop_area").value)

    def _is_caution(
        self,
        min_range: Optional[float],
        intruder: Optional[MarkerDetection],
        now: float,
    ) -> bool:
        caution_distance = float(self.get_parameter("caution_distance").value)
        stop_distance = float(self.get_parameter("stop_distance").value)
        if min_range is not None and min_range < stop_distance:
            return True
        if min_range is not None and min_range < caution_distance:
            return True
        if intruder is not None:
            return intruder.area_ratio >= float(self.get_parameter("intruder_caution_area").value)
        if self.latest_intruder_time is None:
            return False
        memory = float(self.get_parameter("intruder_memory_seconds").value)
        return now - self.latest_intruder_time <= memory

    def _camera_ready(self, now: float) -> bool:
        if self.latest_image_time is None:
            return False
        timeout = float(self.get_parameter("image_timeout_seconds").value)
        return now - self.latest_image_time <= timeout

    def _minimum_range(self) -> Optional[float]:
        values = [self.ranges[topic] for topic in self.range_topics if topic in self.ranges]
        return min(values) if values else None

    def _minimum_for_topics(self, topics: Iterable[str]) -> Optional[float]:
        values = [self.ranges[topic] for topic in topics if topic in self.ranges]
        return min(values) if values else None

    @staticmethod
    def _largest_marker(
        markers: Iterable[MarkerDetection],
        accepted_ids: Iterable[int],
    ) -> Optional[MarkerDetection]:
        accepted = {int(marker_id) for marker_id in accepted_ids}
        candidates = [marker for marker in markers if marker.marker_id in accepted]
        return max(candidates, key=lambda marker: marker.area_ratio) if candidates else None

    def _enter_state(self, state: DriveState, reason: str, **details: object) -> None:
        if self.state == state and self.last_led_state == state:
            return
        previous = self.state
        self.state = state
        self._publish_led_for_state(state)
        self.last_led_state = state
        self._publish_event(
            "state_change",
            previous=previous.value if isinstance(previous, DriveState) else str(previous),
            state=state.value,
            reason=reason,
            **details,
        )

    def _publish_led_for_state(self, state: DriveState) -> None:
        if state == DriveState.EMERGENCY_STOP:
            effect = LEDEffect()
            effect.mask = LEDEffect.ALL
            effect.submask = 255
            effect.effect = LEDEffect.FLASH
            effect.color = self._color(1.0, 0.0, 0.0)
            effect.t1 = 0.12
            effect.t2 = 0.12
            self.led_effect_pub.publish(effect)
            return

        colors: Dict[DriveState, Tuple[float, float, float]] = {
            DriveState.WAITING_FOR_CAMERA: (0.0, 0.0, 0.45),
            DriveState.SEARCH_LINE: (0.0, 0.15, 1.0),
            DriveState.LANE_FOLLOW: (0.0, 0.45, 1.0),
            DriveState.CAUTION: (1.0, 0.72, 0.0),
            DriveState.PARKED: (0.0, 1.0, 0.1),
        }
        color = colors.get(state, (0.0, 0.0, 0.0))
        self.led_color_pub.publish(self._color(*color))

    @staticmethod
    def _color(r: float, g: float, b: float) -> ColorRGBA:
        msg = ColorRGBA()
        msg.r = float(r)
        msg.g = float(g)
        msg.b = float(b)
        msg.a = 1.0
        return msg

    def _publish_debug_frame(self, frame: "np.ndarray") -> None:
        try:
            import cv2
            import numpy as np
        except Exception:
            return

        debug = frame.copy()
        height, width = debug.shape[:2]
        roi_y = int(height * float(self.get_parameter("lane_roi_top").value))
        cv2.rectangle(debug, (0, roi_y), (width - 1, height - 1), (255, 255, 0), 1)
        cv2.line(debug, (width // 2, roi_y), (width // 2, height - 1), (80, 80, 80), 1)
        if self.latest_lane.detected:
            cx = int(self.latest_lane.center_x * width)
            cv2.circle(debug, (cx, int(height * 0.82)), 8, (0, 255, 255), -1)

        for marker in self.latest_markers:
            pts = marker.corners.astype(np.int32).reshape((-1, 1, 2))
            cv2.polylines(debug, [pts], True, (0, 0, 255), 2)
            x = int(marker.center_x * width)
            y = int(marker.center_y * height)
            cv2.putText(
                debug,
                str(marker.marker_id),
                (x - 12, y - 8),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 0, 255),
                2,
            )

        state_text = self.state.value
        cv2.putText(debug, state_text, (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (40, 255, 40), 2)
        msg = self.bridge.cv2_to_imgmsg(debug, encoding="bgr8")
        self.debug_pub.publish(msg)

    def _publish_event(self, event: str, **details: object) -> None:
        payload = {
            "time": round(self._now_seconds(), 3),
            "event": event,
            "state": self.state.value,
            "scenario_level": self.scenario_level,
        }
        payload.update(details)
        msg = String()
        msg.data = json.dumps(payload, sort_keys=True)
        self.events_pub.publish(msg)
        self.get_logger().info(msg.data)

    def _write_metrics(self, event: str) -> None:
        now = self._now_seconds()
        if now - self.last_status_log_time < 0.25 and event == "drive":
            return
        self.last_status_log_time = now

        intruder = self._largest_marker(self.latest_markers, self.intruder_ids)
        marker_id = intruder.marker_id if intruder is not None else ""
        marker_area = intruder.area_ratio if intruder is not None else ""
        min_range = self._minimum_range()
        reaction_time = ""
        if self.first_intruder_seen_time is not None and self.state == DriveState.EMERGENCY_STOP:
            reaction_time = now - self.first_intruder_seen_time

        self.csv_writer.writerow(
            {
                "time": f"{now:.3f}",
                "scenario_level": self.scenario_level,
                "state": self.state.value,
                "event": event,
                "lane_detected": int(self.latest_lane.detected),
                "lane_error": f"{self.latest_lane.error:.4f}",
                "lane_confidence": f"{self.latest_lane.confidence:.3f}",
                "marker_id": marker_id,
                "marker_area": f"{marker_area:.6f}" if marker_area != "" else "",
                "min_range": f"{min_range:.3f}" if min_range is not None else "",
                "reaction_time": f"{reaction_time:.3f}" if reaction_time != "" else "",
            }
        )
        self.csv_file.flush()

    def _open_log(self) -> Tuple[csv.DictWriter, object]:
        path = str(self.get_parameter("metrics_path").value)
        os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
        exists = os.path.exists(path) and os.path.getsize(path) > 0
        csv_file = open(path, "a", newline="")
        writer = csv.DictWriter(
            csv_file,
            fieldnames=[
                "time",
                "scenario_level",
                "state",
                "event",
                "lane_detected",
                "lane_error",
                "lane_confidence",
                "marker_id",
                "marker_area",
                "min_range",
                "reaction_time",
            ],
        )
        if not exists:
            writer.writeheader()
        return writer, csv_file

    def _now_seconds(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _str_list(self, name: str) -> List[str]:
        value = self.get_parameter(name).value
        return [str(item) for item in value]

    def _int_list(self, name: str) -> List[int]:
        value = self.get_parameter(name).value
        return [int(item) for item in value]

    def destroy_node(self) -> None:
        self._publish_stop()
        try:
            self.csv_file.close()
        except Exception:
            pass
        super().destroy_node()


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = GuardianDriveNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
