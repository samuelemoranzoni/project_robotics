import json
import math
from typing import List, Optional

import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose2D, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import ColorRGBA, Int32, String

from .perception import (
    CameraObjectDetector,
    MultiLaneDetector,
    PerceptionResult,
    SafeLaneSelector,
    draw_debug,
)


class SafeLaneNode(Node):
    """Camera-based safe lane selection controller."""

    def __init__(self) -> None:
        super().__init__("safe_lane_node")
        self._declare_parameters()

        self.bridge = CvBridge()
        self.lane_detector = MultiLaneDetector(
            roi_top=float(self.get_parameter("lane_roi_top").value),
            min_line_length=int(self.get_parameter("lane_min_line_length").value),
            max_line_gap=int(self.get_parameter("lane_max_line_gap").value),
            cluster_px=int(self.get_parameter("lane_cluster_px").value),
            min_lane_width=float(self.get_parameter("min_lane_width").value),
            max_lane_width=float(self.get_parameter("max_lane_width").value),
            image_center_x=float(self.get_parameter("image_center_x").value),
            ego_track_alpha=float(self.get_parameter("ego_track_alpha").value),
            ego_track_max_jump=float(self.get_parameter("ego_track_max_jump").value),
        )
        self.object_detector = CameraObjectDetector(
            min_area_ratio=float(self.get_parameter("min_object_area").value),
            enable_aruco=bool(self.get_parameter("enable_aruco").value),
            aruco_dictionary=str(self.get_parameter("aruco_dictionary").value),
            car_marker_ids=self._int_list("car_marker_ids"),
            obstacle_marker_ids=self._int_list("obstacle_marker_ids"),
        )
        self.selector = SafeLaneSelector(
            switch_penalty=float(self.get_parameter("switch_penalty").value),
            object_lane_penalty=float(self.get_parameter("object_lane_penalty").value),
            adjacent_lane_penalty=float(self.get_parameter("adjacent_lane_penalty").value),
            prefer_center_weight=float(self.get_parameter("prefer_center_weight").value),
        )

        self.cmd_pub = self.create_publisher(Twist, str(self.get_parameter("cmd_vel_topic").value), 10)
        self.selected_lane_pub = self.create_publisher(Int32, str(self.get_parameter("selected_lane_topic").value), 10)
        self.ego_lane_pub = self.create_publisher(Int32, str(self.get_parameter("ego_lane_topic").value), 10)
        self.status_pub = self.create_publisher(String, str(self.get_parameter("status_topic").value), 10)
        self.led_pub = self.create_publisher(ColorRGBA, str(self.get_parameter("led_topic").value), 10)

        self.debug_enabled = bool(self.get_parameter("debug_image").value)
        self.debug_pub = None
        if self.debug_enabled:
            self.debug_pub = self.create_publisher(Image, str(self.get_parameter("debug_image_topic").value), 3)

        self.create_subscription(Image, str(self.get_parameter("image_topic").value), self._image_callback, 3)
        self.create_subscription(Odometry, str(self.get_parameter("odom_topic").value), self._odom_callback, 10)
        self.create_subscription(Pose2D, str(self.get_parameter("world_pose_topic").value), self._world_pose_callback, 10)
        self.create_timer(float(self.get_parameter("watchdog_period").value), self._watchdog)

        self.previous_lane = -1
        self.desired_lane = -1
        self.localized_ego_lane = -1
        self.latest_image_time: Optional[float] = None
        self.latest_pose = None
        self.latest_world_pose = None
        self.latest_yaw = None
        self.lane_pose_source = "none"
        self.latest_world_lane_error_y: Optional[float] = None
        self.last_cmd_linear_x = 0.0
        self.last_cmd_angular_z = 0.0
        self.last_control_reason = "init"
        self.last_selected_risk = 0.0
        self.last_led = ""
        self.get_logger().info("Safe lane selection node started.")

    def _declare_parameters(self) -> None:
        self.declare_parameter("image_topic", "camera/image_color")
        self.declare_parameter("odom_topic", "odom")
        self.declare_parameter("world_pose_topic", "world_pose")
        self.declare_parameter("cmd_vel_topic", "cmd_vel")
        self.declare_parameter("selected_lane_topic", "safe_lane/selected_lane")
        self.declare_parameter("ego_lane_topic", "safe_lane/ego_lane")
        self.declare_parameter("status_topic", "safe_lane/status")
        self.declare_parameter("led_topic", "leds/color")
        self.declare_parameter("debug_image_topic", "safe_lane/debug_image")
        self.declare_parameter("debug_image", True)

        self.declare_parameter("lane_roi_top", 0.43)
        self.declare_parameter("lane_min_line_length", 35)
        self.declare_parameter("lane_max_line_gap", 18)
        self.declare_parameter("lane_cluster_px", 42)
        self.declare_parameter("min_lane_width", 0.12)
        self.declare_parameter("max_lane_width", 0.45)
        self.declare_parameter("ego_track_alpha", 0.30)
        self.declare_parameter("ego_track_max_jump", 0.18)

        self.declare_parameter("enable_aruco", False)
        self.declare_parameter("aruco_dictionary", "DICT_4X4_50")
        self.declare_parameter("car_marker_ids", [21, 22, 23])
        self.declare_parameter("obstacle_marker_ids", [31, 32, 33])
        self.declare_parameter("min_object_area", 0.002)
        self.declare_parameter("enable_object_detection", True)
        self.declare_parameter("enable_obstacle_braking", False)

        self.declare_parameter("switch_penalty", 0.35)
        self.declare_parameter("object_lane_penalty", 2.5)
        self.declare_parameter("adjacent_lane_penalty", 0.35)
        self.declare_parameter("prefer_center_weight", 0.25)
        self.declare_parameter("enable_safe_lane_selection", False)

        self.declare_parameter("base_speed", 0.22)
        self.declare_parameter("caution_speed", 0.10)
        self.declare_parameter("search_turn_speed", 0.0)
        self.declare_parameter("lane_kp", 1.35)
        self.declare_parameter("orientation_kp", 0.70)
        self.declare_parameter("lateral_kp", 0.25)
        self.declare_parameter("max_angular_speed", 1.10)
        self.declare_parameter("max_lateral_speed", 0.16)
        self.declare_parameter("use_lateral_velocity", False)
        self.declare_parameter("selected_lane_risk_stop", 1.0)
        self.declare_parameter("selected_lane_risk_slow", 0.55)
        self.declare_parameter("orientation_ok_deg", 10.0)
        self.declare_parameter("image_timeout", 1.0)
        self.declare_parameter("watchdog_period", 0.2)
        # Stable physical lane id from localization. The scene has four
        # drivable bands between the two yellow borders and the three dashed
        # separators. Keep this on for the intermediate milestone so ego-lane
        # identification does not depend on how many dashed segments Hough
        # happened to see in one image.
        self.declare_parameter("use_localization_ego_lane", True)
        self.declare_parameter("use_odom_lane_fallback", False)
        self.declare_parameter("world_lane_centers_y", [-1.66, -0.65, 0.65, 1.66])
        self.declare_parameter("world_lane_boundaries_y", [-2.02, -1.30, 0.0, 1.30, 2.02])
        self.declare_parameter("localization_lane_hysteresis_m", 0.18)
        self.declare_parameter("use_world_lane_control", True)
        self.declare_parameter("world_lane_kp", 0.22)
        # Image x-coordinate that corresponds to "robot is centered in its lane".
        # 0.5 is the optical axis; raise this if the camera/gimbal yaws right.
        self.declare_parameter("image_center_x", 0.5)
        # Orientation reading baseline (deg) caused by the same camera yaw bias.
        # Subtracted from perception.lane.orientation_error_deg so that "no
        # actual heading error" really means zero after the offset.
        self.declare_parameter("orientation_offset_deg", 0.0)

    def _image_callback(self, msg: Image) -> None:
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().warning(f"Cannot convert image: {exc}")
            return

        self.latest_image_time = self._now()
        lane = self.lane_detector.detect(frame)
        logical_lane = self._localized_ego_lane()
        if logical_lane >= 0:
            lane.current_lane = logical_lane
            if self.desired_lane < 0:
                self.desired_lane = logical_lane

        objects = []
        if bool(self.get_parameter("enable_object_detection").value):
            objects = self.object_detector.detect(frame, lane.lane_centers)
        perception = PerceptionResult(lane=lane, objects=objects)

        if bool(self.get_parameter("enable_safe_lane_selection").value):
            selected_lane, scores = self.selector.select(lane, objects, self.previous_lane)
            controller_lane = selected_lane
        elif lane.detected:
            controller_lane = lane.target_lane_index
            if controller_lane < 0 or controller_lane >= len(lane.lane_centers):
                controller_lane = self._nearest_visual_lane(lane.lane_centers)
            # For the intermediate milestone, publish/debug the stable physical
            # lane id, while the controller uses controller_lane/target_center_x.
            selected_lane = self.desired_lane if self.desired_lane >= 0 else lane.current_lane
            if selected_lane < 0:
                selected_lane = controller_lane
            scores = [1.0 if idx == controller_lane else 0.0 for idx in range(len(lane.lane_centers))]
        else:
            selected_lane, controller_lane, scores = -1, -1, []

        self._publish_selected_lane(selected_lane)
        self._publish_ego_lane(lane.current_lane)
        self._publish_control(perception, selected_lane, controller_lane, scores)
        self._publish_status(perception, selected_lane, scores)

        if selected_lane >= 0:
            self.previous_lane = selected_lane

        if self.debug_enabled and self.debug_pub is not None:
            debug = draw_debug(frame, perception, selected_lane, scores, self._debug_pose_lines())
            self.debug_pub.publish(self.bridge.cv2_to_imgmsg(debug, encoding="bgr8"))

    def _odom_callback(self, msg: Odometry) -> None:
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        self.latest_pose = (float(position.x), float(position.y))
        self.latest_yaw = self._yaw_from_quaternion(orientation.x, orientation.y, orientation.z, orientation.w)

    def _world_pose_callback(self, msg: Pose2D) -> None:
        self.latest_world_pose = (float(msg.x), float(msg.y), float(msg.theta))

    def _publish_control(
        self,
        perception: PerceptionResult,
        selected_lane: int,
        controller_lane: int,
        scores: List[float],
    ) -> None:
        if not perception.lane.detected or selected_lane < 0:
            angular = float(self.get_parameter("search_turn_speed").value)
            reason = "no_lane" if not perception.lane.detected else "no_selected_lane"
            self._publish_cmd(0.0, angular, reason)
            self._set_led("search")
            return

        world_control = (
            bool(self.get_parameter("use_world_lane_control").value)
            and self.latest_world_pose is not None
            and not bool(self.get_parameter("enable_safe_lane_selection").value)
        )
        if world_control:
            world_centers = [float(value) for value in self.get_parameter("world_lane_centers_y").value]
            if selected_lane < 0 or selected_lane >= len(world_centers):
                self._publish_cmd(0.0, 0.0, "invalid_world_lane")
                self._set_led("search")
                return
            target_y = world_centers[selected_lane]
            current_y = float(self.latest_world_pose[1])
            world_error_y = target_y - current_y
            self.latest_world_lane_error_y = world_error_y
            target_center = perception.lane.target_center_x
        elif bool(self.get_parameter("enable_safe_lane_selection").value):
            if controller_lane < 0 or controller_lane >= len(perception.lane.lane_centers):
                self._publish_cmd(0.0, 0.0, "invalid_controller_lane")
                self._set_led("search")
                return
            target_center = perception.lane.lane_centers[controller_lane]
            world_error_y = None
        else:
            target_center = perception.lane.target_center_x
            if not math.isfinite(target_center):
                self._publish_cmd(0.0, 0.0, "invalid_target_center")
                self._set_led("search")
                return
            world_error_y = None
            self.latest_world_lane_error_y = None
        image_center_x = float(self.get_parameter("image_center_x").value)
        orientation_offset_deg = float(self.get_parameter("orientation_offset_deg").value)
        lateral_error = target_center - image_center_x
        corrected_orient_deg = perception.lane.orientation_error_deg - orientation_offset_deg
        orientation_error = math.radians(corrected_orient_deg)
        use_object_risk_for_speed = bool(self.get_parameter("enable_obstacle_braking").value) or bool(
            self.get_parameter("enable_safe_lane_selection").value
        )
        selected_risk = (
            self._selected_lane_risk(perception, controller_lane)
            if use_object_risk_for_speed and controller_lane >= 0
            else 0.0
        )
        self.last_selected_risk = selected_risk

        if selected_risk >= float(self.get_parameter("selected_lane_risk_stop").value):
            self._publish_cmd(0.0, 0.0, "obstacle_stop")
            self._set_led("stop")
            return

        speed = float(self.get_parameter("base_speed").value)
        if selected_risk >= float(self.get_parameter("selected_lane_risk_slow").value):
            speed = float(self.get_parameter("caution_speed").value)
            self._set_led("caution")
            reason = "obstacle_caution"
        elif selected_lane != perception.lane.current_lane:
            speed *= 0.75
            self._set_led("lane_change")
            reason = "lane_change"
        else:
            self._set_led("drive")
            reason = "drive"

        lane_kp = float(self.get_parameter("lane_kp").value)
        orientation_kp = float(self.get_parameter("orientation_kp").value)
        if world_error_y is not None:
            angular = float(self.get_parameter("world_lane_kp").value) * world_error_y
        else:
            angular = -lane_kp * lateral_error - orientation_kp * orientation_error
        max_angular = float(self.get_parameter("max_angular_speed").value)
        angular = max(-max_angular, min(max_angular, angular))

        msg = Twist()
        msg.linear.x = speed
        msg.angular.z = angular

        if bool(self.get_parameter("use_lateral_velocity").value):
            max_lateral = float(self.get_parameter("max_lateral_speed").value)
            lateral = float(self.get_parameter("lateral_kp").value) * lateral_error
            msg.linear.y = max(-max_lateral, min(max_lateral, lateral))

        self._publish_cmd(msg.linear.x, msg.angular.z, reason, msg)

    def _publish_cmd(
        self,
        linear_x: float,
        angular_z: float,
        reason: str,
        msg: Optional[Twist] = None,
    ) -> None:
        if msg is None:
            msg = Twist()
            msg.linear.x = linear_x
            msg.angular.z = angular_z
        self.last_cmd_linear_x = float(msg.linear.x)
        self.last_cmd_angular_z = float(msg.angular.z)
        self.last_control_reason = reason
        self.cmd_pub.publish(msg)

    def _selected_lane_risk(self, perception: PerceptionResult, selected_lane: int) -> float:
        risks = [obj.near_score for obj in perception.objects if obj.lane_index == selected_lane]
        return max(risks) if risks else 0.0

    def _publish_selected_lane(self, selected_lane: int) -> None:
        msg = Int32()
        msg.data = int(selected_lane)
        self.selected_lane_pub.publish(msg)

    def _publish_ego_lane(self, ego_lane: int) -> None:
        msg = Int32()
        msg.data = int(ego_lane)
        self.ego_lane_pub.publish(msg)

    def _localized_ego_lane(self) -> int:
        self.lane_pose_source = "vision"
        if not bool(self.get_parameter("use_localization_ego_lane").value):
            return -1
        if self.latest_world_pose is not None:
            pose_y = float(self.latest_world_pose[1])
            self.lane_pose_source = "world"
        elif bool(self.get_parameter("use_odom_lane_fallback").value) and self.latest_pose is not None:
            pose_y = float(self.latest_pose[1])
            self.lane_pose_source = "odom"
        else:
            return -1
        lane_centers = [float(value) for value in self.get_parameter("world_lane_centers_y").value]
        if not lane_centers:
            return -1
        boundaries = [float(value) for value in self.get_parameter("world_lane_boundaries_y").value]
        if len(boundaries) == len(lane_centers) + 1:
            nearest = -1
            for index in range(len(boundaries) - 1):
                low = min(boundaries[index], boundaries[index + 1])
                high = max(boundaries[index], boundaries[index + 1])
                if low <= pose_y <= high:
                    nearest = index
                    break
            if nearest < 0:
                distances = [abs(pose_y - center_y) for center_y in lane_centers]
                nearest = int(min(range(len(distances)), key=distances.__getitem__))
            self.localized_ego_lane = nearest
            return self.localized_ego_lane

        distances = [abs(pose_y - center_y) for center_y in lane_centers]
        nearest = int(min(range(len(distances)), key=distances.__getitem__))
        if self.localized_ego_lane < 0 or self.localized_ego_lane >= len(lane_centers):
            self.localized_ego_lane = nearest
            return self.localized_ego_lane

        current_distance = distances[self.localized_ego_lane]
        hysteresis = float(self.get_parameter("localization_lane_hysteresis_m").value)
        if nearest != self.localized_ego_lane and distances[nearest] + hysteresis < current_distance:
            self.localized_ego_lane = nearest
        return self.localized_ego_lane

    def _nearest_visual_lane(self, lane_centers: List[float]) -> int:
        if not lane_centers:
            return -1
        image_center_x = float(self.get_parameter("image_center_x").value)
        return int(min(range(len(lane_centers)), key=lambda idx: abs(lane_centers[idx] - image_center_x)))

    def _debug_pose_lines(self) -> List[str]:
        hold = f" hold#={self.desired_lane + 1}" if self.desired_lane >= 0 else ""
        err = (
            f" yerr={self.latest_world_lane_error_y:+.2f}"
            if self.latest_world_lane_error_y is not None
            else ""
        )
        cmd = f"ctrl={self.last_control_reason} lin={self.last_cmd_linear_x:.2f} ang={self.last_cmd_angular_z:.2f}"
        if self.latest_world_pose is not None:
            x, y, theta = self.latest_world_pose
            return [f"pose=world x={x:.2f} y={y:.2f} yaw={math.degrees(theta):.1f}{hold}{err}", cmd]
        if self.latest_pose is not None:
            x, y = self.latest_pose
            return [f"pose=odom x={x:.2f} y={y:.2f} source={self.lane_pose_source}{hold}{err}", cmd]
        return [f"pose=none source={self.lane_pose_source}{hold}{err}", cmd]

    def _publish_status(self, perception: PerceptionResult, selected_lane: int, scores: List[float]) -> None:
        image_center_x = float(self.get_parameter("image_center_x").value)
        orientation_offset_deg = float(self.get_parameter("orientation_offset_deg").value)
        corrected_orientation_error_deg = perception.lane.orientation_error_deg - orientation_offset_deg
        target_center_x = perception.lane.target_center_x if perception.lane.detected else None
        orientation_ok = abs(perception.lane.orientation_error_deg) <= float(
            self.get_parameter("orientation_ok_deg").value
        )
        payload = {
            "time": round(self._now(), 3),
            "step": "intermediate_lane_and_orientation",
            "selected_lane_index": selected_lane,
            "selected_lane_number": selected_lane + 1 if selected_lane >= 0 else -1,
            "selected_lane": selected_lane,
            "desired_lane_index": self.desired_lane,
            "desired_lane_number": self.desired_lane + 1 if self.desired_lane >= 0 else -1,
            "ego_lane_index": perception.lane.current_lane,
            "ego_lane_number": perception.lane.current_lane + 1 if perception.lane.current_lane >= 0 else -1,
            "ego_lane": perception.lane.current_lane,
            "visual_target_lane": perception.lane.target_lane_index,
            "localized_ego_lane": self.localized_ego_lane,
            "lane_pose_source": self.lane_pose_source,
            "lane_detected": perception.lane.detected,
            "lane_centers": [round(center, 3) for center in perception.lane.lane_centers],
            "lane_scores": [round(score, 3) for score in scores],
            "target_center_x": round(target_center_x, 3) if target_center_x is not None else None,
            "world_lane_error_y": (
                round(self.latest_world_lane_error_y, 3)
                if self.latest_world_lane_error_y is not None
                else None
            ),
            "image_center_x": round(image_center_x, 3),
            "lateral_error": round(perception.lane.lateral_error, 3),
            "orientation_error_deg": round(perception.lane.orientation_error_deg, 2),
            "corrected_orientation_error_deg": round(corrected_orientation_error_deg, 2),
            "orientation_ok": bool(orientation_ok),
            "object_detection_enabled": bool(self.get_parameter("enable_object_detection").value),
            "obstacle_braking_enabled": bool(self.get_parameter("enable_obstacle_braking").value),
            "safe_lane_selection_enabled": bool(self.get_parameter("enable_safe_lane_selection").value),
            "control_reason": self.last_control_reason,
            "cmd_linear_x": round(self.last_cmd_linear_x, 3),
            "cmd_angular_z": round(self.last_cmd_angular_z, 3),
            "selected_lane_risk_used_for_control": round(self.last_selected_risk, 3),
            "object_count": len(perception.objects),
            "max_object_risk": round(max([obj.near_score for obj in perception.objects], default=0.0), 3),
            "objects": [
                {
                    "kind": obj.kind,
                    "id": obj.object_id,
                    "lane_index": obj.lane_index,
                    "lane_number": obj.lane_index + 1 if obj.lane_index >= 0 else -1,
                    "x": round(obj.center_x, 3),
                    "y": round(obj.center_y, 3),
                    "area": round(obj.area, 4),
                    "confidence": round(obj.confidence, 3),
                    "risk": round(obj.near_score, 3),
                }
                for obj in perception.objects[:8]
            ],
        }
        if self.latest_pose is not None:
            payload["odom"] = {
                "x": round(self.latest_pose[0], 3),
                "y": round(self.latest_pose[1], 3),
                "yaw": round(self.latest_yaw or 0.0, 3),
            }
        if self.latest_world_pose is not None:
            payload["world_pose"] = {
                "x": round(self.latest_world_pose[0], 3),
                "y": round(self.latest_world_pose[1], 3),
                "yaw": round(self.latest_world_pose[2], 3),
            }
        msg = String()
        msg.data = json.dumps(payload, sort_keys=True)
        self.status_pub.publish(msg)

    def _set_led(self, mode: str) -> None:
        if self.last_led == mode:
            return
        colors = {
            "search": (0.0, 0.0, 1.0),
            "drive": (0.0, 0.8, 0.2),
            "lane_change": (0.0, 0.7, 1.0),
            "caution": (1.0, 0.75, 0.0),
            "stop": (1.0, 0.0, 0.0),
        }
        color = colors.get(mode, (0.0, 0.0, 0.0))
        msg = ColorRGBA()
        msg.r, msg.g, msg.b = color
        msg.a = 1.0
        self.led_pub.publish(msg)
        self.last_led = mode

    def _watchdog(self) -> None:
        if self.latest_image_time is None:
            return
        if self._now() - self.latest_image_time > float(self.get_parameter("image_timeout").value):
            self._publish_cmd(0.0, 0.0, "watchdog_image_timeout")
            self._set_led("search")

    def _int_list(self, name: str) -> List[int]:
        return [int(value) for value in self.get_parameter(name).value]

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    @staticmethod
    def _yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def destroy_node(self) -> None:
        self._publish_cmd(0.0, 0.0, "shutdown")
        super().destroy_node()


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = SafeLaneNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
