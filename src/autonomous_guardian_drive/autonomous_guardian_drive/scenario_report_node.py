import json
from collections import Counter
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class ScenarioReportNode(Node):
    """Print a compact live summary of guardian events."""

    def __init__(self) -> None:
        super().__init__("scenario_report")
        self.declare_parameter("events_topic", "guardian/events")
        self.events = Counter()
        self.states = Counter()
        self.last_event: Optional[dict] = None
        self.create_subscription(
            String,
            self.get_parameter("events_topic").value,
            self._event_callback,
            10,
        )
        self.create_timer(5.0, self._print_summary)

    def _event_callback(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            payload = {"event": msg.data, "state": "unknown"}
        self.last_event = payload
        self.events[str(payload.get("event", "unknown"))] += 1
        self.states[str(payload.get("state", "unknown"))] += 1

    def _print_summary(self) -> None:
        if self.last_event is None:
            self.get_logger().info("Waiting for guardian events...")
            return
        event_summary = ", ".join(f"{key}={value}" for key, value in self.events.items())
        state_summary = ", ".join(f"{key}={value}" for key, value in self.states.items())
        self.get_logger().info(f"Events: {event_summary} | States: {state_summary}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ScenarioReportNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

