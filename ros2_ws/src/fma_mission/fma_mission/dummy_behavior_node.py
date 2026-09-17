"""ROS2 wrapper for mission-conditioned dummy behavior."""

from fma_interfaces.msg import MissionState, ObstacleArray
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
import rclpy

from fma_mission.dummy_behavior import (
    DummyAction,
    decide_dummy_behavior,
)


class DummyBehaviorNode(Node):
    def __init__(self):
        super().__init__('dummy_behavior')

        self.declare_parameter('avoid_trigger_distance_m', 3.0)
        self.declare_parameter('stop_trigger_distance_m', 3.0)
        self.declare_parameter('min_confidence', 0.5)
        self.declare_parameter('confirm_frames', 3)
        self.declare_parameter('dropout_frames', 2)
        self.declare_parameter('obstacle_timeout_sec', 0.5)

        self.avoid_trigger_distance_m = float(
            self.get_parameter('avoid_trigger_distance_m').value
        )
        self.stop_trigger_distance_m = float(
            self.get_parameter('stop_trigger_distance_m').value
        )
        self.min_confidence = float(
            self.get_parameter('min_confidence').value
        )
        self.confirm_frames = int(
            self.get_parameter('confirm_frames').value
        )
        self.dropout_frames = int(
            self.get_parameter('dropout_frames').value
        )
        self.obstacle_timeout_sec = float(
            self.get_parameter('obstacle_timeout_sec').value
        )

        self.current_mission = MissionState.NORMAL_DRIVE
        self.mission_active = False
        self.last_action = DummyAction.NONE
        self.candidate_action = DummyAction.NONE
        self.confirmed_action = DummyAction.NONE
        self.hit_count = 0
        self.miss_count = 0
        self.last_obstacle_rx_ns = None

        mission_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.create_subscription(
            MissionState,
            '/mission/current',
            self.on_mission,
            mission_qos,
        )

        self.create_subscription(
            ObstacleArray,
            '/perception/obstacles',
            self.on_obstacles,
            10,
        )

        self.create_timer(0.1, self.check_obstacle_freshness)

    def reset_filter(self):
        self.candidate_action = DummyAction.NONE
        self.confirmed_action = DummyAction.NONE
        self.hit_count = 0
        self.miss_count = 0

        if self.last_action is not DummyAction.NONE:
            self.get_logger().info('DUMMY_ACTION=NONE')
            self.last_action = DummyAction.NONE

    def on_mission(self, msg):
        changed = (
            msg.current_mission != self.current_mission
            or msg.active != self.mission_active
        )

        self.current_mission = msg.current_mission
        self.mission_active = msg.active

        if changed:
            self.reset_filter()

    def on_obstacles(self, msg):
        self.last_obstacle_rx_ns = self.get_clock().now().nanoseconds

        decision = decide_dummy_behavior(
            self.current_mission,
            self.mission_active,
            msg.obstacles,
            avoid_trigger_distance_m=self.avoid_trigger_distance_m,
            stop_trigger_distance_m=self.stop_trigger_distance_m,
            min_confidence=self.min_confidence,
        )

        if decision.action is not DummyAction.NONE:
            self.miss_count = 0

            if decision.action == self.candidate_action:
                self.hit_count += 1
            else:
                self.candidate_action = decision.action
                self.hit_count = 1

            if (
                self.confirmed_action is DummyAction.NONE
                and self.hit_count >= self.confirm_frames
            ):
                self.confirmed_action = decision.action

                if decision.action != self.last_action:
                    self.get_logger().info(
                        f'DUMMY_ACTION={decision.action.value} '
                        f'distance={decision.distance_m:.2f}m '
                        f'x={decision.x_m:.2f}m '
                        f'y={decision.y_m:.2f}m '
                        f'confidence={decision.confidence:.2f}'
                    )
                    self.last_action = decision.action

            return

        self.candidate_action = DummyAction.NONE
        self.hit_count = 0

        if self.confirmed_action is DummyAction.NONE:
            return

        self.miss_count += 1

        if self.miss_count <= self.dropout_frames:
            return

        self.confirmed_action = DummyAction.NONE
        self.miss_count = 0

        if self.last_action is not DummyAction.NONE:
            self.get_logger().info('DUMMY_ACTION=NONE')
            self.last_action = DummyAction.NONE

    def check_obstacle_freshness(self):
        if self.confirmed_action is DummyAction.NONE:
            return

        if self.last_obstacle_rx_ns is None:
            return

        age_sec = (
            self.get_clock().now().nanoseconds
            - self.last_obstacle_rx_ns
        ) / 1e9

        if age_sec > self.obstacle_timeout_sec:
            self.reset_filter()


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = DummyBehaviorNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    return 0
