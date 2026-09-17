"""Signal-car mission behavior with temporal voting."""

import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String

from fma_interfaces.msg import LaneSignal, MissionState
from fma_mission.signal_car_behavior import (
    KEEP,
    LEFT,
    WAIT,
    SignalCarConfig,
    SignalCarVoter,
)


class SignalCarBehaviorNode(Node):

    def __init__(self):
        super().__init__('signal_car_behavior')

        self.declare_parameter('window_sec', 2.0)
        self.declare_parameter('min_valid_samples', 4)
        self.declare_parameter('majority_ratio', 0.75)
        self.declare_parameter('min_confidence', 0.60)

        self.voter = SignalCarVoter(
            SignalCarConfig(
                window_sec=float(
                    self.get_parameter('window_sec').value
                ),
                min_valid_samples=int(
                    self.get_parameter('min_valid_samples').value
                ),
                majority_ratio=float(
                    self.get_parameter('majority_ratio').value
                ),
                min_confidence=float(
                    self.get_parameter('min_confidence').value
                ),
            )
        )

        self.active = False
        self.completed = False

        transient = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.request_pub = self.create_publisher(
            String,
            '/mission/lane_change_request',
            transient,
        )

        self.status_pub = self.create_publisher(
            MissionState,
            '/mission/status',
            10,
        )

        self.create_subscription(
            MissionState,
            '/mission/current',
            self.on_mission,
            transient,
        )

        self.create_subscription(
            LaneSignal,
            '/perception/lane_signal',
            self.on_signal,
            10,
        )

        self.get_logger().info(
            'Signal-car behavior ready'
        )

    def on_mission(self, msg):
        now_active = (
            msg.active
            and msg.current_mission == MissionState.SIGNAL_CAR
        )

        if now_active and not self.active:
            self.voter.reset()
            self.completed = False
            self.get_logger().info(
                'SIGNAL_CAR entered: waiting for stable signal'
            )

        if not now_active and self.active:
            self.voter.reset()

        self.active = now_active

    def on_signal(self, msg):
        if not self.active or self.completed:
            return

        decision = self.voter.update(
            time.monotonic(),
            msg.left_state,
            msg.center_state,
            msg.right_state,
            msg.left_confidence,
            msg.center_confidence,
            msg.right_confidence,
        )

        if decision == WAIT:
            return

        request = String()
        request.data = decision
        self.request_pub.publish(request)

        status = MissionState()
        status.header.stamp = self.get_clock().now().to_msg()
        status.header.frame_id = ''
        status.current_mission = MissionState.SIGNAL_CAR
        status.active = True
        status.completed = True
        self.status_pub.publish(status)

        self.completed = True

        self.get_logger().info(
            f'SIGNAL_CAR decision={decision} completed=true'
        )


def main(args=None):
    rclpy.init(args=args)
    node = SignalCarBehaviorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
