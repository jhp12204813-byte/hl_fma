"""Keep STOP/FINISH commands fresh without computing steering or drive PWM."""
import time

from fma_interfaces.msg import DriveCommand
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String

from fma_mission.control_mode import CONTROL_MODES


class MissionSafetyNode(Node):
    def __init__(self):
        super().__init__('mission_safety')
        self.mode = 'STOP'
        self.last_mode_received = None
        self.ready_at = {}
        self.publisher = self.create_publisher(DriveCommand, '/cmd/mission', 1)
        self.subscription = self.create_subscription(
            String, '/mission/control_mode', self.on_mode,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        # Volatile heartbeat: never accept a latched readiness from an old controller.
        self.ready_subscription = self.create_subscription(
            String, '/mission/controller_ready', self.on_ready, 10)
        self.timer = self.create_timer(
            .05, self.publish_stop, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.publish_stop()

    def on_ready(self, msg):
        if msg.data in ('GPS', 'OBSTACLE', 'REVERSE') and msg.data == self.mode:
            self.ready_at[msg.data] = time.monotonic()
        else:
            # STOP, unknown or wrong-mode heartbeats revoke readiness.
            self.ready_at.clear()
        self.publish_stop()

    def on_mode(self, msg):
        if self.mode == 'FINISH':
            self.publish_stop()
            return
        if msg.data != self.mode:
            self.ready_at.clear()
        self.last_mode_received = time.monotonic()
        if msg.data not in CONTROL_MODES:
            self.get_logger().warning(f'Unknown control mode {msg.data!r}; holding STOP')
            self.mode = 'STOP'
        else:
            self.mode = msg.data
        self.publish_stop()

    def publish_stop(self):
        # Manager publishes at 2 Hz. Missing mode updates fail closed after 1.5 s.
        stale = (self.last_mode_received is None
                 or time.monotonic() - self.last_mode_received >= 1.5)
        ready = (self.mode in self.ready_at
                 and time.monotonic() - self.ready_at[self.mode] < .3)
        if not stale and (self.mode == 'LANE' or (
                self.mode in ('GPS', 'OBSTACLE', 'REVERSE') and ready)):
            return
        msg = DriveCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.speed_mps = 0.0
        msg.steering_angle_rad = 0.0
        msg.emergency_stop = True
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = MissionSafetyNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
