"""Select fresh drive candidates with an explicitly released emergency latch."""
from dataclasses import dataclass
import math
import time

import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from fma_interfaces.msg import DriveCommand


@dataclass(frozen=True)
class Candidate:
    speed_mps: float
    steering_angle_rad: float
    emergency_stop: bool
    received_at: float


def candidate_is_valid(candidate):
    return (candidate is not None and math.isfinite(candidate.speed_mps)
            and math.isfinite(candidate.steering_angle_rad))


def select_source(lane, mission, emergency_latched, now, lane_timeout, mission_timeout):
    """Pure selection using local receive times; timeout boundary is stale."""
    if emergency_latched:
        return 'emergency'
    for source, candidate, timeout in (
            ('mission', mission, mission_timeout), ('lane', lane, lane_timeout)):
        if candidate_is_valid(candidate) and 0 <= now - candidate.received_at < timeout:
            return source
    return None


class CommandArbiterNode(Node):
    """Default mutually-exclusive callbacks serialize input and timer state."""

    def __init__(self):
        super().__init__('command_arbiter')
        defaults = {
            'lane_topic': '/cmd/lane', 'mission_topic': '/cmd/mission',
            'emergency_topic': '/cmd/emergency', 'final_topic': '/cmd/final',
            'lane_timeout_sec': 0.5, 'mission_timeout_sec': 0.5,
            'emergency_timeout_sec': 0.5, 'output_rate_hz': 20.0,
        }
        self.config = {
            name: self.declare_parameter(
                name, value, ParameterDescriptor(read_only=True)).value
            for name, value in defaults.items()
        }
        for name in ('lane_timeout_sec', 'mission_timeout_sec',
                     'emergency_timeout_sec', 'output_rate_hz'):
            if not math.isfinite(self.config[name]) or self.config[name] <= 0:
                raise ValueError(f'{name} must be finite and positive')
        self.candidates = {'lane': None, 'mission': None}
        self.emergency_latched = False
        self.last_emergency = None
        self.warning_times = {}
        self.publisher = self.create_publisher(DriveCommand, self.config['final_topic'], 1)
        self.lane_subscription = self.create_subscription(
            DriveCommand, self.config['lane_topic'], self.on_lane, 1)
        self.mission_subscription = self.create_subscription(
            DriveCommand, self.config['mission_topic'], self.on_mission, 1)
        self.emergency_subscription = self.create_subscription(
            DriveCommand, self.config['emergency_topic'], self.on_emergency, 1)
        self.output_timer = self.create_timer(
            1.0 / self.config['output_rate_hz'], self.publish_output,
            clock=Clock(clock_type=ClockType.STEADY_TIME))

    def warn(self, key, message, now):
        if now - self.warning_times.get(key, -math.inf) >= 5.0:
            self.get_logger().warning(message)
            self.warning_times[key] = now

    def receive_candidate(self, source, msg):
        now = time.monotonic()
        candidate = Candidate(msg.speed_mps, msg.steering_angle_rad, msg.emergency_stop, now)
        # Invalid new input also invalidates the previous command from this source.
        self.candidates[source] = candidate if candidate_is_valid(candidate) else None
        if self.candidates[source] is None:
            self.warn(source, f'Invalid {source} candidate ignored: non-finite speed/steering', now)

    def on_lane(self, msg):
        self.receive_candidate('lane', msg)

    def on_mission(self, msg):
        self.receive_candidate('mission', msg)

    def on_emergency(self, msg):
        # A newly received message is fresh locally, regardless of sender stamp.
        # Numeric fields never influence either SET or explicit CLEAR.
        self.last_emergency = time.monotonic()
        self.emergency_latched = msg.emergency_stop

    def publish_output(self):
        now = time.monotonic()
        source = select_source(
            self.candidates['lane'], self.candidates['mission'], self.emergency_latched,
            now, self.config['lane_timeout_sec'], self.config['mission_timeout_sec'])
        if (self.emergency_latched and self.last_emergency is not None
                and now - self.last_emergency >= self.config['emergency_timeout_sec']):
            self.warn('emergency_stale', 'Emergency input stale; STOP latch remains set', now)
        output = DriveCommand()
        output.header.stamp = self.get_clock().now().to_msg()
        output.header.frame_id = ''
        if source in ('lane', 'mission'):
            candidate = self.candidates[source]
            output.speed_mps = candidate.speed_mps
            output.steering_angle_rad = candidate.steering_angle_rad
            output.emergency_stop = candidate.emergency_stop
        else:
            output.speed_mps = 0.0
            output.steering_angle_rad = 0.0
            output.emergency_stop = True
        self.publisher.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = CommandArbiterNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except ValueError as error:
        rclpy.logging.get_logger('command_arbiter').error(f'Invalid configuration: {error}')
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
