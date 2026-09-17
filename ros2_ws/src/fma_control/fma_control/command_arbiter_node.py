"""Select fresh drive candidates with an explicitly released emergency latch."""
from dataclasses import dataclass
import math
import time

import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String

from fma_interfaces.msg import DriveCommand


@dataclass(frozen=True)
class Candidate:
    speed_mps: float
    steering_angle_rad: float
    emergency_stop: bool
    received_at: float
    use_pwm_override: bool = False
    drive_pwm_percent: int = 0


def candidate_is_valid(candidate):
    return (candidate is not None and math.isfinite(candidate.speed_mps)
            and math.isfinite(candidate.steering_angle_rad))


def select_source(lane, mission, emergency_latched, now, lane_timeout, mission_timeout,
                  manual=None, manual_timeout=0.5, gps=None, gps_timeout=0.5,
                  control_mode=None):
    """Pure selection using local receive times; timeout boundary is stale."""
    if candidate_is_valid(manual) and 0 <= now - manual.received_at < manual_timeout:
        return 'manual'
    if emergency_latched:
        return 'emergency'

    # Mission safety STOP always outranks autonomous motion controllers.
    if (candidate_is_valid(mission)
            and 0 <= now - mission.received_at < mission_timeout):
        return 'mission'

    # Autonomous controllers are mode-gated. Never fall back to lane in GPS mode.
    if control_mode == 'GPS':
        if (candidate_is_valid(gps)
                and 0 <= now - gps.received_at < gps_timeout):
            return 'gps'
        return None

    # None preserves the legacy startup behaviour until the first mode arrives.
    if control_mode in (None, 'LANE'):
        if (candidate_is_valid(lane)
                and 0 <= now - lane.received_at < lane_timeout):
            return 'lane'

    return None


class CommandArbiterNode(Node):
    """Default mutually-exclusive callbacks serialize input and timer state."""

    def __init__(self):
        super().__init__('command_arbiter')
        defaults = {
            'lane_topic': '/cmd/lane', 'mission_topic': '/cmd/mission',
            'gps_topic': '/cmd/gps',
            'control_mode_topic': '/mission/control_mode',
            'emergency_topic': '/cmd/emergency', 'final_topic': '/cmd/final',
            'manual_topic': '/cmd/manual', 'manual_timeout_sec': 0.5,
            'lane_timeout_sec': 0.5, 'mission_timeout_sec': 0.5,
            'gps_timeout_sec': 0.5,
            'emergency_timeout_sec': 0.5, 'output_rate_hz': 20.0,
        }
        self.config = {
            name: self.declare_parameter(
                name, value, ParameterDescriptor(read_only=True)).value
            for name, value in defaults.items()
        }
        for name in ('lane_timeout_sec', 'mission_timeout_sec', 'gps_timeout_sec',
                     'manual_timeout_sec', 'emergency_timeout_sec', 'output_rate_hz'):
            if not math.isfinite(self.config[name]) or self.config[name] <= 0:
                raise ValueError(f'{name} must be finite and positive')
        self.candidates = {
            'lane': None, 'mission': None, 'gps': None, 'manual': None}
        self.control_mode = None
        self.emergency_latched = False
        self.last_emergency = None
        self.warning_times = {}
        self.publisher = self.create_publisher(DriveCommand, self.config['final_topic'], 1)
        self.lane_subscription = self.create_subscription(
            DriveCommand, self.config['lane_topic'], self.on_lane, 1)
        self.mission_subscription = self.create_subscription(
            DriveCommand, self.config['mission_topic'], self.on_mission, 1)
        self.gps_subscription = self.create_subscription(
            DriveCommand, self.config['gps_topic'], self.on_gps, 1)
        self.manual_subscription = self.create_subscription(
            DriveCommand, self.config['manual_topic'], self.on_manual, 1)
        self.emergency_subscription = self.create_subscription(
            DriveCommand, self.config['emergency_topic'], self.on_emergency, 1)
        self.control_mode_subscription = self.create_subscription(
            String, self.config['control_mode_topic'], self.on_control_mode,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.output_timer = self.create_timer(
            1.0 / self.config['output_rate_hz'], self.publish_output,
            clock=Clock(clock_type=ClockType.STEADY_TIME))

    def warn(self, key, message, now):
        if now - self.warning_times.get(key, -math.inf) >= 5.0:
            self.get_logger().warning(message)
            self.warning_times[key] = now

    def receive_candidate(self, source, msg):
        now = time.monotonic()
        candidate = Candidate(msg.speed_mps, msg.steering_angle_rad, msg.emergency_stop, now,
                              msg.use_pwm_override, msg.drive_pwm_percent)
        # Invalid new input also invalidates the previous command from this source.
        self.candidates[source] = candidate if candidate_is_valid(candidate) else None
        if self.candidates[source] is None:
            self.warn(source, f'Invalid {source} candidate ignored: non-finite speed/steering', now)

    def on_lane(self, msg):
        self.receive_candidate('lane', msg)

    def on_mission(self, msg):
        self.receive_candidate('mission', msg)

    def on_gps(self, msg):
        self.receive_candidate('gps', msg)

    def on_control_mode(self, msg):
        self.control_mode = msg.data

    def on_manual(self, msg):
        self.receive_candidate('manual', msg)

    def on_emergency(self, msg):
        # A newly received message is fresh locally, regardless of sender stamp.
        # Numeric fields never influence either SET or explicit CLEAR.
        self.last_emergency = time.monotonic()
        self.emergency_latched = msg.emergency_stop

    def publish_output(self):
        now = time.monotonic()
        source = select_source(
            self.candidates['lane'], self.candidates['mission'], self.emergency_latched,
            now, self.config['lane_timeout_sec'], self.config['mission_timeout_sec'],
            manual=self.candidates['manual'],
            manual_timeout=self.config['manual_timeout_sec'],
            gps=self.candidates['gps'],
            gps_timeout=self.config['gps_timeout_sec'],
            control_mode=self.control_mode)
        if (self.emergency_latched and self.last_emergency is not None
                and now - self.last_emergency >= self.config['emergency_timeout_sec']):
            self.warn('emergency_stale', 'Emergency input stale; latch remains set', now)
        output = DriveCommand()
        output.header.stamp = self.get_clock().now().to_msg()
        output.header.frame_id = ''
        if source in ('lane', 'mission', 'gps', 'manual'):
            candidate = self.candidates[source]
            output.speed_mps = candidate.speed_mps
            output.steering_angle_rad = candidate.steering_angle_rad
            output.emergency_stop = candidate.emergency_stop
            # Autonomous sources cannot inherit/request manual duty override.
            output.use_pwm_override = source == 'manual' and candidate.use_pwm_override
            output.drive_pwm_percent = candidate.drive_pwm_percent if source == 'manual' else 0
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
