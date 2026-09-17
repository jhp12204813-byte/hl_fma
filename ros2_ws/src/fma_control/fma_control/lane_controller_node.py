"""Opt-in lane P controller with receive-time freshness and fail-safe STOP."""
from dataclasses import dataclass
import math
import time

import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from fma_interfaces.msg import DriveCommand, Lane


@dataclass(frozen=True)
class ControlConfig:
    enabled: bool = False
    k_lateral: float = 1.0
    k_heading: float = 1.0
    confidence_threshold: float = 0.6
    lane_timeout_sec: float = 0.5
    forward_speed_mps: float = 0.2
    output_rate_hz: float = 20.0

    def __post_init__(self):
        if not isinstance(self.enabled, bool):
            raise ValueError('enabled must be boolean')
        if not all(math.isfinite(v) for k, v in vars(self).items() if k != 'enabled'):
            raise ValueError('Control parameters must be finite')
        if min(self.k_lateral, self.k_heading) < 0:
            raise ValueError('Gains must be nonnegative')
        if not 0 <= self.confidence_threshold <= 1:
            raise ValueError('Confidence threshold must be in 0..1')
        if min(self.lane_timeout_sec, self.forward_speed_mps, self.output_rate_hz) <= 0:
            raise ValueError('Timeout, forward command and output rate must be positive')


@dataclass(frozen=True)
class LaneSample:
    detected: bool
    lateral_error_m: float
    heading_error_rad: float
    curvature: float
    confidence: float
    received_at: float


def lane_command(sample, now, config):
    """Return speed, steering, emergency flag; positive steering corrects left."""
    stop = (0.0, 0.0, False)
    if not config.enabled or sample is None or not sample.detected:
        return stop
    if not all(math.isfinite(v) for v in (
            sample.lateral_error_m, sample.heading_error_rad, sample.curvature,
            sample.confidence, sample.received_at, now)):
        return stop
    if not 0 <= now - sample.received_at < config.lane_timeout_sec:
        return stop
    if not config.confidence_threshold <= sample.confidence <= 1.0:
        return stop
    steering = (config.k_lateral * sample.lateral_error_m
                + config.k_heading * sample.heading_error_rad)
    if not math.isfinite(steering):
        return stop
    return config.forward_speed_mps, max(-0.3054, min(0.2810, steering)), False


class LaneControllerNode(Node):
    def __init__(self):
        super().__init__('lane_controller')
        self.config = ControlConfig(**{
            k: self.declare_parameter(k, v, ParameterDescriptor(read_only=True)).value
            for k, v in vars(ControlConfig()).items()})
        self.sample = None
        self.publisher = self.create_publisher(DriveCommand, '/cmd/lane', 1)
        self.subscription = self.create_subscription(Lane, '/perception/lane', self.on_lane, 1)
        self.timer = self.create_timer(
            1. / self.config.output_rate_hz, self.publish_command,
            clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.publish_command()

    def on_lane(self, msg):
        self.sample = LaneSample(
            msg.detected, msg.lateral_error_m, msg.heading_error_rad,
            msg.curvature, msg.confidence, time.monotonic())

    def publish_command(self):
        speed, steering, emergency = lane_command(self.sample, time.monotonic(), self.config)
        msg = DriveCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = ''
        msg.speed_mps = speed
        msg.steering_angle_rad = steering
        msg.emergency_stop = emergency
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = LaneControllerNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except ValueError as error:
        rclpy.logging.get_logger('lane_controller').error(str(error))
        return 1
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0
