"""GPS-primary steering with lane-departure correction."""
from dataclasses import dataclass
import math
import time

import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

from fma_interfaces.msg import DriveCommand, Lane


@dataclass(frozen=True)
class GuardConfig:
    enabled: bool = False

    confidence_threshold: float = 0.6
    lane_timeout_sec: float = 0.5
    gps_timeout_sec: float = 0.5

    # These are lane-center error thresholds, NOT literal boundary distance.
    engage_error_m: float = 0.35
    release_error_m: float = 0.20

    k_lateral: float = 0.5
    k_heading: float = 0.4
    max_correction_rad: float = 0.10

    steering_left_limit_rad: float = 0.2810
    steering_right_limit_rad: float = -0.3054

    output_rate_hz: float = 20.0

    def __post_init__(self):
        if not isinstance(self.enabled, bool):
            raise ValueError('enabled must be boolean')

        numeric = [
            self.confidence_threshold,
            self.lane_timeout_sec,
            self.gps_timeout_sec,
            self.engage_error_m,
            self.release_error_m,
            self.k_lateral,
            self.k_heading,
            self.max_correction_rad,
            self.steering_left_limit_rad,
            self.steering_right_limit_rad,
            self.output_rate_hz,
        ]

        if not all(math.isfinite(v) for v in numeric):
            raise ValueError('lane guard parameters must be finite')

        if not 0.0 <= self.confidence_threshold <= 1.0:
            raise ValueError('confidence_threshold must be 0..1')

        if self.release_error_m < 0:
            raise ValueError('release_error_m must be nonnegative')

        if self.engage_error_m <= self.release_error_m:
            raise ValueError(
                'engage_error_m must be greater than release_error_m'
            )

        if min(
            self.lane_timeout_sec,
            self.gps_timeout_sec,
            self.max_correction_rad,
            self.output_rate_hz,
        ) <= 0:
            raise ValueError('timeouts/rate/correction must be positive')

        if self.steering_left_limit_rad <= 0:
            raise ValueError('left steering limit must be positive')

        if self.steering_right_limit_rad >= 0:
            raise ValueError('right steering limit must be negative')


@dataclass(frozen=True)
class LaneSample:
    detected: bool
    lateral_error_m: float
    heading_error_rad: float
    curvature: float
    confidence: float
    received_at: float


@dataclass(frozen=True)
class GpsCommand:
    speed_mps: float
    steering_angle_rad: float
    emergency_stop: bool
    use_pwm_override: bool
    drive_pwm_percent: int
    received_at: float


def clamp_steering(value, config):
    return max(
        config.steering_right_limit_rad,
        min(config.steering_left_limit_rad, value),
    )


def lane_usable(sample, now, config):
    if sample is None or not sample.detected:
        return False

    values = (
        sample.lateral_error_m,
        sample.heading_error_rad,
        sample.curvature,
        sample.confidence,
        sample.received_at,
        now,
    )

    if not all(math.isfinite(v) for v in values):
        return False

    if not 0.0 <= now - sample.received_at < config.lane_timeout_sec:
        return False

    return sample.confidence >= config.confidence_threshold


def guarded_steering(gps_steering, lane, now, engaged, config):
    """Return steering, new_engaged, correction.

    Positive lane lateral error uses the existing project convention:
    positive steering corrects LEFT.
    """
    gps_steering = clamp_steering(gps_steering, config)

    if not config.enabled or not lane_usable(lane, now, config):
        return gps_steering, False, 0.0

    error = abs(lane.lateral_error_m)

    if engaged:
        engaged = error > config.release_error_m
    else:
        engaged = error >= config.engage_error_m

    if not engaged:
        return gps_steering, False, 0.0

    correction = (
        config.k_lateral * lane.lateral_error_m
        + config.k_heading * lane.heading_error_rad
    )

    correction = max(
        -config.max_correction_rad,
        min(config.max_correction_rad, correction),
    )

    return (
        clamp_steering(gps_steering + correction, config),
        True,
        correction,
    )


class LaneGuardNode(Node):
    def __init__(self):
        super().__init__('lane_guard')

        self.config = GuardConfig(**{
            key: self.declare_parameter(
                key,
                value,
                ParameterDescriptor(read_only=True),
            ).value
            for key, value in vars(GuardConfig()).items()
        })

        self.lane = None
        self.gps = None
        self.engaged = False

        self.output = self.create_publisher(
            DriveCommand,
            '/cmd/gps',
            1,
        )

        self.state_pub = self.create_publisher(
            String,
            '/lane_guard/state',
            1,
        )

        self.gps_sub = self.create_subscription(
            DriveCommand,
            '/cmd/gps_raw',
            self.on_gps,
            1,
        )

        self.lane_sub = self.create_subscription(
            Lane,
            '/perception/lane',
            self.on_lane,
            1,
        )

        self.timer = self.create_timer(
            1.0 / self.config.output_rate_hz,
            self.publish_command,
            clock=Clock(clock_type=ClockType.STEADY_TIME),
        )

    def on_gps(self, msg):
        self.gps = GpsCommand(
            speed_mps=msg.speed_mps,
            steering_angle_rad=msg.steering_angle_rad,
            emergency_stop=msg.emergency_stop,
            use_pwm_override=msg.use_pwm_override,
            drive_pwm_percent=msg.drive_pwm_percent,
            received_at=time.monotonic(),
        )

    def on_lane(self, msg):
        self.lane = LaneSample(
            detected=msg.detected,
            lateral_error_m=msg.lateral_error_m,
            heading_error_rad=msg.heading_error_rad,
            curvature=msg.curvature,
            confidence=msg.confidence,
            received_at=time.monotonic(),
        )

    def publish_command(self):
        now = time.monotonic()

        msg = DriveCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = ''

        if (
            self.gps is None
            or not math.isfinite(self.gps.received_at)
            or not 0.0 <= now - self.gps.received_at < self.config.gps_timeout_sec
        ):
            self.engaged = False
            msg.speed_mps = 0.0
            msg.steering_angle_rad = 0.0
            msg.emergency_stop = True
            msg.use_pwm_override = False
            msg.drive_pwm_percent = 0

            self.output.publish(msg)
            self.state_pub.publish(String(data='GPS_STALE'))
            return

        msg.speed_mps = self.gps.speed_mps
        msg.emergency_stop = self.gps.emergency_stop

        # Autonomous command never requests manual PWM override.
        msg.use_pwm_override = False
        msg.drive_pwm_percent = 0

        if self.gps.emergency_stop:
            self.engaged = False
            msg.steering_angle_rad = self.gps.steering_angle_rad

            self.output.publish(msg)
            self.state_pub.publish(String(data='GPS_STOP'))
            return

        steering, self.engaged, correction = guarded_steering(
            self.gps.steering_angle_rad,
            self.lane,
            now,
            self.engaged,
            self.config,
        )

        msg.steering_angle_rad = steering
        self.output.publish(msg)

        if not self.config.enabled:
            state = 'DISABLED'
        elif not lane_usable(self.lane, now, self.config):
            state = 'NO_LANE'
        elif not self.engaged:
            state = 'PASS'
        elif correction > 0:
            state = 'CORRECT_LEFT'
        elif correction < 0:
            state = 'CORRECT_RIGHT'
        else:
            state = 'GUARD'

        self.state_pub.publish(String(data=state))


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = LaneGuardNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except ValueError as error:
        rclpy.logging.get_logger('lane_guard').error(str(error))
        return 1
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    return 0
