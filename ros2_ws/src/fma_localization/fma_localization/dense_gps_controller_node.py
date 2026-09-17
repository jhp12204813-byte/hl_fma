"""Polyline/lookahead GPS controller for the dense competition route."""

from dataclasses import dataclass
import json
import math
from pathlib import Path
import time

import yaml

from fma_interfaces.msg import DriveCommand
import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool, Float64, String

from fma_localization.gps_controller_node import (
    FixSample,
    HeadingSample,
    bearing_deg,
    fresh,
    valid_coordinates,
    wrap_degrees,
)
from fma_localization.polyline_tracker import (
    PolylineTracker,
    RoutePoint,
)


@dataclass(frozen=True)
class DenseControlConfig:
    enabled: bool = False

    # Empty means: use defaults.lookahead_m from competition_route.yaml
    lookahead_m: float = 0.0

    k_heading: float = 1.0
    forward_speed_mps: float = 0.2

    fix_timeout_sec: float = 1.5
    heading_timeout_sec: float = 1.5
    health_timeout_sec: float = 1.5
    mode_timeout_sec: float = 1.5

    output_rate_hz: float = 20.0
    heading_offset_deg: float = 0.0

    max_left_steering_rad: float = 0.2810
    max_right_steering_rad: float = 0.3054

    # If the vehicle is farther than this from the route,
    # do not publish a motion-ready heartbeat.
    max_cross_track_error_m: float = 3.0

    # Search only near the remembered route progress.
    search_forward_segments: int = 30

    finish_tolerance_m: float = 1.0

    def __post_init__(self):
        if not isinstance(self.enabled, bool):
            raise ValueError('enabled must be boolean')

        numeric = (
            self.lookahead_m,
            self.k_heading,
            self.forward_speed_mps,
            self.fix_timeout_sec,
            self.heading_timeout_sec,
            self.health_timeout_sec,
            self.mode_timeout_sec,
            self.output_rate_hz,
            self.heading_offset_deg,
            self.max_left_steering_rad,
            self.max_right_steering_rad,
            self.max_cross_track_error_m,
            self.finish_tolerance_m,
        )

        if not all(math.isfinite(v) for v in numeric):
            raise ValueError('Dense GPS parameters must be finite')

        if self.lookahead_m < 0:
            raise ValueError('lookahead_m must be >= 0')

        if self.k_heading < 0:
            raise ValueError('k_heading must be nonnegative')

        if min(
            self.forward_speed_mps,
            self.fix_timeout_sec,
            self.heading_timeout_sec,
            self.health_timeout_sec,
            self.mode_timeout_sec,
            self.output_rate_hz,
            self.max_left_steering_rad,
            self.max_right_steering_rad,
            self.max_cross_track_error_m,
        ) <= 0:
            raise ValueError('Dense GPS speed/time/rate/limits must be positive')

        if self.finish_tolerance_m < 0:
            raise ValueError('finish_tolerance_m must be nonnegative')

        if (
            type(self.search_forward_segments) is not int
            or self.search_forward_segments < 1
        ):
            raise ValueError(
                'search_forward_segments must be positive integer'
            )


def load_dense_route(path):
    path = Path(path)

    if not path.is_file():
        raise ValueError(f'Dense route file not found: {path}')

    with path.open(encoding='utf-8') as stream:
        data = yaml.safe_load(stream)

    if not isinstance(data, dict):
        raise ValueError('Dense route root must be mapping')

    if data.get('profile') != 'competition_dense':
        raise ValueError('Dense route profile must be competition_dense')

    route = data.get('route')
    if (
        not isinstance(route, dict)
        or route.get('controller') != 'polyline_lookahead'
    ):
        raise ValueError(
            'Dense route controller must be polyline_lookahead'
        )

    defaults = data.get('defaults', {})
    if not isinstance(defaults, dict):
        raise ValueError('Dense route defaults must be mapping')

    lookahead = defaults.get('lookahead_m', 1.8)
    if (
        type(lookahead) not in (int, float)
        or not math.isfinite(lookahead)
        or lookahead <= 0
    ):
        raise ValueError('Invalid dense route lookahead_m')

    rows = data.get('waypoints')
    if not isinstance(rows, list) or len(rows) < 2:
        raise ValueError('Dense route requires at least 2 waypoints')

    points = []

    for index, row in enumerate(rows, 1):
        expected_id = f'P{index:03d}'

        if not isinstance(row, dict):
            raise ValueError(
                f'Invalid dense route record at {expected_id}'
            )

        if row.get('id') != expected_id:
            raise ValueError(
                f'Expected {expected_id}, got {row.get("id")}'
            )

        latitude = row.get('latitude')
        longitude = row.get('longitude')

        if not valid_coordinates(latitude, longitude):
            raise ValueError(
                f'Invalid coordinates at {expected_id}'
            )

        points.append(
            RoutePoint(
                latitude=float(latitude),
                longitude=float(longitude),
            )
        )

    return PolylineTracker(points), float(lookahead)


def steering_to_target(fix, heading, latitude, longitude, config):
    target_bearing = bearing_deg(
        fix.latitude,
        fix.longitude,
        latitude,
        longitude,
    )

    vehicle_heading = (
        heading.heading_deg + config.heading_offset_deg
    ) % 360.0

    error_rad = math.radians(
        wrap_degrees(target_bearing - vehicle_heading)
    )

    # Vehicle convention: positive steering = LEFT.
    steering = -config.k_heading * error_rad

    return max(
        -config.max_right_steering_rad,
        min(config.max_left_steering_rad, steering),
    )


class DenseGpsControllerNode(Node):
    def __init__(self):
        super().__init__('dense_gps_controller')

        self.route_file = self.declare_parameter(
            'route_file',
            '',
            ParameterDescriptor(read_only=True),
        ).value

        defaults = vars(DenseControlConfig())
        values = {
            name: self.declare_parameter(
                name,
                value,
                ParameterDescriptor(read_only=True),
            ).value
            for name, value in defaults.items()
        }

        self.config = DenseControlConfig(**values)

        if not self.route_file:
            raise ValueError('route_file parameter is required')

        self.tracker, yaml_lookahead_m = load_dense_route(
            self.route_file
        )

        self.lookahead_m = (
            self.config.lookahead_m
            if self.config.lookahead_m > 0
            else yaml_lookahead_m
        )

        self.fix = None
        self.heading = None

        self.gps_healthy = False
        self.health_at = None

        self.mode = 'STOP'
        self.mode_at = None

        self.progress_s_m = 0.0
        self.segment_index = 0

        self.command_publisher = self.create_publisher(
            DriveCommand,
            '/cmd/gps_raw',
            1,
        )

        self.ready_publisher = self.create_publisher(
            String,
            '/mission/controller_ready',
            10,
        )

        self.progress_publisher = self.create_publisher(
            String,
            '/mission/route_progress',
            10,
        )

        self.fix_subscription = self.create_subscription(
            NavSatFix,
            '/gps/fix',
            self.on_fix,
            qos_profile_sensor_data,
        )

        self.heading_subscription = self.create_subscription(
            Float64,
            '/gps/heading',
            self.on_heading,
            qos_profile_sensor_data,
        )

        self.health_subscription = self.create_subscription(
            Bool,
            '/gps/healthy',
            self.on_health,
            qos_profile_sensor_data,
        )

        transient = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.mode_subscription = self.create_subscription(
            String,
            '/mission/control_mode',
            self.on_mode,
            transient,
        )

        self.timer = self.create_timer(
            1.0 / self.config.output_rate_hz,
            self.publish_command,
            clock=Clock(clock_type=ClockType.STEADY_TIME),
        )

        self.get_logger().info(
            f'DENSE GPS route_points={len(self.tracker.points)} '
            f'length={self.tracker.total_length_m:.1f}m '
            f'lookahead={self.lookahead_m:.2f}m'
        )

    def on_fix(self, msg):
        now = time.monotonic()

        if (
            msg.status.status in (0, 1, 2)
            and valid_coordinates(
                msg.latitude,
                msg.longitude,
            )
        ):
            self.fix = FixSample(
                msg.latitude,
                msg.longitude,
                now,
            )
        else:
            self.fix = None

    def on_heading(self, msg):
        now = time.monotonic()

        if math.isfinite(msg.data):
            self.heading = HeadingSample(
                msg.data % 360.0,
                now,
            )
        else:
            self.heading = None

    def on_health(self, msg):
        self.gps_healthy = bool(msg.data)
        self.health_at = time.monotonic()

    def on_mode(self, msg):
        self.mode = msg.data
        self.mode_at = time.monotonic()

    def publish_command(self):
        if not self.config.enabled:
            return

        now = time.monotonic()

        # Route tracking is independent of the active driving controller.
        # GPS progress must continue during GPS, LANE, OBSTACLE and STOP modes.
        tracking_ready = (
            self.gps_healthy
            and fresh(
                self.health_at,
                now,
                self.config.health_timeout_sec,
            )
            and self.fix is not None
            and fresh(
                self.fix.received_at,
                now,
                self.config.fix_timeout_sec,
            )
        )

        if not tracking_ready:
            return

        result = self.tracker.track(
            self.fix.latitude,
            self.fix.longitude,
            previous_s_m=self.progress_s_m,
            previous_segment=self.segment_index,
            lookahead_m=self.lookahead_m,
            forward_segments=self.config.search_forward_segments,
            finish_tolerance_m=self.config.finish_tolerance_m,
        )

        if (
            result.projection.distance_m
            > self.config.max_cross_track_error_m
        ):
            return

        self.progress_s_m = result.progress_s_m
        self.segment_index = max(
            self.segment_index,
            result.projection.segment_index,
        )

        # Always publish route progress while GPS tracking is healthy.
        self.progress_publisher.publish(String(
            data=json.dumps({
                'segment_index': self.segment_index,
                'progress_s_m': self.progress_s_m,
                'total_length_m': self.tracker.total_length_m,
                'cross_track_error_m': result.projection.distance_m,
                'lookahead_s_m': result.target.s_m,
                'finished': result.finished,
            }, separators=(',', ':'))
        ))

        # Only GPS mode is allowed to produce /cmd/gps.
        drive_ready = (
            self.mode == 'GPS'
            and fresh(
                self.mode_at,
                now,
                self.config.mode_timeout_sec,
            )
            and self.heading is not None
            and fresh(
                self.heading.received_at,
                now,
                self.config.heading_timeout_sec,
            )
        )

        if not drive_ready:
            return

        command = DriveCommand()
        command.header.stamp = self.get_clock().now().to_msg()
        command.header.frame_id = ''

        if result.finished:
            command.speed_mps = 0.0
            command.steering_angle_rad = 0.0
            command.emergency_stop = True
        else:
            command.speed_mps = self.config.forward_speed_mps
            command.steering_angle_rad = steering_to_target(
                self.fix,
                self.heading,
                result.target.latitude,
                result.target.longitude,
                self.config,
            )
            command.emergency_stop = False

        command.use_pwm_override = False
        command.drive_pwm_percent = 0

        self.command_publisher.publish(command)
        self.ready_publisher.publish(String(data='GPS'))


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = DenseGpsControllerNode()
        rclpy.spin(node)

    except (KeyboardInterrupt, ExternalShutdownException):
        pass

    except ValueError as error:
        rclpy.logging.get_logger(
            'dense_gps_controller'
        ).error(str(error))
        return 1

    finally:
        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

    return 0


if __name__ == '__main__':
    raise SystemExit(main())
