"""Low-speed waypoint controller using F9P position and moving-base heading."""
from dataclasses import dataclass
import json
import math
import time

from fma_interfaces.msg import DriveCommand
import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool, Float64, String


@dataclass(frozen=True)
class ControlConfig:
    enabled: bool = False
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

    def __post_init__(self):
        if not isinstance(self.enabled, bool):
            raise ValueError('enabled must be boolean')
        numeric = [
            self.k_heading, self.forward_speed_mps,
            self.fix_timeout_sec, self.heading_timeout_sec,
            self.health_timeout_sec, self.mode_timeout_sec,
            self.output_rate_hz, self.heading_offset_deg,
            self.max_left_steering_rad, self.max_right_steering_rad,
        ]
        if not all(math.isfinite(v) for v in numeric):
            raise ValueError('GPS control parameters must be finite')
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
        ) <= 0:
            raise ValueError('GPS control speed/time/rate/limits must be positive')


@dataclass(frozen=True)
class FixSample:
    latitude: float
    longitude: float
    received_at: float


@dataclass(frozen=True)
class HeadingSample:
    heading_deg: float
    received_at: float


@dataclass(frozen=True)
class TargetWaypoint:
    number: int
    id: str
    latitude: float
    longitude: float
    activation_radius_m: float
    hard_point: bool


def valid_coordinates(latitude, longitude):
    return (
        isinstance(latitude, (int, float))
        and not isinstance(latitude, bool)
        and isinstance(longitude, (int, float))
        and not isinstance(longitude, bool)
        and math.isfinite(latitude)
        and math.isfinite(longitude)
        and -90.0 <= latitude <= 90.0
        and -180.0 <= longitude <= 180.0
    )


def wrap_degrees(angle):
    return (angle + 180.0) % 360.0 - 180.0


def bearing_deg(lat1, lon1, lat2, lon2):
    """Initial great-circle bearing: north=0, east=90 degrees."""
    if not valid_coordinates(lat1, lon1) or not valid_coordinates(lat2, lon2):
        raise ValueError('Invalid coordinates')
    p1 = math.radians(lat1)
    p2 = math.radians(lat2)
    dl = math.radians(lon2 - lon1)
    y = math.sin(dl) * math.cos(p2)
    x = math.cos(p1) * math.sin(p2) - math.sin(p1) * math.cos(p2) * math.cos(dl)
    return math.degrees(math.atan2(y, x)) % 360.0


def parse_target(text):
    try:
        data = json.loads(text)
        number = data['number']
        waypoint_id = data['id']
        latitude = data['latitude']
        longitude = data['longitude']
        radius = data['activation_radius_m']
        hard_point = data['hard_point']
    except (json.JSONDecodeError, KeyError, TypeError):
        return None

    if (
        type(number) is not int
        or number < 1
        or not isinstance(waypoint_id, str)
        or not waypoint_id
        or not valid_coordinates(latitude, longitude)
        or type(radius) not in (int, float)
        or not math.isfinite(radius)
        or radius <= 0
        or type(hard_point) is not bool
    ):
        return None

    return TargetWaypoint(
        number, waypoint_id, float(latitude), float(longitude),
        float(radius), hard_point
    )


def gps_command(fix, heading, target, config):
    target_bearing = bearing_deg(
        fix.latitude, fix.longitude, target.latitude, target.longitude)

    vehicle_heading = (heading.heading_deg + config.heading_offset_deg) % 360.0

    # Positive navigation error means target is clockwise/right of the vehicle.
    heading_error_rad = math.radians(
        wrap_degrees(target_bearing - vehicle_heading))

    # Vehicle convention: positive steering = LEFT, so sign is inverted.
    steering = -config.k_heading * heading_error_rad
    steering = max(
        -config.max_right_steering_rad,
        min(config.max_left_steering_rad, steering))

    return config.forward_speed_mps, steering


def fresh(received_at, now, timeout):
    return (
        received_at is not None
        and math.isfinite(received_at)
        and math.isfinite(now)
        and 0.0 <= now - received_at < timeout
    )


class GpsControllerNode(Node):
    def __init__(self):
        super().__init__('gps_controller')

        defaults = vars(ControlConfig())
        values = {
            name: self.declare_parameter(
                name, value, ParameterDescriptor(read_only=True)).value
            for name, value in defaults.items()
        }
        self.config = ControlConfig(**values)

        self.fix = None
        self.heading = None
        self.gps_healthy = False
        self.health_at = None
        self.target = None
        self.mode = 'STOP'
        self.mode_at = None

        self.command_publisher = self.create_publisher(
            DriveCommand, '/cmd/gps', 1)
        self.ready_publisher = self.create_publisher(
            String, '/mission/controller_ready', 10)

        self.fix_subscription = self.create_subscription(
            NavSatFix, '/gps/fix', self.on_fix, qos_profile_sensor_data)
        self.heading_subscription = self.create_subscription(
            Float64, '/gps/heading', self.on_heading, qos_profile_sensor_data)
        self.health_subscription = self.create_subscription(
            Bool, '/gps/healthy', self.on_health, qos_profile_sensor_data)

        transient = QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.target_subscription = self.create_subscription(
            String, '/mission/target_waypoint', self.on_target, transient)
        self.mode_subscription = self.create_subscription(
            String, '/mission/control_mode', self.on_mode, transient)

        self.timer = self.create_timer(
            1.0 / self.config.output_rate_hz,
            self.publish_command,
            clock=Clock(clock_type=ClockType.STEADY_TIME))

    def on_fix(self, msg):
        now = time.monotonic()
        if (
            msg.status.status in (0, 1, 2)
            and valid_coordinates(msg.latitude, msg.longitude)
        ):
            self.fix = FixSample(msg.latitude, msg.longitude, now)
        else:
            self.fix = None

    def on_heading(self, msg):
        now = time.monotonic()
        if math.isfinite(msg.data):
            self.heading = HeadingSample(msg.data % 360.0, now)
        else:
            self.heading = None

    def on_health(self, msg):
        self.gps_healthy = bool(msg.data)
        self.health_at = time.monotonic()

    def on_target(self, msg):
        self.target = parse_target(msg.data)

    def on_mode(self, msg):
        self.mode = msg.data
        self.mode_at = time.monotonic()

    def publish_command(self):
        if not self.config.enabled:
            return

        now = time.monotonic()

        ready = (
            self.mode == 'GPS'
            and fresh(self.mode_at, now, self.config.mode_timeout_sec)
            and self.gps_healthy
            and fresh(self.health_at, now, self.config.health_timeout_sec)
            and self.fix is not None
            and fresh(self.fix.received_at, now, self.config.fix_timeout_sec)
            and self.heading is not None
            and fresh(
                self.heading.received_at, now,
                self.config.heading_timeout_sec)
            and self.target is not None
        )

        if not ready:
            return

        speed, steering = gps_command(
            self.fix, self.heading, self.target, self.config)

        command = DriveCommand()
        command.header.stamp = self.get_clock().now().to_msg()
        command.header.frame_id = ''
        command.speed_mps = speed
        command.steering_angle_rad = steering
        command.emergency_stop = False
        command.pwm_control = False
        command.drive_pwm = 0

        # Motion candidate first, readiness second.
        self.command_publisher.publish(command)
        self.ready_publisher.publish(String(data='GPS'))


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = GpsControllerNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except ValueError as error:
        rclpy.logging.get_logger('gps_controller').error(str(error))
        return 1
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0
