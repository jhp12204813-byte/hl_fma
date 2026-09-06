"""Convert high-level drive requests to vehicle commands; no hardware access.

Parameters are configured at startup. Angle endpoints remain unconfigured until
measured on the vehicle. Speed magnitude cannot request numeric speed control.
"""

from dataclasses import dataclass
import math
import time

import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from fma_interfaces.msg import DriveCommand, VehicleCommand


@dataclass(frozen=True)
class ConversionConfig:
    speed_deadband_mps: float = 0.01
    steering_calibration_enabled: bool = False
    steering_right_adc: int = 150
    steering_center_adc: int = 2132
    steering_left_adc: int = 3950
    steering_right_angle_rad: float = math.nan
    steering_left_angle_rad: float = math.nan
    steering_zero_tolerance_rad: float = 0.001


def speed_to_drive_state(speed_mps, deadband=0.01):
    """Use sign only: +0.2 and +1.0 both request DRIVE_FORWARD."""
    if not math.isfinite(deadband) or deadband < 0:
        raise ValueError('speed deadband must be finite and nonnegative')
    if not math.isfinite(speed_mps):
        raise ValueError('speed must be finite')
    if speed_mps > deadband:
        return VehicleCommand.DRIVE_FORWARD
    if speed_mps < -deadband:
        return VehicleCommand.DRIVE_REVERSE
    return VehicleCommand.DRIVE_STOP


def validate_config(config):
    c = config
    speed_to_drive_state(0.0, c.speed_deadband_mps)
    if not math.isfinite(c.steering_zero_tolerance_rad) or c.steering_zero_tolerance_rad < 0:
        raise ValueError('steering tolerance must be finite and nonnegative')
    endpoints = (c.steering_right_adc, c.steering_center_adc, c.steering_left_adc)
    if any(type(value) is not int for value in endpoints):
        raise ValueError('ADC endpoints must be integers')
    if not 150 <= endpoints[0] < endpoints[1] < endpoints[2] <= 3950:
        raise ValueError('ADC endpoints must be ordered within 150..3950')


def steering_angle_to_adc(angle_rad, config=ConversionConfig()):
    """Map measured REP-103 endpoints piecewise, with endpoint clamping."""
    validate_config(config)
    c = config
    if not math.isfinite(angle_rad):
        raise ValueError('steering angle must be finite')
    if c.steering_calibration_enabled:
        if not (math.isfinite(c.steering_right_angle_rad)
                and c.steering_right_angle_rad < 0
                and math.isfinite(c.steering_left_angle_rad)
                and c.steering_left_angle_rad > 0):
            raise ValueError('enabled steering calibration requires measured finite endpoints')
    if abs(angle_rad) <= c.steering_zero_tolerance_rad:
        return c.steering_center_adc
    if not c.steering_calibration_enabled:
        raise ValueError('nonzero steering requires calibration; forcing safe STOP')
    if angle_rad < 0:
        fraction = min(angle_rad / c.steering_right_angle_rad, 1.0)
        target = c.steering_center_adc + fraction * (c.steering_right_adc - c.steering_center_adc)
    else:
        fraction = min(angle_rad / c.steering_left_angle_rad, 1.0)
        target = c.steering_center_adc + fraction * (c.steering_left_adc - c.steering_center_adc)
    return max(150, min(3950, round(target)))


class VehicleControllerNode(Node):
    def __init__(self):
        super().__init__('vehicle_controller')
        defaults = vars(ConversionConfig()).copy()
        defaults.update(command_topic='/cmd/final', vehicle_command_topic='/vehicle/command',
                        command_timeout_sec=0.5)
        values = {
            name: self.declare_parameter(
                name, value, ParameterDescriptor(read_only=True)).value
            for name, value in defaults.items()
        }
        self.config = ConversionConfig(**{
            name: values[name] for name in vars(ConversionConfig())})
        validate_config(self.config)
        self.command_timeout = values['command_timeout_sec']
        if not math.isfinite(self.command_timeout) or self.command_timeout <= 0:
            raise ValueError('command_timeout_sec must be finite and positive')
        self.last_valid_command = None
        self.last_warning = -math.inf
        self.publisher = self.create_publisher(
            VehicleCommand, values['vehicle_command_topic'], 1)
        self.subscription = self.create_subscription(
            DriveCommand, values['command_topic'], self.on_command, 1)
        # Fixed 20 Hz safety checks. Monotonic time and a steady timer work even
        # when simulated ROS time pauses. Only stale/startup STOP is repeated.
        self.timeout_timer = self.create_timer(
            0.05, self.check_timeout, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().warning(
            'Speed magnitude is ignored: +0.2 and +1.0 m/s both request FORWARD. '
            'Numeric speed control is unavailable.')

    def publish_command(self, drive_state, steering_adc, emergency_stop):
        output = VehicleCommand()
        output.header.stamp = self.get_clock().now().to_msg()
        output.header.frame_id = ''
        output.drive_state = drive_state
        output.steering_adc = steering_adc
        output.emergency_stop = emergency_stop
        self.publisher.publish(output)

    def publish_safe_stop(self):
        # emergency_stop also forces a low-level fail-safe STOP for invalid or
        # stale input. It does NOT claim the cause was /cmd/emergency; there is
        # currently no separate fault/status interface to express that cause.
        self.publish_command(VehicleCommand.DRIVE_STOP,
                             self.config.steering_center_adc, True)

    def on_command(self, command):
        if command.emergency_stop:
            self.publish_safe_stop()
            return
        try:
            state = speed_to_drive_state(command.speed_mps, self.config.speed_deadband_mps)
            adc = steering_angle_to_adc(command.steering_angle_rad, self.config)
        except ValueError as error:
            now = time.monotonic()
            if now - self.last_warning >= 5.0:
                self.get_logger().warning(str(error))
                self.last_warning = now
            self.publish_safe_stop()
            return
        self.last_valid_command = time.monotonic()
        self.publish_command(state, adc, False)

    def check_timeout(self):
        if (self.last_valid_command is None
                or time.monotonic() - self.last_valid_command >= self.command_timeout):
            self.publish_safe_stop()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = VehicleControllerNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except ValueError as error:
        rclpy.logging.get_logger('vehicle_controller').error(f'Invalid configuration: {error}')
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
