"""Bridge for the current polling STM32 protocol, without numeric speed control.

All parameters are startup-only. receive_only suppresses EVERY write, including
shutdown/error STOPs. TX spacing reduces polling-UART overruns but cannot provide
an acknowledgement or delivery guarantee with this firmware.
"""

import math
import re
import time

import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
import serial

from fma_interfaces.msg import DriveCommand, VehicleFeedback


TELEMETRY = re.compile(
    r'ENC=([+-]?[0-9]+) SPEED=([+-]?[0-9]+)mm/s STEER=([0-9]+) DRIVE=([0-9]+)')


def parse_telemetry(line):
    """Return wire values converted to ROS units; reject invalid/range errors."""
    match = TELEMETRY.fullmatch(line)
    if match is None:
        raise ValueError('invalid telemetry syntax')
    encoder, speed, steering, drive = map(int, match.groups())
    if not -(2**31) <= encoder < 2**31 or not -(2**31) <= speed < 2**31:
        raise ValueError('firmware int32 value out of range')
    if not 0 <= steering <= 65535 or drive not in (0, 1, 2):
        raise ValueError('invalid steering ADC or drive state')
    return encoder, speed / 1000.0, steering, drive


def speed_to_drive(speed, deadband):
    """Magnitude is ignored: 0.2 and 1.0 m/s both request fixed 30% forward."""
    if not math.isfinite(speed) or not math.isfinite(deadband) or deadband < 0:
        raise ValueError('invalid speed or deadband')
    return b'W' if speed > deadband else b'S' if speed < -deadband else b'X'


def steering_to_adc(angle, enabled, right_angle, left_angle,
                    right_adc, center_adc, left_adc, tolerance):
    """Use measured REP-103 endpoints only; otherwise allow center only."""
    if not math.isfinite(angle):
        raise ValueError('non-finite steering request')
    if enabled and not (math.isfinite(right_angle) and right_angle < 0
                        and math.isfinite(left_angle) and left_angle > 0):
        raise ValueError('steering calibration endpoints are unconfigured/invalid')
    if abs(angle) <= tolerance:
        return center_adc
    if not enabled:
        raise ValueError('nonzero steering requires measured calibration; stopping')
    if angle < 0:
        return round(center_adc + min(angle / right_angle, 1.0) * (right_adc - center_adc))
    return round(center_adc + min(angle / left_angle, 1.0) * (left_adc - center_adc))


class STM32BridgeNode(Node):
    """Default mutually-exclusive callback group owns all serial access."""

    def __init__(self):
        super().__init__('stm32_bridge_node')
        defaults = {
            'port': '/dev/ttyACM0', 'baudrate': 115200,
            'command_topic': '/cmd/final', 'feedback_topic': '/vehicle/feedback',
            'drive_refresh_hz': 5.0, 'steering_refresh_hz': 10.0,
            'ros_command_timeout_sec': 0.5, 'speed_deadband_mps': 0.01,
            'steering_center_adc': 2182, 'steering_right_adc': 50,
            'steering_left_adc': 4040, 'steering_calibration_enabled': False,
            'steering_right_angle_rad': float('nan'),
            'steering_left_angle_rad': float('nan'),
            'steering_zero_tolerance_rad': 0.001, 'receive_only': False,
        }
        self.config = {}
        for name, value in defaults.items():
            self.config[name] = self.declare_parameter(
                name, value, ParameterDescriptor(read_only=True)).value
        self.serial = None
        self.connected = False
        self.buffer = bytearray()
        self.discard_line = False
        self.last_command = None
        self.drive = b'X'
        self.steering = None
        self.safe_stop = True
        self.next_drive = self.next_steering = self.next_tx = 0.0
        self.was_stopped = False
        self.warning_times = {}
        self.publisher = self.create_publisher(
            VehicleFeedback, self.config['feedback_topic'], 10)
        self.subscription = self.create_subscription(
            DriveCommand, self.config['command_topic'], self.on_command, 1)
        self.validate_config()
        self.drive_period = 1.0 / self.config['drive_refresh_hz']
        self.steering_period = 1.0 / self.config['steering_refresh_hz']
        self.get_logger().warning(
            'Speed magnitude is ignored: 0.2 and 1.0 m/s both mean fixed 30% '
            'forward above deadband. No numeric speed control or latched emergency protocol.')
        try:
            self.serial = serial.Serial(
                port=self.config['port'], baudrate=self.config['baudrate'],
                bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE, timeout=0, write_timeout=0.02,
                xonxoff=False, rtscts=False, dsrdtr=False, exclusive=True)
            self.connected = True
            self.get_logger().info(
                f"Serial opened: {self.config['port']}; receive_only={self.config['receive_only']}")
        except (serial.SerialException, OSError, ValueError) as error:
            self.get_logger().error(f'Serial open failed; commands unavailable: {error}')
        # Steady clock keeps safety checks running even when ROS /clock pauses.
        self.io_timer = self.create_timer(
            0.005, self.poll, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def validate_config(self):
        c = self.config
        for name in ('drive_refresh_hz', 'steering_refresh_hz'):
            if not math.isfinite(c[name]) or not 2.0 <= c[name] <= 20.0:
                raise ValueError(f'{name} must be 2..20 Hz for 700 ms firmware timeout')
        for name in ('speed_deadband_mps', 'steering_zero_tolerance_rad'):
            if not math.isfinite(c[name]) or c[name] < 0:
                raise ValueError(f'{name} must be finite and nonnegative')
        if not math.isfinite(c['ros_command_timeout_sec']) or c['ros_command_timeout_sec'] <= 0:
            raise ValueError('ros_command_timeout_sec must be finite and positive')
        if not 50 <= c['steering_right_adc'] < c['steering_center_adc'] < c['steering_left_adc'] <= 4040:
            raise ValueError('ADC endpoints must be ordered within firmware range 50..4040')
        if c['baudrate'] <= 0:
            raise ValueError('baudrate must be positive')
        # Invalid enabled calibration is handled as STOP for every received command.

    def warn(self, key, message):
        now = time.monotonic()
        if now - self.warning_times.get(key, -math.inf) >= 5.0:
            self.get_logger().warning(message)
            self.warning_times[key] = now

    def on_command(self, msg):
        self.last_command = time.monotonic()
        self.safe_stop = True
        self.drive, self.steering = b'X', None
        if not msg.emergency_stop:
            c = self.config
            try:
                drive = speed_to_drive(msg.speed_mps, c['speed_deadband_mps'])
                adc = steering_to_adc(
                    msg.steering_angle_rad, c['steering_calibration_enabled'],
                    c['steering_right_angle_rad'], c['steering_left_angle_rad'],
                    c['steering_right_adc'], c['steering_center_adc'],
                    c['steering_left_adc'], c['steering_zero_tolerance_rad'])
                self.drive, self.steering, self.safe_stop = drive, adc, False
            except ValueError as error:
                self.warn('command', str(error))
        if self.safe_stop:
            self.next_drive = 0.0
        self.transmit(time.monotonic())

    def transmit(self, now):
        if self.config['receive_only'] or not self.connected:
            return
        stale = (self.last_command is None or
                 now - self.last_command >= self.config['ros_command_timeout_sec'])
        stopped = self.safe_stop or stale
        if stopped and not self.was_stopped:
            self.next_drive = 0.0
        self.was_stopped = stopped
        if now < self.next_tx:
            return
        if now >= self.next_drive:
            self.write(b'X' if stopped else self.drive)
            self.next_drive = time.monotonic() + self.drive_period
        elif not stopped and now >= self.next_steering:
            self.write(f'T{self.steering:04d}'.encode('ascii'))
            self.next_steering = time.monotonic() + self.steering_period

    def write(self, packet):
        try:
            if self.serial.write(packet) != len(packet):
                raise serial.SerialException('partial serial write')
            # Allow firmware blocking reply/direction-change delay to finish.
            self.next_tx = time.monotonic() + 0.02
        except (serial.SerialException, OSError) as error:
            self.serial_failed(error)

    def serial_failed(self, error):
        self.connected = False
        self.get_logger().error(f'Serial connection invalid; no reconnect: {error}')
        self.close_serial()

    def poll(self):
        if not self.connected:
            return
        self.transmit(time.monotonic())
        if not self.connected:
            return
        try:
            data = self.serial.read(min(self.serial.in_waiting, 4096))
        except (serial.SerialException, OSError) as error:
            self.serial_failed(error)
            return
        for value in data:
            if self.discard_line:
                if value == 10:
                    self.discard_line = False
                continue
            self.buffer.append(value)
            if value == 10:
                line = bytes(self.buffer)
                self.buffer.clear()
                self.process_line(line)
            elif len(self.buffer) > 256:
                self.buffer.clear()
                self.discard_line = True
                self.warn('rx', 'Discarding oversized serial line')

    def process_line(self, raw):
        try:
            line = raw.decode('ascii')
            if not line.endswith('\r\n'):
                raise ValueError('serial line missing CRLF')
            line = line[:-2]
            if not line.startswith('ENC'):
                self.get_logger().debug(f'STM32 status: {line}')
                return
            encoder, speed, steering, drive = parse_telemetry(line)
        except (ValueError, UnicodeError) as error:
            self.warn('rx', f'Invalid telemetry discarded: {error}')
            return
        msg = VehicleFeedback()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = ''
        msg.encoder_count, msg.speed_mps = encoder, speed
        msg.steering_adc, msg.drive_state = steering, drive
        self.publisher.publish(msg)

    def close_serial(self):
        port = self.serial
        self.connected = False
        if port is None:
            return
        try:
            if port.is_open and not self.config['receive_only']:
                # A first X may only abort an incomplete T packet in firmware.
                for _ in range(2):
                    time.sleep(0.02)
                    try:
                        port.write(b'X')
                    except (serial.SerialException, OSError):
                        pass
        finally:
            try:
                port.close()
            except (serial.SerialException, OSError):
                pass
            self.serial = None

    def destroy_node(self):
        self.io_timer.cancel()
        self.close_serial()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = STM32BridgeNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except ValueError as error:
        rclpy.logging.get_logger('stm32_bridge_node').error(f'Invalid configuration: {error}')
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
