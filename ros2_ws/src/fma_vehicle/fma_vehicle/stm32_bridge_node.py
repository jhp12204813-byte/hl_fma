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

from fma_interfaces.msg import VehicleCommand, VehicleFeedback


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


DRIVE_PACKETS = {
    VehicleCommand.DRIVE_STOP: b'X',
    VehicleCommand.DRIVE_FORWARD: b'W',
    VehicleCommand.DRIVE_REVERSE: b'S',
}
STEERING_MIN_ADC = 150
STEERING_MAX_ADC = 3950


class STM32BridgeNode(Node):
    """Default mutually-exclusive callback group owns all serial access."""

    def __init__(self):
        super().__init__('stm32_bridge_node')
        defaults = {
            'port': '/dev/ttyACM0', 'baudrate': 115200,
            'vehicle_command_topic': '/vehicle/command',
            'feedback_topic': '/vehicle/feedback',
            'drive_refresh_hz': 5.0, 'steering_refresh_hz': 10.0,
            'vehicle_command_timeout_sec': 0.5, 'receive_only': False,
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
            VehicleCommand, self.config['vehicle_command_topic'], self.on_command, 1)
        self.validate_config()
        self.drive_period = 1.0 / self.config['drive_refresh_hz']
        self.steering_period = 1.0 / self.config['steering_refresh_hz']
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
        timeout = c['vehicle_command_timeout_sec']
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError('vehicle_command_timeout_sec must be finite and positive')
        if c['baudrate'] <= 0:
            raise ValueError('baudrate must be positive')

    def warn(self, key, message):
        now = time.monotonic()
        if now - self.warning_times.get(key, -math.inf) >= 5.0:
            self.get_logger().warning(message)
            self.warning_times[key] = now

    def on_command(self, msg):
        self.safe_stop = True
        self.drive, self.steering = b'X', None
        # Emergency overrides even invalid fields; never retain a motion target.
        if not msg.emergency_stop:
            if msg.drive_state not in DRIVE_PACKETS:
                self.warn('command', 'Invalid drive state; forcing STOP')
            elif not STEERING_MIN_ADC <= msg.steering_adc <= STEERING_MAX_ADC:
                self.warn('command', 'Steering ADC outside 150..3950; forcing STOP')
            else:
                self.last_command = time.monotonic()
                self.drive = DRIVE_PACKETS[msg.drive_state]
                self.safe_stop = self.drive == b'X'
                if not self.safe_stop:
                    self.steering = msg.steering_adc
        if self.safe_stop:
            self.next_drive = 0.0
        self.transmit(time.monotonic())

    def transmit(self, now):
        if self.config['receive_only'] or not self.connected:
            return
        stale = (self.last_command is None or
                 now - self.last_command >= self.config['vehicle_command_timeout_sec'])
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
