"""RAMP mission drive override: preserve lane steering and override only drive PWM."""
import math
import time

import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile

from fma_interfaces.msg import DriveCommand, MissionState


class RampControllerNode(Node):
    """Publish /cmd/mission only while the RAMP mission is active.

    The lane command remains the steering authority. This node copies its speed
    direction, steering angle and emergency flag, then replaces only drive PWM.
    ramp_pwm=0 intentionally disables the override until a measured value is set.
    """

    def __init__(self):
        super().__init__('ramp_controller')
        defaults = {
            'mission_topic': '/mission/current',
            'lane_topic': '/cmd/lane',
            'output_topic': '/cmd/mission',
            'ramp_pwm': 0,
            'lane_timeout_sec': 0.5,
            'output_rate_hz': 20.0,
        }
        self.config = {
            name: self.declare_parameter(
                name, value, ParameterDescriptor(read_only=True)).value
            for name, value in defaults.items()
        }
        pwm = self.config['ramp_pwm']
        if type(pwm) is not int or not 0 <= pwm <= DriveCommand.DRIVE_PWM_MAX:
            raise ValueError(f'ramp_pwm must be integer 0..{DriveCommand.DRIVE_PWM_MAX}')
        for name in ('lane_timeout_sec', 'output_rate_hz'):
            value = self.config[name]
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f'{name} must be finite and positive')

        self.ramp_active = False
        self.latest_lane = None
        self.latest_lane_at = None
        self.last_warning = -math.inf

        self.publisher = self.create_publisher(DriveCommand, self.config['output_topic'], 1)
        mission_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.mission_subscription = self.create_subscription(
            MissionState, self.config['mission_topic'], self.on_mission, mission_qos)
        self.lane_subscription = self.create_subscription(
            DriveCommand, self.config['lane_topic'], self.on_lane, 1)
        self.timer = self.create_timer(
            1.0 / self.config['output_rate_hz'], self.tick,
            clock=Clock(clock_type=ClockType.STEADY_TIME))

        if pwm == 0:
            self.get_logger().warning(
                'ramp_pwm=0: RAMP override is disabled until a measured PWM is configured.')

    def on_mission(self, msg):
        self.ramp_active = (
            msg.current_mission == MissionState.RAMP and msg.active and not msg.completed)

    def on_lane(self, msg):
        if (not math.isfinite(msg.speed_mps)
                or not math.isfinite(msg.steering_angle_rad)):
            return
        self.latest_lane = (
            float(msg.speed_mps), float(msg.steering_angle_rad), bool(msg.emergency_stop))
        self.latest_lane_at = time.monotonic()

    def warn_throttled(self, message, now):
        if now - self.last_warning >= 5.0:
            self.get_logger().warning(message)
            self.last_warning = now

    def tick(self):
        if not self.ramp_active:
            return
        now = time.monotonic()
        if self.config['ramp_pwm'] == 0:
            self.warn_throttled('RAMP active but ramp_pwm=0; no mission command published.', now)
            return
        if (self.latest_lane is None or self.latest_lane_at is None
                or now - self.latest_lane_at >= self.config['lane_timeout_sec']):
            self.warn_throttled('RAMP active but lane command is stale; no mission command published.', now)
            return

        speed_mps, steering_angle_rad, emergency_stop = self.latest_lane
        output = DriveCommand()
        output.header.stamp = self.get_clock().now().to_msg()
        output.header.frame_id = ''
        output.speed_mps = speed_mps
        output.steering_angle_rad = steering_angle_rad
        output.emergency_stop = emergency_stop
        output.pwm_control = True
        output.drive_pwm = self.config['ramp_pwm']
        self.publisher.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = RampControllerNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except ValueError as error:
        rclpy.logging.get_logger('ramp_controller').error(f'Invalid configuration: {error}')
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
