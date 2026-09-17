"""ROS2 node for mission-conditioned dummy avoidance."""

import math
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile

from fma_interfaces.msg import (
    DriveCommand,
    MissionState,
    ObstacleArray,
    VehicleFeedback,
)

from fma_mission.dummy_maneuver import (
    DummyManeuverController,
    ManeuverConfig,
    ManeuverPhase,
    ObstacleSample,
)


class DummyManeuverNode(Node):
    def __init__(self):
        super().__init__('dummy_maneuver')

        # Safety: dry-run by default.
        self.declare_parameter('enable_drive', False)

        self.declare_parameter('trigger_distance_m', 2.0)
        self.declare_parameter('child_trigger_distance_m', 2.0)
        self.declare_parameter('child_clear_distance_m', 3.0)
        self.declare_parameter('child_clear_hold_sec', 2.0)
        self.declare_parameter('clear_frames', 5)
        self.declare_parameter('min_confidence', 0.5)

        self.declare_parameter('confirm_frames', 3)
        self.declare_parameter('dropout_frames', 2)

        self.declare_parameter('drive_pwm', 120)
        self.declare_parameter('forward_speed_mps', 0.2)
        self.declare_parameter('steering_rad', 0.20)

        self.declare_parameter('stop_hold_sec', 3.0)
        self.declare_parameter('avoid_out_distance_m', 1.00)
        self.declare_parameter('avoid_out_hold_sec', 1.0)
        self.declare_parameter('pass_straight_distance_m', 1.30)
        self.declare_parameter('avoid_return_distance_m', 1.00)

        self.declare_parameter('feedback_timeout_sec', 0.5)
        self.declare_parameter('stopped_speed_threshold_mps', 0.05)
        self.declare_parameter('control_rate_hz', 20.0)

        self.enable_drive = bool(
            self.get_parameter('enable_drive').value
        )
        self.trigger_distance_m = float(
            self.get_parameter('trigger_distance_m').value
        )
        self.min_confidence = float(
            self.get_parameter('min_confidence').value
        )
        self.confirm_frames = int(
            self.get_parameter('confirm_frames').value
        )
        self.dropout_frames = int(
            self.get_parameter('dropout_frames').value
        )

        self.drive_pwm = int(
            self.get_parameter('drive_pwm').value
        )
        self.forward_speed_mps = float(
            self.get_parameter('forward_speed_mps').value
        )
        self.feedback_timeout_sec = float(
            self.get_parameter('feedback_timeout_sec').value
        )
        self.stopped_speed_threshold_mps = float(
            self.get_parameter('stopped_speed_threshold_mps').value
        )
        control_rate_hz = float(
            self.get_parameter('control_rate_hz').value
        )

        config = ManeuverConfig(
            trigger_distance_m=self.trigger_distance_m,
            child_trigger_distance_m=float(
                self.get_parameter('child_trigger_distance_m').value
            ),
            child_clear_distance_m=float(
                self.get_parameter('child_clear_distance_m').value
            ),
            child_clear_hold_sec=float(
                self.get_parameter('child_clear_hold_sec').value
            ),
            clear_frames=int(
                self.get_parameter('clear_frames').value
            ),
            min_confidence=self.min_confidence,
            steering_rad=float(
                self.get_parameter('steering_rad').value
            ),
            drive_pwm=self.drive_pwm,
            stop_hold_sec=float(
                self.get_parameter('stop_hold_sec').value
            ),
            avoid_out_distance_m=float(
                self.get_parameter(
                    'avoid_out_distance_m'
                ).value
            ),
            avoid_out_hold_sec=float(
                self.get_parameter(
                    'avoid_out_hold_sec'
                ).value
            ),
            pass_straight_distance_m=float(
                self.get_parameter(
                    'pass_straight_distance_m'
                ).value
            ),
            avoid_return_distance_m=float(
                self.get_parameter(
                    'avoid_return_distance_m'
                ).value
            ),
        )

        self.controller = DummyManeuverController(config)

        self.current_mission = MissionState.NORMAL_DRIVE
        self.mission_active = False

        self.encoder_count = None
        self.vehicle_speed_mps = None
        self.feedback_rx_time = None

        self.confirmed_obstacle = None
        self.hit_count = 0
        self.miss_count = 0

        self.last_phase = ManeuverPhase.IDLE
        self.was_commanding = False

        mission_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.create_subscription(
            MissionState,
            '/mission/current',
            self.on_mission,
            mission_qos,
        )

        self.create_subscription(
            ObstacleArray,
            '/perception/obstacles',
            self.on_obstacles,
            10,
        )

        self.create_subscription(
            VehicleFeedback,
            '/vehicle/feedback',
            self.on_feedback,
            10,
        )

        self.command_pub = self.create_publisher(
            DriveCommand,
            '/cmd/mission',
            10,
        )

        self.create_timer(
            1.0 / control_rate_hz,
            self.on_timer,
        )

        self.get_logger().info(
            'Dummy maneuver ready: '
            f'enable_drive={self.enable_drive}'
        )

    def reset_perception_filter(self):
        self.confirmed_obstacle = None
        self.hit_count = 0
        self.miss_count = 0

    def on_mission(self, msg):
        changed = (
            msg.current_mission != self.current_mission
            or msg.active != self.mission_active
        )

        self.current_mission = msg.current_mission
        self.mission_active = msg.active

        if changed:
            self.controller.reset()
            self.reset_perception_filter()

    def on_feedback(self, msg):
        self.encoder_count = int(msg.encoder_count)
        self.vehicle_speed_mps = float(msg.speed_mps)
        self.feedback_rx_time = time.monotonic()

    def nearest_valid_obstacle(self, msg):
        valid = []

        for item in msg.obstacles:
            if not item.in_path:
                continue

            if not math.isfinite(item.distance_m):
                continue

            if not math.isfinite(item.y_m):
                continue

            if not math.isfinite(item.confidence):
                continue

            if item.distance_m <= 0.0:
                continue

            if item.distance_m > self.trigger_distance_m:
                continue

            if item.confidence < self.min_confidence:
                continue

            valid.append(item)

        if not valid:
            return None

        item = min(
            valid,
            key=lambda obs: obs.distance_m,
        )

        return ObstacleSample(
            distance_m=float(item.distance_m),
            y_m=float(item.y_m),
            confidence=float(item.confidence),
            in_path=True,
        )

    def on_obstacles(self, msg):
        sample = self.nearest_valid_obstacle(msg)

        if sample is not None:
            self.miss_count = 0
            self.hit_count += 1

            if self.hit_count >= self.confirm_frames:
                self.confirmed_obstacle = sample

            return

        self.hit_count = 0

        if self.confirmed_obstacle is None:
            return

        self.miss_count += 1

        if self.miss_count > self.dropout_frames:
            self.confirmed_obstacle = None
            self.miss_count = 0

    def feedback_is_fresh(self):
        if self.feedback_rx_time is None:
            return False

        return (
            time.monotonic() - self.feedback_rx_time
            < self.feedback_timeout_sec
        )

    def publish_stop(self):
        msg = DriveCommand()
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.speed_mps = 0.0
        msg.steering_angle_rad = 0.0
        msg.emergency_stop = False

        # STOP does not require raw mission PWM permission.
        msg.pwm_control = False
        msg.drive_pwm = 0

        self.command_pub.publish(msg)

    def publish_move(self, steering_rad, drive_pwm):
        msg = DriveCommand()
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.speed_mps = self.forward_speed_mps
        msg.steering_angle_rad = float(steering_rad)
        msg.emergency_stop = False

        msg.pwm_control = True
        msg.drive_pwm = int(drive_pwm)

        self.command_pub.publish(msg)

    def on_timer(self):
        # Never allow autonomous movement without fresh STM32 feedback.
        if self.enable_drive and not self.feedback_is_fresh():
            if self.was_commanding:
                self.publish_stop()

            self.was_commanding = False
            return

        vehicle_stopped = (
            self.feedback_is_fresh()
            and self.vehicle_speed_mps is not None
            and abs(self.vehicle_speed_mps)
            <= self.stopped_speed_threshold_mps
        )

        output = self.controller.update(
            mission=self.current_mission,
            mission_active=self.mission_active,
            obstacle=self.confirmed_obstacle,
            now_sec=time.monotonic(),
            encoder_count=self.encoder_count,
            vehicle_stopped=vehicle_stopped,
        )

        if output.phase is not self.last_phase:
            side = (
                output.pass_side.name
                if output.pass_side is not None
                else 'NONE'
            )

            self.get_logger().info(
                f'MANEUVER_PHASE={output.phase.value} '
                f'pass_side={side} '
                f'drive_enabled={self.enable_drive}'
            )

            self.last_phase = output.phase

        # Dry-run stops here.
        if not self.enable_drive:
            return

        if output.command_active:
            if output.stop:
                self.publish_stop()
            else:
                self.publish_move(
                    output.steering_rad,
                    output.drive_pwm,
                )

            self.was_commanding = True
            return

        # One STOP when releasing mission control.
        # Arbiter then lets the mission candidate become stale
        # and normal driving can regain control.
        if self.was_commanding:
            self.publish_stop()

        self.was_commanding = False


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = DummyManeuverNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

    return 0
