"""LEFT lane change using observed or inferred lane separator."""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String

from fma_interfaces.msg import (
    DriveCommand,
    LaneBoundaryState,
    MissionState,
)

from fma_mission.lane_change_maneuver import (
    IDLE,
    LEFT,
    REACQUIRE,
    CrossingConfig,
    LaneBoundaryCrossing,
    LaneChangeConfig,
    LaneChangeManeuver,
    resolve_lane_width,
    separator_before_crossing,
    separator_after_crossing,
)


class LaneChangeManeuverNode(Node):

    def __init__(self):
        super().__init__('lane_change_maneuver')

        self.declare_parameter('enable_drive', False)

        self.declare_parameter('forward_speed_mps', 0.12)
        self.declare_parameter('shift_steering_rad', 0.12)
        self.declare_parameter('counter_steering_rad', 0.10)

        self.declare_parameter('min_shift_sec', 0.40)
        self.declare_parameter('max_shift_sec', 4.00)
        self.declare_parameter('counter_duration_sec', 0.80)
        self.declare_parameter('max_maneuver_sec', 8.00)

        self.declare_parameter('crossing_arm_distance_m', 0.90)
        self.declare_parameter('crossing_confirm_distance_m', 0.20)
        self.declare_parameter('boundary_confidence', 0.55)
        self.declare_parameter('crossing_confirm_samples', 2)

        self.declare_parameter('lane_width_min_m', 3.0)
        self.declare_parameter('lane_width_max_m', 3.5)
        self.declare_parameter('fallback_lane_width_m', 3.25)

        self.declare_parameter('lane_confidence', 0.60)
        self.declare_parameter('center_tolerance_m', 0.18)
        self.declare_parameter('heading_tolerance_rad', 0.15)

        self.declare_parameter('reacquire_samples', 4)
        self.declare_parameter('single_reacquire_samples', 6)

        self.declare_parameter('output_rate_hz', 20.0)

        self.enable_drive = bool(
            self.get_parameter('enable_drive').value
        )

        self.maneuver = LaneChangeManeuver(
            LaneChangeConfig(
                forward_speed_mps=float(
                    self.get_parameter('forward_speed_mps').value
                ),
                shift_steering_rad=float(
                    self.get_parameter('shift_steering_rad').value
                ),
                counter_steering_rad=float(
                    self.get_parameter('counter_steering_rad').value
                ),
                min_shift_sec=float(
                    self.get_parameter('min_shift_sec').value
                ),
                max_shift_sec=float(
                    self.get_parameter('max_shift_sec').value
                ),
                counter_duration_sec=float(
                    self.get_parameter('counter_duration_sec').value
                ),
                max_maneuver_sec=float(
                    self.get_parameter('max_maneuver_sec').value
                ),
            )
        )

        self.crossing = LaneBoundaryCrossing(
            CrossingConfig(
                arm_distance_m=float(
                    self.get_parameter(
                        'crossing_arm_distance_m'
                    ).value
                ),
                confirm_distance_m=float(
                    self.get_parameter(
                        'crossing_confirm_distance_m'
                    ).value
                ),
                min_confidence=float(
                    self.get_parameter(
                        'boundary_confidence'
                    ).value
                ),
                confirm_samples=int(
                    self.get_parameter(
                        'crossing_confirm_samples'
                    ).value
                ),
            )
        )

        self.lane_width_min_m = float(
            self.get_parameter('lane_width_min_m').value
        )
        self.lane_width_max_m = float(
            self.get_parameter('lane_width_max_m').value
        )
        self.fallback_lane_width_m = float(
            self.get_parameter('fallback_lane_width_m').value
        )

        self.lane_confidence = float(
            self.get_parameter('lane_confidence').value
        )
        self.center_tolerance_m = float(
            self.get_parameter('center_tolerance_m').value
        )
        self.heading_tolerance_rad = float(
            self.get_parameter('heading_tolerance_rad').value
        )

        self.reacquire_samples = int(
            self.get_parameter('reacquire_samples').value
        )
        self.single_reacquire_samples = int(
            self.get_parameter(
                'single_reacquire_samples'
            ).value
        )

        self.active = False
        self.request = None
        self.boundary_crossed = False

        self.lane_good_streak = 0
        self.reacquire_source = None
        self.last_logged_phase = None

        transient = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.command_pub = self.create_publisher(
            DriveCommand,
            '/cmd/mission',
            1,
        )

        self.create_subscription(
            MissionState,
            '/mission/current',
            self.on_mission,
            transient,
        )

        self.create_subscription(
            String,
            '/mission/lane_change_request',
            self.on_request,
            transient,
        )

        self.create_subscription(
            LaneBoundaryState,
            '/perception/lane_boundary',
            self.on_boundary,
            10,
        )

        rate = float(
            self.get_parameter('output_rate_hz').value
        )

        self.timer = self.create_timer(
            1.0 / rate,
            self.tick,
        )

        self.get_logger().info(
            'Lane-change maneuver ready '
            f'enable_drive={self.enable_drive}'
        )

    def reset_runtime(self):
        self.maneuver.reset()
        self.crossing.reset()

        self.request = None
        self.boundary_crossed = False

        self.lane_good_streak = 0
        self.reacquire_source = None
        self.last_logged_phase = None

    def on_mission(self, msg):
        active = (
            msg.active
            and msg.current_mission == MissionState.LANE_CHANGE
        )

        if active and not self.active:
            self.crossing.reset()
            self.boundary_crossed = False
            self.lane_good_streak = 0
            self.reacquire_source = None

            self.get_logger().info('LANE_CHANGE entered')

        if not active and self.active:
            self.reset_runtime()

        self.active = active

    def on_request(self, msg):
        if msg.data not in ('KEEP', 'LEFT'):
            self.get_logger().warning(
                f'Invalid lane-change request ignored: {msg.data}'
            )
            return

        self.request = msg.data

        if msg.data == LEFT:
            self.crossing.reset()
            self.boundary_crossed = False

        self.get_logger().info(
            f'LANE_CHANGE request={self.request}'
        )

    def on_boundary(self, msg):
        if not self.active or self.request != LEFT:
            return

        expected = (
            float(msg.expected_lane_width_m)
            if msg.expected_lane_width_valid
            else None
        )

        selected = (
            float(msg.selected_lane_width_m)
            if msg.selected_lane_width_valid
            else None
        )

        width, width_source = resolve_lane_width(
            expected,
            selected,
            minimum_m=self.lane_width_min_m,
            maximum_m=self.lane_width_max_m,
            fallback_m=self.fallback_lane_width_m,
        )

        if not self.boundary_crossed:

            if not self.crossing.armed:
                x, confidence, separator_source = (
                    separator_before_crossing(
                        left_valid=msg.left_valid,
                        left_x=msg.left_x_m,
                        left_confidence=msg.left_confidence,
                        right_valid=msg.right_valid,
                        right_x=msg.right_x_m,
                        right_confidence=msg.right_confidence,
                        lane_width=width,
                    )
                )

                if x is not None:
                    self.crossing.update(
                        valid=True,
                        side='LEFT',
                        x_m=x,
                        confidence=confidence,
                    )

            else:
                x, confidence, separator_source = (
                    separator_after_crossing(
                        left_valid=msg.left_valid,
                        left_x=msg.left_x_m,
                        left_confidence=msg.left_confidence,
                        right_valid=msg.right_valid,
                        right_x=msg.right_x_m,
                        right_confidence=msg.right_confidence,
                        lane_width=width,
                    )
                )

                if x is not None:
                    crossed = self.crossing.update(
                        valid=True,
                        side='RIGHT',
                        x_m=x,
                        confidence=confidence,
                    )

                    if crossed:
                        self.boundary_crossed = True

                        self.get_logger().info(
                            'LANE boundary crossed '
                            f'x={x:.3f}m '
                            f'width={width:.3f}m '
                            f'width_source={width_source} '
                            f'separator={separator_source}'
                        )

        if self.maneuver.phase != REACQUIRE:
            self.lane_good_streak = 0
            self.reacquire_source = None
            return

        source = str(msg.control_source)

        valid_source = source in (
            'PAIR_TRACK',
            'SINGLE_LEFT_TRACK',
            'SINGLE_RIGHT_TRACK',
        )

        good = (
            msg.center_valid
            and valid_source
            and msg.tracking_confidence >= self.lane_confidence
            and math.isfinite(float(msg.center_lateral_error_m))
            and math.isfinite(float(msg.center_heading_error_rad))
            and abs(float(msg.center_lateral_error_m))
            <= self.center_tolerance_m
            and abs(float(msg.center_heading_error_rad))
            <= self.heading_tolerance_rad
        )

        if good:
            if source != self.reacquire_source:
                self.reacquire_source = source
                self.lane_good_streak = 1
            else:
                self.lane_good_streak += 1
        else:
            self.reacquire_source = None
            self.lane_good_streak = 0

    def log_phase(self):
        if self.maneuver.phase == self.last_logged_phase:
            return

        self.last_logged_phase = self.maneuver.phase

        self.get_logger().info(
            f'LANE_CHANGE phase={self.maneuver.phase}'
        )

    def tick(self):
        if not self.active:
            return

        now = time.monotonic()

        if (
            self.maneuver.phase == IDLE
            and self.request is not None
        ):
            self.maneuver.start(self.request, now)
            self.log_phase()

        if self.maneuver.phase == IDLE:
            return

        required = self.reacquire_samples

        if self.reacquire_source in (
            'SINGLE_LEFT_TRACK',
            'SINGLE_RIGHT_TRACK',
        ):
            required = self.single_reacquire_samples

        reacquired = self.lane_good_streak >= required

        self.maneuver.update(
            now,
            dashed_crossed=self.boundary_crossed,
            lane_reacquired=reacquired,
        )

        self.log_phase()

        if not self.enable_drive:
            return

        command = self.maneuver.command()

        if command is None:
            return

        speed, steering, emergency = command

        msg = DriveCommand()
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.speed_mps = speed
        msg.steering_angle_rad = steering
        msg.emergency_stop = emergency
        msg.pwm_control = False
        msg.drive_pwm = 0

        self.command_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = LaneChangeManeuverNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
