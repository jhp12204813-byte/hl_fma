"""SIGNAL_CAR LEFT/KEEP request -> lane-change maneuver."""

import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String

from fma_interfaces.msg import DriveCommand, Lane, MissionState

from fma_mission.lane_change_maneuver import (
    DONE,
    FAILED,
    IDLE,
    REACQUIRE,
    LaneChangeConfig,
    LaneChangeManeuver,
)


class LaneChangeManeuverNode(Node):

    def __init__(self):
        super().__init__('lane_change_maneuver')

        # SAFETY: false means absolutely no /cmd/mission vehicle command.
        self.declare_parameter('enable_drive', False)

        # Initial conservative placeholders.
        # These MUST be tuned on the real vehicle before enabling drive.
        self.declare_parameter('forward_speed_mps', 0.12)
        self.declare_parameter('shift_steering_rad', 0.10)
        self.declare_parameter('counter_steering_rad', 0.08)
        self.declare_parameter('shift_duration_sec', 1.0)
        self.declare_parameter('counter_duration_sec', 0.8)
        self.declare_parameter('max_maneuver_sec', 6.0)

        self.declare_parameter('lane_confidence', 0.60)
        self.declare_parameter('center_tolerance_m', 0.15)
        self.declare_parameter('heading_tolerance_rad', 0.15)
        self.declare_parameter('reacquire_samples', 4)
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
                shift_duration_sec=float(
                    self.get_parameter('shift_duration_sec').value
                ),
                counter_duration_sec=float(
                    self.get_parameter('counter_duration_sec').value
                ),
                max_maneuver_sec=float(
                    self.get_parameter('max_maneuver_sec').value
                ),
            )
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

        self.active = False
        self.request = None
        self.lane_good_streak = 0
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
            Lane,
            '/perception/lane',
            self.on_lane,
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

    def on_mission(self, msg):
        active = (
            msg.active
            and msg.current_mission == MissionState.LANE_CHANGE
        )

        if active and not self.active:
            self.get_logger().info('LANE_CHANGE entered')

        if not active and self.active:
            self.maneuver.reset()
            self.request = None
            self.lane_good_streak = 0

        self.active = active

    def on_request(self, msg):
        if msg.data not in ('KEEP', 'LEFT'):
            self.get_logger().warning(
                f'Invalid lane-change request ignored: {msg.data}'
            )
            return

        self.request = msg.data

        self.get_logger().info(
            f'LANE_CHANGE request={self.request}'
        )

    def on_lane(self, msg):
        if (
            not self.active
            or self.maneuver.phase != REACQUIRE
        ):
            self.lane_good_streak = 0
            return

        good = (
            msg.detected
            and msg.confidence >= self.lane_confidence
            and abs(msg.lateral_error_m)
            <= self.center_tolerance_m
            and abs(msg.heading_error_rad)
            <= self.heading_tolerance_rad
        )

        if good:
            self.lane_good_streak += 1
        else:
            self.lane_good_streak = 0

    def log_phase(self):
        if self.maneuver.phase == self.last_logged_phase:
            return

        self.last_logged_phase = self.maneuver.phase

        self.get_logger().info(
            'LANE_CHANGE phase='
            f'{self.maneuver.phase}'
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

        reacquired = (
            self.lane_good_streak >= self.reacquire_samples
        )

        self.maneuver.update(
            now,
            lane_reacquired=reacquired,
        )

        self.log_phase()

        # Dry-run means NO VEHICLE COMMANDS.
        if not self.enable_drive:
            return

        command = self.maneuver.command()

        if command is None:
            # REACQUIRE/DONE: stop publishing /cmd/mission.
            # The arbiter will allow /cmd/lane to take over
            # after mission candidate freshness expires.
            return

        speed, steering, emergency = command

        msg = DriveCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = ''
        msg.speed_mps = speed
        msg.steering_angle_rad = steering
        msg.emergency_stop = emergency
        msg.pwm_control = False
        msg.drive_pwm = 0

        self.command_pub.publish(msg)

        if self.maneuver.phase == FAILED:
            self.get_logger().error(
                'LANE_CHANGE failed; publishing STOP'
            )


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
