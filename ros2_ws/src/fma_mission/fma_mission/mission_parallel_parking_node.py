"""Parallel parking mission controller - dry-run state machine.

This version validates:
- mission activation
- ParkingInfo reception
- parking state transitions
- mission completion reporting

IMPORTANT:
Vehicle motion is intentionally disabled.
Every control cycle publishes STOP.
"""

from enum import Enum, auto
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile

from fma_interfaces.msg import DriveCommand, MissionState, ParkingInfo


class ParkingStage(Enum):
    IDLE = auto()
    APPROACH = auto()
    STOP_BEFORE_REVERSE = auto()
    REVERSE_TURN_IN = auto()
    REVERSE_COUNTER_STEER = auto()
    ALIGN = auto()
    PARKED = auto()
    COMPLETE = auto()


class ParallelParkingNode(Node):

    def __init__(self):
        super().__init__('mission_parallel_parking')

        self.active = False
        self.stage = ParkingStage.IDLE
        self.stage_started_at = time.monotonic()

        self.parking_info = None
        self.last_parking_receive_time = None
        self.last_parking_log_time = 0.0

        # --------------------------------------------------------------
        # Publishers
        # --------------------------------------------------------------

        self.cmd_pub = self.create_publisher(
            DriveCommand,
            '/cmd/mission',
            10,
        )

        self.status_pub = self.create_publisher(
            MissionState,
            '/mission/status',
            10,
        )

        # --------------------------------------------------------------
        # Subscribers
        # --------------------------------------------------------------

        mission_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.mission_sub = self.create_subscription(
            MissionState,
            '/mission/current',
            self.on_mission,
            mission_qos,
        )

        self.parking_sub = self.create_subscription(
            ParkingInfo,
            '/perception/parking',
            self.on_parking,
            10,
        )

        # 20 Hz state/control loop
        self.timer = self.create_timer(
            0.05,
            self.control_loop,
        )

        self.get_logger().info(
            'Parallel parking controller ready - DRY RUN'
        )

    # ==============================================================
    # Mission handling
    # ==============================================================

    def on_mission(self, msg):
        should_activate = (
            msg.active
            and msg.current_mission == MissionState.PARALLEL_PARKING
        )

        if should_activate and not self.active:
            self.activate()

        elif not should_activate and self.active:
            self.deactivate()

    def activate(self):
        self.active = True

        self.parking_info = None
        self.last_parking_receive_time = None

        self.set_stage(ParkingStage.APPROACH)

        self.get_logger().info(
            'PARALLEL_PARKING activated'
        )

    def deactivate(self):
        self.publish_stop()

        self.active = False
        self.parking_info = None
        self.last_parking_receive_time = None

        self.set_stage(ParkingStage.IDLE)

        self.get_logger().info(
            'PARALLEL_PARKING deactivated'
        )

    # ==============================================================
    # Parking perception
    # ==============================================================

    def on_parking(self, msg):
        self.parking_info = msg
        self.last_parking_receive_time = time.monotonic()

        now = time.monotonic()

        if now - self.last_parking_log_time >= 0.5:
            self.get_logger().info(
                'ParkingInfo '
                f'valid={msg.valid} '
                f'front={msg.front_clearance_m:.2f}m '
                f'rear={msg.rear_clearance_m:.2f}m '
                f'left={msg.left_clearance_m:.2f}m '
                f'right={msg.right_clearance_m:.2f}m'
            )

            self.last_parking_log_time = now

    def parking_data_ready(self):
        if self.parking_info is None:
            return False

        if self.last_parking_receive_time is None:
            return False

        age = time.monotonic() - self.last_parking_receive_time

        if age > 2.0:
            return False

        if not self.parking_info.valid:
            return False

        return True

    # ==============================================================
    # Drive output
    # ==============================================================

    def publish_drive(self, speed_mps, steering_angle_rad):
        msg = DriveCommand()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = ''

        msg.speed_mps = float(speed_mps)
        msg.steering_angle_rad = float(steering_angle_rad)

        msg.emergency_stop = False
        msg.pwm_control = False
        msg.drive_pwm = 0

        self.cmd_pub.publish(msg)

    def publish_stop(self):
        self.publish_drive(
            speed_mps=0.0,
            steering_angle_rad=0.0,
        )

    # ==============================================================
    # State handling
    # ==============================================================

    def set_stage(self, new_stage):
        if new_stage == self.stage:
            return

        old_stage = self.stage

        self.stage = new_stage
        self.stage_started_at = time.monotonic()

        self.get_logger().info(
            f'Parking stage: {old_stage.name} -> {new_stage.name}'
        )

    def stage_age(self):
        return time.monotonic() - self.stage_started_at

    # ==============================================================
    # Main loop
    # ==============================================================

    def control_loop(self):
        if not self.active:
            return

        #
        # DRY RUN SAFETY:
        # No parking stage is allowed to move the vehicle yet.
        #
        self.publish_stop()

        if self.stage == ParkingStage.APPROACH:
            self.run_approach()

        elif self.stage == ParkingStage.STOP_BEFORE_REVERSE:
            self.run_stop_before_reverse()

        elif self.stage == ParkingStage.REVERSE_TURN_IN:
            self.run_reverse_turn_in()

        elif self.stage == ParkingStage.REVERSE_COUNTER_STEER:
            self.run_reverse_counter_steer()

        elif self.stage == ParkingStage.ALIGN:
            self.run_align()

        elif self.stage == ParkingStage.PARKED:
            self.run_parked()

        elif self.stage == ParkingStage.COMPLETE:
            self.report_complete()

    # ==============================================================
    # DRY-RUN state transitions
    #
    # The numbers below are TEST VALUES ONLY.
    # They are NOT the final physical parking thresholds.
    # ==============================================================

    def run_approach(self):
        if not self.parking_data_ready():
            return

        info = self.parking_info

        # Synthetic test:
        # enough right-side/rear space means entry point detected.
        if (
            info.right_clearance_m >= 0.80
            and info.rear_clearance_m >= 1.00
        ):
            self.set_stage(
                ParkingStage.STOP_BEFORE_REVERSE
            )

    def run_stop_before_reverse(self):
        self.publish_stop()

        # Confirm a stationary pause before reverse stage.
        if self.stage_age() >= 0.5:
            self.set_stage(
                ParkingStage.REVERSE_TURN_IN
            )

    def run_reverse_turn_in(self):
        if not self.parking_data_ready():
            return

        info = self.parking_info

        # Synthetic test:
        # rear distance became smaller during first reverse arc.
        if info.rear_clearance_m <= 0.80:
            self.set_stage(
                ParkingStage.REVERSE_COUNTER_STEER
            )

    def run_reverse_counter_steer(self):
        if not self.parking_data_ready():
            return

        info = self.parking_info

        # Synthetic test:
        # vehicle moved close enough to parking-side boundary.
        if info.right_clearance_m <= 0.45:
            self.set_stage(
                ParkingStage.ALIGN
            )

    def run_align(self):
        if not self.parking_data_ready():
            return

        info = self.parking_info

        # Synthetic test:
        # require useful front/rear clearance and approximately centred.
        front_ok = info.front_clearance_m >= 0.40
        rear_ok = info.rear_clearance_m >= 0.40

        centred = (
            abs(
                info.front_clearance_m
                - info.rear_clearance_m
            )
            <= 0.15
        )

        if front_ok and rear_ok and centred:
            self.set_stage(
                ParkingStage.PARKED
            )

    def run_parked(self):
        self.publish_stop()

        if self.stage_age() >= 0.5:
            self.set_stage(
                ParkingStage.COMPLETE
            )

    # ==============================================================
    # Completion
    # ==============================================================

    def report_complete(self):
        self.publish_stop()

        msg = MissionState()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = ''

        msg.current_mission = MissionState.PARALLEL_PARKING
        msg.active = False
        msg.completed = True

        self.status_pub.publish(msg)

        self.get_logger().info(
            'PARALLEL_PARKING completed'
        )

        self.active = False
        self.stage = ParkingStage.IDLE


def main(args=None):
    rclpy.init(args=args)

    node = None

    try:
        node = ParallelParkingNode()
        rclpy.spin(node)

    except (KeyboardInterrupt, ExternalShutdownException):
        pass

    finally:
        if node is not None:
            node.publish_stop()
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
