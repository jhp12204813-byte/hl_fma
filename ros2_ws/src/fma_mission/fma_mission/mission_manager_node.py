"""GPS triggers the ordered course; mission nodes report completion separately."""
from pathlib import Path
import time

from ament_index_python.packages import get_package_share_directory
from fma_interfaces.msg import MissionState
from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix

from fma_mission.waypoint_progress import WaypointProgress, load_waypoints


class MissionManagerNode(Node):
    def __init__(self):
        super().__init__('mission_manager')
        default = Path(get_package_share_directory('fma_mission')) / 'config' / 'waypoints.yaml'
        path = self.declare_parameter('waypoints_file', str(default),
                                      ParameterDescriptor(read_only=True)).value
        self.progress = WaypointProgress(load_waypoints(path))
        self.last_log = -float('inf')
        self.publisher = self.create_publisher(
            MissionState, '/mission/current',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.gps_subscription = self.create_subscription(
            NavSatFix, '/gps/fix', self.on_gps, qos_profile_sensor_data)
        self.status_subscription = self.create_subscription(
            MissionState, '/mission/status', self.on_status, 10)
        self.timer = self.create_timer(.5, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.publish_state()  # START, then NORMAL_DRIVE on the first timer tick.

    def publish_state(self):
        msg = MissionState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = ''
        msg.current_mission = getattr(MissionState, self.progress.state)
        msg.active = self.progress.active
        msg.completed = self.progress.state == 'FINISH'
        self.publisher.publish(msg)

    def tick(self):
        self.progress.start()
        self.publish_state()

    def on_gps(self, msg):
        previous = self.progress.state
        distance = self.progress.gps(msg.latitude, msg.longitude, msg.status.status in (0, 1, 2))
        if distance is None:
            return
        target = self.progress.target
        if self.progress.state != previous:
            if self.progress.state == 'FINISH':
                self.get_logger().info('FINISH waypoint reached; Mission state: FINISH')
            else:
                self.get_logger().info(f'{target.id} reached; Activating mission: {self.progress.state}')
            self.publish_state()
        elif time.monotonic() - self.last_log >= 2.0:
            self.get_logger().info(
                f'TARGET: {target.id} {target.missions[0]} | DISTANCE: {distance:.2f} m '
                f'| STATE: {self.progress.state}')
            self.last_log = time.monotonic()

    def on_status(self, msg):
        previous = self.progress.state
        if self.progress.complete(msg.current_mission, msg.completed):
            if self.progress.active:
                detail = f'Activating chained mission: {self.progress.state}'
            else:
                detail = f'Next target: {self.progress.target.id} {self.progress.target.missions[0]}'
            self.get_logger().info(f'{previous} completed; {detail}')
            self.publish_state()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = MissionManagerNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
