"""GPS triggers the ordered course; mission nodes report completion separately."""
import json
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
from std_msgs.msg import String

from fma_mission.control_mode import control_mode_for_mission
from fma_mission.waypoint_progress import WaypointProgress, load_waypoints


class MissionManagerNode(Node):
    def __init__(self):
        super().__init__('mission_manager')
        default = Path(get_package_share_directory('fma_mission')) / 'config' / 'waypoints.yaml'
        path = self.declare_parameter('waypoints_file', str(default),
                                      ParameterDescriptor(read_only=True)).value
        self.progress = WaypointProgress(load_waypoints(path))
        self.last_log = -float('inf')
        self.control_mode = None
        self.last_reported_mission = None
        self.last_diagnostic = None
        self.proceed_at = None
        self.feedback_context = None
        self.control_mode_publisher = self.create_publisher(
            String, '/mission/control_mode',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.publisher = self.create_publisher(
            MissionState, '/mission/current',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.gps_subscription = self.create_subscription(
            NavSatFix, '/gps/fix', self.on_gps, qos_profile_sensor_data)
        self.status_subscription = self.create_subscription(
            MissionState, '/mission/status', self.on_status, 10)
        self.feedback_subscription = self.create_subscription(
            String, '/mission/feedback', self.on_feedback, 10)
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
        context = (self.progress.state, self.progress.phase)
        if context != self.feedback_context:
            self.proceed_at = None
            self.feedback_context = context
        proceed = self.proceed_at is not None and time.monotonic() - self.proceed_at < 1.5
        mode = control_mode_for_mission(self.progress.state, self.progress.phase, proceed=proceed)
        if mode != self.control_mode:
            self.get_logger().info(
                f'CONTROL MODE {self.control_mode or "INITIAL"} -> {mode} '
                f'reason=mission_{self.progress.state.lower()}')
            self.control_mode = mode
        if self.progress.legacy:
            if self.last_reported_mission != self.progress.state:
                self.get_logger().info(
                    f'MISSION mission={self.progress.state} control_mode={mode} '
                    f'waypoint={self.progress.target.id}')
                self.last_reported_mission = self.progress.state
        else:
            w = self.progress.target
            diagnostic = (w.number, self.progress.state, self.progress.phase, mode)
            if diagnostic != self.last_diagnostic:
                self.get_logger().info(
                    f'WAYPOINT number={w.number} id={w.id} type={w.waypoint_type} '
                    f'hard_point={str(w.hard_point).lower()} '
                    f'mission={w.mission or w.mission_entry or "null"}')
                self.get_logger().info(
                    f'MISSION state={self.progress.state} control_mode={mode} '
                    f'phase={self.progress.phase or "null"}')
                self.last_diagnostic = diagnostic
            for waypoint, event in self.progress.events:
                reason = 'coordinate unavailable, skipped' if waypoint.number == 57 else event
                self.get_logger().info(f'WAYPOINT number={waypoint.number} id={waypoint.id} {reason}')
            self.progress.events.clear()
        self.control_mode_publisher.publish(String(data=mode))

    def tick(self):
        self.progress.start()
        self.publish_state()

    def on_gps(self, msg):
        distance = self.progress.gps(msg.latitude, msg.longitude, msg.status.status in (0, 1, 2))
        if distance is not None:
            self.publish_state()

    def on_status(self, msg):
        if self.progress.complete(msg.current_mission, msg.completed):
            self.publish_state()

    def on_feedback(self, msg):
        """Minimal JSON adapter; feedback confirms actual phase entry, never a target."""
        try:
            data = json.loads(msg.data)
            if (not isinstance(data, dict) or set(data) - {'mission', 'phase', 'proceed'}
                    or data.get('mission') != self.progress.state or not self.progress.active):
                raise ValueError('Invalid or inactive mission')
            phase = data.get('phase', self.progress.phase)
            if self.progress.state in ('PERPENDICULAR_PARKING', 'PARALLEL_PARKING'):
                order = ('APPROACH', 'ALIGN', 'REVERSE', 'PARKED', 'COMPLETE')
                current = self.progress.phase
                if (phase not in order or current not in order
                        or order.index(phase) < order.index(current)
                        or order.index(phase) > order.index(current) + 1
                        and not (current == 'APPROACH' and phase == 'REVERSE'
                                 and self.progress.state == 'PARALLEL_PARKING')):
                    raise ValueError('Invalid parking phase transition')
            elif phase != self.progress.phase:
                raise ValueError('Phase must match processed waypoint')
            if 'proceed' in data and type(data['proceed']) is not bool:
                raise ValueError('proceed must be boolean')
            self.progress.phase = phase
            self.feedback_context = (self.progress.state, phase)
            self.proceed_at = time.monotonic() if data.get('proceed') is True else None
        except (ValueError, TypeError):
            self.proceed_at = None
            self.get_logger().warning('Invalid mission feedback; proceed permission revoked')
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
