"""Activate one practice mission without GPS or competition progression."""
from fma_interfaces.msg import MissionState
from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile


SUPPORTED_MISSIONS = (
    'RAMP', 'INTERSECTION_STRAIGHT_1', 'S_CURVE', 'INTERSECTION_STRAIGHT_2',
    'PERPENDICULAR_PARKING', 'INTERSECTION_LEFT', 'CHILD_DUMMY',
    'PARALLEL_PARKING', 'INTERSECTION_RIGHT', 'SIGNAL_CAR', 'LANE_CHANGE')


def mission_enum(name):
    if not isinstance(name, str) or name not in SUPPORTED_MISSIONS:
        raise ValueError('test_mission is required and must be one of: '
                         + ', '.join(SUPPORTED_MISSIONS))
    return getattr(MissionState, name)


class MissionTestRunnerNode(Node):
    def __init__(self):
        super().__init__('mission_test_runner')
        try:
            name = self.declare_parameter(
                'test_mission', '', ParameterDescriptor(read_only=True)).value
            self.mission = mission_enum(name)
        except Exception:
            self.destroy_node()
            raise
        self.completed = False
        self.publisher = self.create_publisher(
            MissionState, '/mission/current',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.subscription = self.create_subscription(
            MissionState, '/mission/status', self.on_status, 10)
        self.timer = self.create_timer(
            .5, self.publish_state, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().info(
            '\n========================================\n'
            f'FMA SINGLE MISSION TEST\nMission: {name}\nGPS: DISABLED\n'
            '========================================')
        self.get_logger().warning('Do not run mission_manager at the same time.')
        self.publish_state()

    def publish_state(self):
        msg = MissionState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = ''
        msg.current_mission = self.mission
        msg.active = not self.completed
        msg.completed = self.completed
        self.publisher.publish(msg)

    def on_status(self, msg):
        if not self.completed and msg.current_mission == self.mission and msg.completed:
            self.completed = True
            self.publish_state()
            self.get_logger().info('TEST COMPLETE')
        # Remain alive publishing this completed state; never chain or restart.


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = MissionTestRunnerNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as error:
        rclpy.logging.get_logger('mission_test_runner').error(str(error))
        return 1
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0
