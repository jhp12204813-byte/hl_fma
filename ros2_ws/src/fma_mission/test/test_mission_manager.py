"""Offline course validation and mocked ROS callbacks; no hardware or DDS."""
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest
from builtin_interfaces.msg import Time
from fma_interfaces.msg import MissionState
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String
from rclpy.qos import DurabilityPolicy, ReliabilityPolicy
from fma_mission import mission_manager_node as node_module
from fma_mission.waypoint_progress import WaypointProgress, load_waypoints, haversine_m

CONFIG = Path(__file__).resolve().parents[1] / 'config' / 'waypoints.yaml'
@pytest.fixture
def node(monkeypatch):
    pub, sub, clock = MagicMock(), MagicMock(), MagicMock()
    mission_publisher, mode_publisher = MagicMock(), MagicMock()
    pub.side_effect = lambda msg_type, *args: (
        mission_publisher if msg_type is MissionState else mode_publisher)
    clock.now.return_value.to_msg.return_value = Time(sec=123)
    monkeypatch.setattr(node_module.Node, '__init__', lambda self, name: None)
    monkeypatch.setattr(node_module, 'get_package_share_directory', lambda name: str(CONFIG.parent.parent))
    monkeypatch.setattr(node_module.Node, 'declare_parameter',
                        lambda self, name, value, descriptor: SimpleNamespace(value=str(CONFIG) if name == 'waypoints_file' else value))
    monkeypatch.setattr(node_module.Node, 'create_publisher', pub)
    monkeypatch.setattr(node_module.Node, 'create_subscription', sub)
    monkeypatch.setattr(node_module.Node, 'create_timer', MagicMock())
    monkeypatch.setattr(node_module.Node, 'get_clock', lambda self: clock)
    monkeypatch.setattr(node_module.Node, 'get_logger', lambda self: MagicMock())
    n = node_module.MissionManagerNode()
    assert [c.args[:2] for c in pub.call_args_list] == [
        (String, '/mission/control_mode'), (MissionState, '/mission/current')]
    for call in pub.call_args_list:
        qos = call.args[2]
        assert qos.depth == 1 and qos.durability == DurabilityPolicy.TRANSIENT_LOCAL
        assert qos.reliability == ReliabilityPolicy.RELIABLE
    assert [c.args[1] for c in sub.call_args_list] == ['/gps/fix', '/mission/status', '/mission/feedback']
    return n



def gps(node):
    w = node.progress.target
    fix = NavSatFix(latitude=w.latitude, longitude=w.longitude)
    fix.status.status = 0
    node.on_gps(fix)


def reach(node, number):
    node.tick()
    for _ in range(100):
        if node.progress.target.number == number:
            return
        gps(node)
    pytest.fail('blocked')


def test_course_publications_atomic_transition_and_finish(node):
    reach(node, 37)
    node.publisher.publish.reset_mock()
    gps(node)
    states = [c.args[0].current_mission for c in node.publisher.publish.call_args_list]
    assert states == [MissionState.PERPENDICULAR_PARKING]
    assert node.control_mode == 'LANE'
    reach(node, 90)
    gps(node)
    node.tick()
    assert node.control_mode == 'FINISH'
    msg = node.publisher.publish.call_args.args[0]
    assert msg.current_mission == MissionState.FINISH and msg.completed and not msg.active


def test_diagnostic_coordinate_skip_and_soft_target(node):
    logger = MagicMock()
    node.get_logger = lambda: logger
    reach(node, 31)
    logs = [c.args[0] for c in logger.info.call_args_list]
    assert any('number=31 id=S_CURVE_GUIDE_1 type=mission_guide hard_point=false mission=S_CURVE' in s for s in logs)
    reach(node, 56)
    gps(node)
    assert any('number=57 id=ROUTE_057 coordinate unavailable, skipped' in c.args[0]
               for c in logger.info.call_args_list)


def test_feedback_permissions_expire_and_phase_matches_progress(node, monkeypatch):
    now = [0.0]
    monkeypatch.setattr(node_module.time, 'monotonic', lambda: now[0])
    reach(node, 23)
    assert node.control_mode == 'STOP'
    node.on_feedback(String(data='{"mission":"INTERSECTION_STRAIGHT_1","proceed":true}'))
    assert node.control_mode == 'LANE'
    now[0] = 1.5
    node.tick()
    assert node.control_mode == 'STOP'
    node.on_feedback(String(data='{"mission":"INTERSECTION_STRAIGHT_1","phase":"EXIT","proceed":true}'))
    assert node.control_mode == 'STOP' and node.progress.phase == 'ENTRY'
    reach(node, 82)
    assert node.progress.phase == 'APPROACH'
    node.on_feedback(String(data='{"mission":"PARALLEL_PARKING","phase":"REVERSE"}'))
    assert node.progress.phase == 'REVERSE' and node.control_mode == 'REVERSE'
    node.on_feedback(String(data='{"mission":"PARALLEL_PARKING","phase":"APPROACH"}'))
    assert node.progress.phase == 'REVERSE'
