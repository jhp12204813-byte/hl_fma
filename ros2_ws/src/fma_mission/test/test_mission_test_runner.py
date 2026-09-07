"""Single-mission runner tests with mocked ROS transport and no GPS/hardware."""
from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest
from builtin_interfaces.msg import Time
from fma_interfaces.msg import MissionState
from rclpy.clock import ClockType
from rclpy.qos import DurabilityPolicy, ReliabilityPolicy
from fma_mission import mission_test_runner_node as runner


@pytest.mark.parametrize('name,value', [
    ('RAMP', 2), ('INTERSECTION_STRAIGHT_1', 3), ('S_CURVE', 4),
    ('INTERSECTION_STRAIGHT_2', 5), ('PERPENDICULAR_PARKING', 6),
    ('INTERSECTION_LEFT', 7), ('CHILD_DUMMY', 8), ('PARALLEL_PARKING', 9),
    ('INTERSECTION_RIGHT', 10), ('SIGNAL_CAR', 11), ('LANE_CHANGE', 12)])
def test_mission_enum(name, value):
    assert runner.mission_enum(name) == value


@pytest.fixture
def env(monkeypatch):
    publisher, subscriber, timer, clock, logger, destroy = [MagicMock() for _ in range(6)]
    clock.now.return_value.to_msg.return_value = Time(sec=123)
    overrides = {}
    monkeypatch.setattr(runner.Node, '__init__', lambda self, name: None)
    monkeypatch.setattr(runner.Node, 'declare_parameter',
                        lambda self, name, default, descriptor:
                        SimpleNamespace(value=overrides.get(name, default)))
    monkeypatch.setattr(runner.Node, 'create_publisher', publisher)
    monkeypatch.setattr(runner.Node, 'create_subscription', subscriber)
    monkeypatch.setattr(runner.Node, 'create_timer', timer)
    monkeypatch.setattr(runner.Node, 'get_clock', lambda self: clock)
    monkeypatch.setattr(runner.Node, 'get_logger', lambda self: logger)
    monkeypatch.setattr(runner.Node, 'destroy_node', destroy)

    def make(**parameters):
        overrides.update(parameters)
        return runner.MissionTestRunnerNode()

    return SimpleNamespace(make=make, publisher=publisher, subscriber=subscriber,
                           timer=timer, logger=logger, destroy=destroy)


@pytest.mark.parametrize('name', ['', 'UNKNOWN', 's_curve', 'START', 'NORMAL_DRIVE', 'FINISH', None])
def test_invalid_no_activation(env, name):
    with pytest.raises(ValueError, match='test_mission is required'):
        env.make(test_mission=name)
    env.publisher.assert_not_called()
    env.subscriber.assert_not_called()
    env.timer.assert_not_called()
    env.destroy.assert_called_once()


def test_missing_parameter(env):
    with pytest.raises(ValueError):
        env.make()
    env.publisher.assert_not_called()


def latest(node):
    return node.publisher.publish.call_args.args[0]


def test_startup_topics_qos_timer_and_header(env):
    node = env.make(test_mission='S_CURVE')
    assert env.publisher.call_count == 1
    assert env.publisher.call_args.args[:2] == (MissionState, '/mission/current')
    qos = env.publisher.call_args.args[2]
    assert qos.depth == 1 and qos.durability == DurabilityPolicy.TRANSIENT_LOCAL
    assert qos.reliability == ReliabilityPolicy.RELIABLE
    assert env.subscriber.call_count == 1  # Only status, no GPS or motion topics.
    assert env.subscriber.call_args.args[:2] == (MissionState, '/mission/status')
    assert env.timer.call_args.args == (.5, node.publish_state)
    assert env.timer.call_args.kwargs['clock'].clock_type == ClockType.STEADY_TIME
    msg = latest(node)
    assert (msg.current_mission, msg.active, msg.completed) == (MissionState.S_CURVE, True, False)
    assert msg.header.stamp == Time(sec=123) and msg.header.frame_id == ''
    env.logger.warning.assert_called_once_with('Do not run mission_manager at the same time.')


@pytest.mark.parametrize('name', runner.SUPPORTED_MISSIONS)
def test_matching_completion_never_chains(env, name):
    node = env.make(test_mission=name)
    selected = runner.mission_enum(name)
    for mission in range(14):
        node.on_status(MissionState(current_mission=mission, completed=False))
        if mission != selected:
            node.on_status(MissionState(current_mission=mission, completed=True))
        assert not node.completed
    node.on_status(MissionState(current_mission=selected, completed=True))
    for mission in range(14):
        node.on_status(MissionState(current_mission=mission, completed=True))
        node.publish_state()
        msg = latest(node)
        assert (msg.current_mission, msg.active, msg.completed) == (selected, False, True)
    assert sum(c.args == ('TEST COMPLETE',) for c in env.logger.info.call_args_list) == 1


def test_main_invalid_reports_error_and_exits(env, monkeypatch):
    monkeypatch.setattr(runner.rclpy, 'init', MagicMock())
    spin, shutdown, logger = MagicMock(), MagicMock(), MagicMock()
    monkeypatch.setattr(runner.rclpy, 'spin', spin)
    monkeypatch.setattr(runner.rclpy, 'ok', lambda: True)
    monkeypatch.setattr(runner.rclpy, 'shutdown', shutdown)
    monkeypatch.setattr(runner.rclpy.logging, 'get_logger', lambda name: logger)
    assert runner.main() == 1
    logger.error.assert_called_once()
    spin.assert_not_called()
    shutdown.assert_called_once()
