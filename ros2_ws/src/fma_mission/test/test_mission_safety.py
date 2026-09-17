"""Mocked ROS only: no DDS, vehicle node or hardware is started."""
from unittest.mock import MagicMock

import pytest
from builtin_interfaces.msg import Time
from fma_interfaces.msg import DriveCommand
from rclpy.clock import ClockType
from rclpy.qos import DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import Bool, String

from fma_mission import mission_safety_node as safety


@pytest.fixture
def env(monkeypatch):
    publisher, subscription, timer, clock = [MagicMock() for _ in range(4)]
    now = [0.0]
    clock.now.return_value.to_msg.return_value = Time(sec=123)
    monkeypatch.setattr(safety.time, 'monotonic', lambda: now[0])
    monkeypatch.setattr(safety.Node, '__init__', lambda self, name: None)
    monkeypatch.setattr(safety.Node, 'create_publisher', publisher)
    monkeypatch.setattr(safety.Node, 'create_subscription', subscription)
    monkeypatch.setattr(safety.Node, 'create_timer', timer)
    monkeypatch.setattr(safety.Node, 'get_clock', lambda self: clock)
    monkeypatch.setattr(safety.Node, 'get_logger', lambda self: MagicMock())
    node = safety.MissionSafetyNode()
    return node, now, publisher, subscription, timer


def assert_stop(node):
    msg = node.publisher.publish.call_args.args[0]
    assert msg.speed_mps == 0 and msg.steering_angle_rad == 0
    assert msg.emergency_stop
    assert not msg.use_pwm_override and msg.drive_pwm_percent == 0


def test_startup_topics_qos_timer(env):
    node, now, publisher, subscription, timer = env
    assert publisher.call_args.args == (DriveCommand, '/cmd/mission', 1)
    assert subscription.call_args_list[0].args[:2] == (String, '/mission/control_mode')
    assert subscription.call_args_list[1].args[:2] == (String, '/mission/controller_ready')
    assert subscription.call_args_list[2].args[:2] == (Bool, '/gps/healthy')
    qos = subscription.call_args_list[0].args[3]
    assert qos.depth == 1 and qos.durability == DurabilityPolicy.TRANSIENT_LOCAL
    assert qos.reliability == ReliabilityPolicy.RELIABLE
    assert timer.call_args.args == (.05, node.publish_stop)
    assert timer.call_args.kwargs['clock'].clock_type == ClockType.STEADY_TIME
    assert_stop(node)


@pytest.mark.parametrize('mode', ['STOP', 'FINISH'])
def test_stop_immediate_and_repeated_beyond_command_timeout(env, mode):
    node, now, *_ = env
    node.on_mode(String(data='LANE'))
    node.publisher.publish.reset_mock()
    node.on_mode(String(data=mode))
    assert node.publisher.publish.call_count == 1
    for step in range(1, 101):
        now[0] = step * .05
        node.publish_stop()
        assert_stop(node)
    assert node.publisher.publish.call_count == 101


@pytest.mark.parametrize('mode', ['LANE'])
def test_handoff_emits_no_mission_candidate(env, mode):
    node, now, *_ = env
    node.publisher.publish.reset_mock()
    node.on_mode(String(data=mode))
    for instant in (.05, .5, 1.49):
        now[0] = instant
        node.publish_stop()
    node.publisher.publish.assert_not_called()
    now[0] = 1.5
    node.publish_stop()
    assert_stop(node)
    node.publisher.publish.reset_mock()
    node.on_mode(String(data=mode))
    node.publish_stop()
    node.publisher.publish.assert_not_called()


def test_unknown_mode_fails_closed(env):
    node, *_ = env
    node.on_mode(String(data='lane'))
    assert node.mode == 'STOP'
    assert_stop(node)


@pytest.mark.parametrize('mode', ['OBSTACLE', 'REVERSE'])
def test_controller_readiness_fail_safe_and_expiry(env, mode):
    node, now, *_ = env
    node.on_mode(String(data=mode))
    for instant in (.05, .5, 1.0):
        now[0] = instant
        node.on_mode(String(data=mode))
        assert_stop(node)
    node.publisher.publish.reset_mock()
    node.on_ready(String(data=mode))
    node.publisher.publish.assert_not_called()
    now[0] += .3
    node.publish_stop()
    assert_stop(node)
    node.on_ready(String(data=mode))
    node.on_mode(String(data='LANE'))
    node.publisher.publish.reset_mock()
    node.publish_stop()
    node.publisher.publish.assert_not_called()
    node.on_mode(String(data=mode))
    assert_stop(node)  # Readiness cannot survive mode re-entry.
    node.on_ready(String(data='bad'))
    assert_stop(node)


def test_gps_requires_controller_ready_and_fresh_health(env):
    node, now, *_ = env

    node.on_mode(String(data='GPS'))
    assert_stop(node)

    # Controller heartbeat alone must not release GPS mode.
    node.on_ready(String(data='GPS'))
    assert_stop(node)

    # Fresh healthy GPS + fresh controller heartbeat releases STOP.
    node.publisher.publish.reset_mock()
    node.on_gps_health(Bool(data=True))
    node.publisher.publish.assert_not_called()

    # Explicit unhealthy GPS must fail closed immediately.
    node.on_gps_health(Bool(data=False))
    assert_stop(node)

    # Recover both heartbeats.
    node.publisher.publish.reset_mock()
    node.on_gps_health(Bool(data=True))
    node.on_ready(String(data='GPS'))
    node.publisher.publish.assert_not_called()

    # GPS health is a heartbeat too; stale health must fail closed.
    now[0] = 1.5
    node.on_mode(String(data='GPS'))
    node.on_ready(String(data='GPS'))
    node.publisher.publish.reset_mock()
    node.publish_stop()
    assert_stop(node)


def test_finish_never_released_by_readiness(env):
    node, now, *_ = env
    node.on_mode(String(data='FINISH'))
    for step in range(100):
        now[0] = step * .05
        node.on_ready(String(data='OBSTACLE'))
        node.publish_stop()
        assert_stop(node)


def test_finish_latches_against_late_lane_mode(env):
    node, now, *_ = env
    node.on_mode(String(data='FINISH'))
    for mode in ('LANE', 'GPS', 'STOP', 'OBSTACLE', 'REVERSE'):
        now[0] += .5
        node.on_mode(String(data=mode))
        assert node.mode == 'FINISH'
        assert_stop(node)
