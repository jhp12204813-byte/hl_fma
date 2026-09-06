"""Pure selection and real node callbacks, without DDS or hardware."""
import math
from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest
from builtin_interfaces.msg import Time
from fma_interfaces.msg import DriveCommand
from rclpy.clock import ClockType

from fma_control import command_arbiter_node as arbiter
from fma_control.command_arbiter_node import Candidate, candidate_is_valid, select_source


@pytest.mark.parametrize('lane,mission,latched,expected', [
    (None, None, False, None),
    (Candidate(1., .1, False, 10.), None, False, 'lane'),
    (Candidate(1., .1, False, 10.), Candidate(2., .2, False, 10.), False, 'mission'),
    (Candidate(1., .1, False, 10.), Candidate(2., .2, False, 10.), True, 'emergency'),
    (Candidate(1., .1, False, 10.), Candidate(2., .2, False, 9.5), False, 'lane'),
    (Candidate(1., .1, False, 9.5), Candidate(2., .2, False, 9.5), False, None),
    (Candidate(math.nan, .1, False, 10.), None, False, None),
    (Candidate(1., .1, False, 10.), Candidate(math.inf, .2, False, 10.), False, 'lane'),
])
def test_selection(lane, mission, latched, expected):
    assert select_source(lane, mission, latched, 10., .5, .5) == expected


@pytest.mark.parametrize('speed,angle', [(math.nan, 0.), (math.inf, 0.),
                                       (-math.inf, 0.), (0., math.nan),
                                       (0., math.inf), (0., -math.inf)])
def test_invalid_numeric_fields(speed, angle):
    assert not candidate_is_valid(Candidate(speed, angle, False, 10.))


@pytest.fixture
def env(monkeypatch):
    publisher, subscriber, timer, logger, clock = [MagicMock() for _ in range(5)]
    clock.now.return_value.to_msg.return_value = Time(sec=123, nanosec=456)
    now, overrides, descriptors = [10.0], {}, {}
    names = []
    monkeypatch.setattr(arbiter.Node, '__init__', lambda self, name: names.append(name))

    def declare(self, name, value, descriptor):
        descriptors[name] = descriptor
        return SimpleNamespace(value=overrides.get(name, value))

    monkeypatch.setattr(arbiter.Node, 'declare_parameter', declare)
    monkeypatch.setattr(arbiter.Node, 'create_publisher', publisher)
    monkeypatch.setattr(arbiter.Node, 'create_subscription', subscriber)
    monkeypatch.setattr(arbiter.Node, 'create_timer', timer)
    monkeypatch.setattr(arbiter.Node, 'get_logger', lambda self: logger)
    monkeypatch.setattr(arbiter.Node, 'get_clock', lambda self: clock)
    monkeypatch.setattr(arbiter.time, 'monotonic', lambda: now[0])

    def make(**parameters):
        overrides.update(parameters)
        return arbiter.CommandArbiterNode()

    return SimpleNamespace(make=make, now=now, publisher=publisher, subscriber=subscriber,
                           timer=timer, logger=logger, clock=clock, names=names,
                           descriptors=descriptors)


def command(speed=1., angle=.1, emergency=False, stamp=999):
    msg = DriveCommand()
    msg.speed_mps, msg.steering_angle_rad, msg.emergency_stop = speed, angle, emergency
    msg.header.stamp.sec = stamp
    msg.header.frame_id = 'sender'
    return msg


def output(node):
    node.publish_output()
    return node.publisher.publish.call_args.args[0]


def values(msg):
    return msg.speed_mps, msg.steering_angle_rad, msg.emergency_stop


def assert_stop(msg):
    assert values(msg) == (0., 0., True)


def test_node_contract_and_timer(env):
    node = env.make()
    assert env.names == ['command_arbiter']
    assert env.publisher.call_args.args == (DriveCommand, '/cmd/final', 1)
    assert [(c.args[0], c.args[1], c.args[3]) for c in env.subscriber.call_args_list] == [
        (DriveCommand, '/cmd/lane', 1), (DriveCommand, '/cmd/mission', 1),
        (DriveCommand, '/cmd/emergency', 1)]
    assert env.timer.call_args.args == (.05, node.publish_output)
    assert env.timer.call_args.kwargs['clock'].clock_type == ClockType.STEADY_TIME
    assert all(d.read_only for d in env.descriptors.values())
    assert not node.emergency_latched
    assert_stop(output(node))
    node.on_lane(command())
    for _ in range(3):
        env.timer.call_args.args[1]()
    assert node.publisher.publish.call_count == 4
    msg = node.publisher.publish.call_args.args[0]
    assert values(msg) == values(command())
    assert msg.header.stamp == Time(sec=123, nanosec=456)
    assert msg.header.frame_id == ''


def test_parameter_overrides(env):
    node = env.make(lane_topic='/test/lane', mission_topic='/test/mission',
                    emergency_topic='/test/emergency', final_topic='/test/final',
                    output_rate_hz=10., lane_timeout_sec=.25, mission_timeout_sec=1.)
    assert [c.args[1] for c in env.subscriber.call_args_list] == [
        '/test/lane', '/test/mission', '/test/emergency']
    assert env.publisher.call_args.args[1] == '/test/final'
    assert env.timer.call_args.args[0] == .1
    node.on_lane(command())
    env.now[0] = 10.25
    assert_stop(output(node))
    node.on_mission(command(2.))
    env.now[0] = 11.
    assert output(node).speed_mps == 2.
    env.now[0] = 11.25
    assert_stop(output(node))


@pytest.mark.parametrize('name', ['lane_timeout_sec', 'mission_timeout_sec',
                                  'emergency_timeout_sec', 'output_rate_hz'])
@pytest.mark.parametrize('value', [0., -1., math.nan, math.inf])
def test_invalid_parameters(env, name, value):
    with pytest.raises(ValueError):
        env.make(**{name: value})
    env.timer.assert_not_called()


def test_priority_latch_stale_and_explicit_release(env):
    node = env.make()
    node.on_lane(command())
    assert output(node).speed_mps == 1.
    node.on_mission(command(2.))
    assert output(node).speed_mps == 2.
    node.on_emergency(command(math.nan, math.nan, True))
    assert node.emergency_latched
    assert_stop(output(node))
    env.now[0] = 20.
    node.on_lane(command())
    node.on_mission(command(3.))
    assert_stop(output(node))
    assert node.emergency_latched
    assert env.logger.warning.call_count == 1
    assert_stop(output(node))
    assert env.logger.warning.call_count == 1
    node.on_emergency(command(math.nan, math.inf, False, stamp=0))
    assert not node.emergency_latched
    assert output(node).speed_mps == 3.
    # Emergency false never becomes a motion candidate itself.
    env.now[0] = 20.5
    assert_stop(output(node))


def test_freshness_fallback_ignores_sender_and_output_clocks(env):
    node = env.make()
    node.on_mission(command(2., stamp=0))
    env.now[0] = 10.25
    node.on_lane(command(1., stamp=2000000000))
    env.now[0] = 10.499
    assert output(node).speed_mps == 2.
    # ROS time jumps backwards, but local monotonic freshness still expires.
    env.clock.now.return_value.to_msg.return_value = Time(sec=0)
    env.now[0] = 10.5
    assert output(node).speed_mps == 1.
    env.now[0] = 10.75
    assert_stop(output(node))


@pytest.mark.parametrize('source', ['lane', 'mission'])
def test_invalid_replaces_previous_and_warnings_throttled(env, source):
    node = env.make()
    callback = getattr(node, 'on_' + source)
    callback(command())
    assert output(node).speed_mps == 1.
    for bad in (math.nan, math.inf, -math.inf):
        callback(command(bad))
        assert_stop(output(node))
    assert env.logger.warning.call_count == 1
    env.now[0] = 15.
    callback(command(angle=math.nan))
    assert env.logger.warning.call_count == 2
    callback(command(2.))
    assert output(node).speed_mps == 2.


def test_invalid_mission_falls_back_to_lane(env):
    node = env.make()
    node.on_lane(command())
    node.on_mission(command(2.))
    node.on_mission(command(math.inf))
    assert output(node).speed_mps == 1.


@pytest.mark.parametrize('source', ['lane', 'mission'])
def test_candidate_emergency_flag_preserved_without_latching(env, source):
    node = env.make()
    getattr(node, 'on_' + source)(command(-1., -.2, True))
    assert values(output(node)) == values(command(-1., -.2, True))
    assert not node.emergency_latched


def test_candidates_are_snapshots(env):
    node = env.make()
    msg = command()
    node.on_lane(msg)
    msg.speed_mps = 99.
    first = output(node)
    first.speed_mps = 88.
    assert output(node).speed_mps == 1.


def test_emergency_timeout_diagnostics_never_release(env):
    node = env.make(emergency_timeout_sec=.25)
    node.on_emergency(command(emergency=True))
    env.now[0] = 10.249
    assert_stop(output(node))
    env.logger.warning.assert_not_called()
    env.now[0] = 10.25
    assert_stop(output(node))
    env.logger.warning.assert_called_once()
    assert node.emergency_latched
