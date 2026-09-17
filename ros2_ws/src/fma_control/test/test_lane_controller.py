"""Pure control and mocked ROS adapter regression without DDS or hardware."""
from dataclasses import replace
import math
from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest
from builtin_interfaces.msg import Time
from fma_interfaces.msg import DriveCommand, Lane
from rclpy.clock import ClockType
from fma_control import lane_controller_node as controller
from fma_control.lane_controller_node import ControlConfig, LaneSample, lane_command


GOOD = LaneSample(True, 0., 0., 0., .9, 10.)
ENABLED = ControlConfig(enabled=True)
STOP = (0., 0., False)


@pytest.mark.parametrize('lateral,heading,steering', [
    (0., 0., 0.), (.1, .05, .15), (-.1, -.05, -.15),
    (10., 10., .2810), (-10., -10., -.3054),
])
def test_forward_sign_and_clamp(lateral, heading, steering):
    sample = replace(GOOD, lateral_error_m=lateral, heading_error_rad=heading)
    speed, angle, emergency = lane_command(sample, 10., ENABLED)
    assert speed == .2 and angle == pytest.approx(steering) and not emergency


@pytest.mark.parametrize('sample,now,config', [
    (None, 10., ENABLED),
    (GOOD, 10., ControlConfig()),
    (replace(GOOD, detected=False), 10., ENABLED),
    (replace(GOOD, confidence=.59), 10., ENABLED),
    (replace(GOOD, confidence=-.1), 10., ENABLED),
    (replace(GOOD, confidence=1.1), 10., ENABLED),
    (GOOD, 10.5, ENABLED),
    (GOOD, 9.9, ENABLED),
])
def test_fail_safe(sample, now, config):
    assert lane_command(sample, now, config) == STOP


@pytest.mark.parametrize('field', [
    'lateral_error_m', 'heading_error_rad', 'curvature', 'confidence', 'received_at'])
@pytest.mark.parametrize('value', [math.nan, math.inf, -math.inf])
def test_nonfinite_stop(field, value):
    assert lane_command(replace(GOOD, **{field: value}), 10., ENABLED) == STOP


@pytest.mark.parametrize('kwargs', [
    {'enabled': 'true'}, {'k_lateral': -1.}, {'k_heading': math.inf},
    {'lane_timeout_sec': 0.}, {'confidence_threshold': 1.1},
    {'forward_speed_mps': -.2}, {'output_rate_hz': 0.},
])
def test_invalid_parameters(kwargs):
    with pytest.raises(ValueError):
        ControlConfig(**kwargs)


def test_gains_threshold_and_custom_timeout():
    config = ControlConfig(enabled=True, k_lateral=2., k_heading=.5,
                           confidence_threshold=.9, lane_timeout_sec=.25)
    sample = replace(GOOD, lateral_error_m=.05, heading_error_rad=.1)
    assert lane_command(sample, 10.249, config)[1] == pytest.approx(.15)
    assert lane_command(sample, 10.25, config) == STOP


@pytest.mark.parametrize('enabled', [False, True])
def test_node_contract_freshness_and_invalid_replacement(monkeypatch, enabled):
    pubs, subs, timers, names, descriptors = [], [], [], [], []
    pub = MagicMock()
    clock = MagicMock()
    clock.now.return_value.to_msg.return_value = Time(sec=123)
    now = [10.]
    monkeypatch.setattr(controller.Node, '__init__', lambda self, name: names.append(name))
    def declare(self, name, value, descriptor):
        descriptors.append(descriptor)
        return SimpleNamespace(value=enabled if name == 'enabled' else value)
    monkeypatch.setattr(controller.Node, 'declare_parameter', declare)
    def publisher(self, *args):
        pubs.append(args)
        return pub
    monkeypatch.setattr(controller.Node, 'create_publisher', publisher)
    monkeypatch.setattr(controller.Node, 'create_subscription',
                        lambda self, *args: subs.append(args))
    def timer(self, *args, **kwargs):
        timers.append((args, kwargs))
    monkeypatch.setattr(controller.Node, 'create_timer', timer)
    monkeypatch.setattr(controller.Node, 'get_clock', lambda self: clock)
    monkeypatch.setattr(controller.time, 'monotonic', lambda: now[0])
    node = controller.LaneControllerNode()
    assert names == ['lane_controller']
    assert pubs == [(DriveCommand, '/cmd/lane', 1)]
    assert subs[0][:2] == (Lane, '/perception/lane')
    assert all(d.read_only for d in descriptors)
    assert timers[0][0][0] == .05
    assert timers[0][1]['clock'].clock_type == ClockType.STEADY_TIME
    def output():
        node.publish_command()
        msg = pub.publish.call_args.args[0]
        assert not msg.emergency_stop
        return msg
    assert output().speed_mps == 0.
    msg = Lane(detected=True, lateral_error_m=.1, confidence=.9)
    msg.header.stamp.sec = 2000000000
    node.on_lane(msg)
    msg.lateral_error_m = -99.  # Receive snapshot is independent of sender mutation.
    result = output()
    assert result.speed_mps == pytest.approx(.2 if enabled else 0.)
    assert result.steering_angle_rad == pytest.approx(.1 if enabled else 0.)
    assert result.header.stamp.sec == 123 and result.header.frame_id == ''
    now[0] = 10.499
    assert output().speed_mps == pytest.approx(.2 if enabled else 0.)
    clock.now.return_value.to_msg.return_value = Time(sec=0)
    now[0] = 10.5
    assert output().speed_mps == 0.
    msg.lateral_error_m = math.nan
    node.on_lane(msg)
    assert output().speed_mps == 0.
    assert output().steering_angle_rad == 0.
