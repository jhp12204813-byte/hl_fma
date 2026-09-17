import json
import math
from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest
from builtin_interfaces.msg import Time
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool, Float64, String

from fma_localization import gps_controller_node as gps


def test_bearing_and_wrap():
    assert gps.bearing_deg(0., 0., 1., 0.) == pytest.approx(0.)
    assert gps.bearing_deg(0., 0., 0., 1.) == pytest.approx(90.)
    assert gps.wrap_degrees(181.) == pytest.approx(-179.)
    assert gps.wrap_degrees(-181.) == pytest.approx(179.)


def test_target_parser_rejects_null_coordinate():
    valid = json.dumps({
        'number': 1,
        'id': 'ROUTE_001',
        'latitude': 37.0,
        'longitude': 127.0,
        'activation_radius_m': 3.0,
        'hard_point': True,
    })
    assert gps.parse_target(valid).number == 1

    bad = json.dumps({
        'number': 57,
        'id': 'ROUTE_057',
        'latitude': None,
        'longitude': None,
        'activation_radius_m': 3.0,
        'hard_point': True,
    })
    assert gps.parse_target(bad) is None


def test_right_target_produces_negative_steering():
    config = gps.ControlConfig(enabled=True)
    fix = gps.FixSample(0., 0., 0.)
    heading = gps.HeadingSample(0., 0.)
    target = gps.TargetWaypoint(1, 'WP', 0., 1., 3., True)

    speed, steering = gps.gps_command(fix, heading, target, config)

    assert speed == pytest.approx(.2)
    assert steering == pytest.approx(-.3054)


@pytest.fixture
def env(monkeypatch):
    command_publisher = MagicMock()
    ready_publisher = MagicMock()
    subscriber = MagicMock()
    timer = MagicMock()
    clock = MagicMock()
    clock.now.return_value.to_msg.return_value = Time(sec=123)

    now = [10.0]
    overrides = {}

    monkeypatch.setattr(gps.Node, '__init__', lambda self, name: None)

    def declare(self, name, value, descriptor):
        return SimpleNamespace(value=overrides.get(name, value))

    def create_publisher(self, msg_type, topic, qos):
        if topic == '/cmd/gps':
            return command_publisher
        if topic == '/mission/controller_ready':
            return ready_publisher
        raise AssertionError(topic)

    monkeypatch.setattr(gps.Node, 'declare_parameter', declare)
    monkeypatch.setattr(gps.Node, 'create_publisher', create_publisher)
    monkeypatch.setattr(gps.Node, 'create_subscription', subscriber)
    monkeypatch.setattr(gps.Node, 'create_timer', timer)
    monkeypatch.setattr(gps.Node, 'get_clock', lambda self: clock)
    monkeypatch.setattr(gps.time, 'monotonic', lambda: now[0])

    overrides['enabled'] = True
    node = gps.GpsControllerNode()

    return SimpleNamespace(
        node=node,
        now=now,
        command=command_publisher,
        ready=ready_publisher,
        subscriber=subscriber,
    )


def target_message():
    return String(data=json.dumps({
        'number': 1,
        'id': 'ROUTE_001',
        'latitude': 0.0,
        'longitude': 1.0,
        'activation_radius_m': 3.0,
        'hard_point': True,
    }))


def fix_message():
    msg = NavSatFix()
    msg.status.status = 0
    msg.latitude = 0.0
    msg.longitude = 0.0
    return msg


def make_ready(node):
    node.on_fix(fix_message())
    node.on_heading(Float64(data=0.0))
    node.on_health(Bool(data=True))
    node.on_target(target_message())


def test_requires_gps_mode_and_all_inputs(env):
    node = env.node
    make_ready(node)

    node.on_mode(String(data='LANE'))
    node.publish_command()
    env.command.assert_not_called()
    env.ready.assert_not_called()

    node.on_mode(String(data='GPS'))
    node.publish_command()

    msg = env.command.publish.call_args.args[0]
    assert msg.speed_mps == pytest.approx(.2)
    assert msg.steering_angle_rad == pytest.approx(-.3054)
    assert not msg.emergency_stop
    assert not msg.pwm_control
    assert msg.drive_pwm == 0
    assert env.ready.publish.call_args.args[0].data == 'GPS'


def test_unhealthy_or_stale_fix_revokes_output(env):
    node = env.node
    make_ready(node)
    node.on_mode(String(data='GPS'))

    node.publish_command()
    assert env.command.publish.called

    env.command.publish.reset_mock()
    env.ready.publish.reset_mock()

    node.on_health(Bool(data=False))
    node.publish_command()
    env.command.publish.assert_not_called()
    env.ready.publish.assert_not_called()

    node.on_health(Bool(data=True))
    node.on_heading(Float64(data=0.0))
    node.on_mode(String(data='GPS'))

    env.now[0] = 11.5
    node.publish_command()

    env.command.publish.assert_not_called()
    env.ready.publish.assert_not_called()
