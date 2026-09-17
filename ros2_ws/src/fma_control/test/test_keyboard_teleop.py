"""Keyboard state, ROS mocks and terminal cleanup; no graph or serial access."""
from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest
from builtin_interfaces.msg import Time
from fma_interfaces.msg import DriveCommand, VehicleFeedback
from rclpy.clock import ClockType
from fma_control import keyboard_teleop_node as teleop


@pytest.mark.parametrize('key,drive,speed,quit_requested', [
    ('w', 'FORWARD', .2, False), ('W', 'FORWARD', .2, False),
    ('s', 'REVERSE', -.2, False), (' ', 'STOP', 0., False),
    ('x', 'STOP', 0., False), ('q', 'STOP', 0., True),
    ('\x03', 'STOP', 0., True)])
def test_drive_keys(key, drive, speed, quit_requested):
    state = teleop.TeleopState()
    assert (state.drive, state.steering, state.speed_mps, state.pwm_percent) == ('STOP', 0., 0., 0)
    state.handle_key(key)
    assert (state.drive, state.speed_mps, state.quit_requested) == (drive, speed, quit_requested)


def test_steering_step_clamps_center_and_stop():
    state = teleop.TeleopState()
    for key in 'aaaD':
        state.handle_key(key)
    assert state.steering == pytest.approx(.1)
    for _ in range(20):
        state.handle_key('a')
    assert state.steering == .2810
    for _ in range(20):
        state.handle_key('d')
    assert state.steering == -.3054
    state.handle_key('x')
    assert state.steering == -.3054
    state.handle_key('C')
    assert state.steering == 0.
    state.handle_key('q')
    state.handle_key('w')
    assert state.drive == 'STOP'


@pytest.mark.parametrize('speed', [1., -1.])
def test_speed_display(speed):
    assert teleop.speed_display(speed) == '3.60 km/h'


@pytest.fixture
def env(monkeypatch):
    publisher, subscriber, timer, clock = [MagicMock() for _ in range(4)]
    clock.now.return_value.to_msg.return_value = Time(sec=123)
    monkeypatch.setattr(teleop.Node, '__init__', lambda self, name: None)
    monkeypatch.setattr(teleop.Node, 'declare_parameter',
                        lambda self, name, value, descriptor: SimpleNamespace(value=value))
    monkeypatch.setattr(teleop.Node, 'create_publisher', publisher)
    monkeypatch.setattr(teleop.Node, 'create_subscription', subscriber)
    monkeypatch.setattr(teleop.Node, 'create_timer', timer)
    monkeypatch.setattr(teleop.Node, 'get_clock', lambda self: clock)
    monkeypatch.setattr(teleop.time, 'sleep', lambda seconds: None)
    node = teleop.KeyboardTeleopNode()
    return SimpleNamespace(node=node, publisher=publisher, subscriber=subscriber, timer=timer)


def test_topics_timer_commands_headers(env):
    node = env.node
    assert (node.state.pwm_step, node.state.pwm_max) == (5, 50)
    assert env.publisher.call_args.args == (DriveCommand, '/cmd/manual', 1)
    assert env.subscriber.call_args.args[:2] == (VehicleFeedback, '/vehicle/feedback')
    assert env.timer.call_args.args == (.1, node.publish_command)
    assert env.timer.call_args.kwargs['clock'].clock_type == ClockType.STEADY_TIME
    node.publish_command()
    startup = node.publisher.publish.call_args.args[0]
    assert startup.speed_mps == 0. and startup.steering_angle_rad == 0.
    assert not startup.emergency_stop
    for key, speed in [('w', .2), ('s', 0.), ('s', -.2), ('x', 0.)]:
        node.handle_key(key)
        msg = node.publisher.publish.call_args.args[0]
        assert msg.speed_mps == pytest.approx(speed)
        assert not msg.emergency_stop
        assert msg.header.stamp == Time(sec=123)
        assert msg.header.frame_id == ''
    node.handle_key('a')
    node.publish_command()
    msg = node.publisher.publish.call_args.args[0]
    assert msg.speed_mps == 0. and msg.steering_angle_rad == pytest.approx(.05)


def test_feedback_ui(env, monkeypatch):
    node = env.node
    node.handle_key('w')
    assert 'DRIVE TARGET : FORWARD' in node.screen()
    assert 'PWM   : 5 %' in node.screen()
    assert 'SPEED : -- km/h' in node.screen()
    assert 'ADC   : --' in node.screen()
    msg = VehicleFeedback(speed_mps=-1., steering_adc=2780, drive_state=2)
    monkeypatch.setattr(teleop.time, 'monotonic', lambda: 10.)
    node.on_feedback(msg)
    assert 'SPEED : 3.60 km/h' in node.screen()
    assert 'ADC   : 2780' in node.screen()
    assert 'DRIVE FEEDBACK : REVERSE' in node.screen()
    monkeypatch.setattr(teleop.time, 'monotonic', lambda: 12.)
    assert 'STALE' in node.screen()


def test_shutdown_repeats_stop_despite_publish_failure(env):
    node = env.node
    node.handle_key('w')
    node.publisher.publish.reset_mock()
    node.publisher.publish.side_effect = [RuntimeError('publisher failed'), None, None]
    node.stop_repeatedly()
    assert node.publisher.publish.call_count == 3
    for call in node.publisher.publish.call_args_list:
        assert call.args[0].use_pwm_override
        assert call.args[0].drive_pwm_percent == 0
        assert call.args[0].speed_mps == 0.
        assert not call.args[0].emergency_stop
    node.timer.cancel.assert_called_once()


@pytest.mark.parametrize('failure', [None, RuntimeError('spin error'), KeyboardInterrupt()])
def test_terminal_restoration_and_stop(env, monkeypatch, failure):
    node = env.node
    stream = MagicMock()
    stream.fileno.return_value = 42
    saved = [1, 2, 3]
    monkeypatch.setattr(teleop.termios, 'tcgetattr', lambda fd: saved)
    restore = MagicMock()
    monkeypatch.setattr(teleop.termios, 'tcsetattr', restore)
    monkeypatch.setattr(teleop.tty, 'setraw', MagicMock())
    monkeypatch.setattr(teleop.rclpy, 'ok', lambda: True)
    monkeypatch.setattr(teleop.select, 'select', lambda *args: ([42], [], []))
    monkeypatch.setattr(teleop.os, 'read', lambda *args: b'q' if failure is None else b'w')
    monkeypatch.setattr(teleop.rclpy, 'spin_once', MagicMock(side_effect=failure))
    if failure is None:
        teleop.run_terminal(node, stream)
    else:
        with pytest.raises(type(failure)):
            teleop.run_terminal(node, stream)
    restore.assert_called_once_with(42, teleop.termios.TCSANOW, saved)
    assert node.state.drive == 'STOP'
    for call in node.publisher.publish.call_args_list[-3:]:
        assert call.args[0].speed_mps == 0.
        assert call.args[0].use_pwm_override and call.args[0].drive_pwm_percent == 0


def test_terminal_restore_error_happens_after_stops(env, monkeypatch):
    node = env.node
    monkeypatch.setattr(teleop.termios, 'tcgetattr', lambda fd: [1])
    monkeypatch.setattr(teleop.tty, 'setraw', MagicMock())
    monkeypatch.setattr(teleop.rclpy, 'ok', lambda: False)
    monkeypatch.setattr(teleop.termios, 'tcsetattr', MagicMock(side_effect=OSError('restore')))
    with pytest.raises(OSError):
        teleop.run_terminal(node, MagicMock())
    assert node.publisher.publish.call_count == 3


def test_non_tty_exits_before_ros_init(monkeypatch, capsys):
    monkeypatch.setattr(teleop.sys.stdin, 'isatty', lambda: False)
    init = MagicMock()
    monkeypatch.setattr(teleop.rclpy, 'init', init)
    assert teleop.main() == 1
    init.assert_not_called()
    assert 'TTY' in capsys.readouterr().err


@pytest.mark.parametrize('key,opposite,drive', [('w', 's', 'FORWARD'), ('s', 'w', 'REVERSE')])
def test_pwm_steps_clamp(key, opposite, drive):
    state = teleop.TeleopState()
    assert (state.drive, state.pwm_percent) == ('STOP', 0)
    for expected in (*range(5, 51, 5), 50, 50):
        state.handle_key(key)
        assert (state.drive, state.pwm_percent) == (drive, expected)


@pytest.mark.parametrize('key,opposite,drive,next_drive,speed', [
    ('w', 's', 'FORWARD', 'REVERSE', .2),
    ('s', 'w', 'REVERSE', 'FORWARD', -.2)])
@pytest.mark.parametrize('start', [20, 30, 50])
def test_pwm_braking_before_direction_change(env, key, opposite, drive, next_drive, speed, start):
    node = env.node
    for _ in range(start // 5):
        node.handle_key(key)
    assert (node.state.drive, node.state.pwm_percent) == (drive, start)
    for expected in range(start - 5, -1, -5):
        node.handle_key(opposite)
        assert (node.state.drive, node.state.pwm_percent) == (
            drive if expected else 'STOP', expected)
        msg = node.publisher.publish.call_args.args[0]
        assert msg.use_pwm_override and msg.drive_pwm_percent == expected
        assert msg.speed_mps == pytest.approx(speed if expected else 0.)
    node.handle_key(opposite)
    assert (node.state.drive, node.state.pwm_percent) == (next_drive, 5)
    msg = node.publisher.publish.call_args.args[0]
    assert msg.use_pwm_override and msg.drive_pwm_percent == 5
    assert msg.speed_mps == pytest.approx(-speed)


@pytest.mark.parametrize('key', [' ', 'x', 'q', '\x03'])
@pytest.mark.parametrize('drive_key', ['w', 's'])
def test_pwm_stop_keys(env, key, drive_key):
    node = env.node
    for _ in range(10):
        node.handle_key(drive_key)
    node.handle_key(key)
    msg = node.publisher.publish.call_args.args[0]
    assert node.state.drive == 'STOP'
    assert msg.speed_mps == 0. and msg.use_pwm_override and msg.drive_pwm_percent == 0
    assert 'PWM   : 0 %' in node.screen()


@pytest.mark.parametrize('kwargs', [{'pwm_max': -1}, {'pwm_max': 51}, {'pwm_max': 100},
                                    {'pwm_max': 101}, {'pwm_max': 20.5},
                                    {'pwm_step': 0}, {'pwm_step': 101}, {'pwm_step': True}])
def test_invalid_pwm_parameters(kwargs):
    with pytest.raises(ValueError):
        teleop.TeleopState(**kwargs)


@pytest.mark.parametrize('maximum', [0, 13, 50])
def test_explicit_pwm_max(maximum):
    state = teleop.TeleopState(pwm_max=maximum)
    for _ in range(30):
        state.handle_key('w')
    assert state.pwm_percent == maximum
    assert state.drive == ('FORWARD' if maximum else 'STOP')
    for _ in range((maximum + state.pwm_step - 1) // state.pwm_step):
        state.handle_key('s')
        assert 0 <= state.pwm_percent <= maximum
    assert (state.drive, state.pwm_percent) == ('STOP', 0)


def test_pwm_publication_and_stopped_steering(env):
    node = env.node
    for key, expected in [('a', 0), ('d', 0), ('c', 0), ('w', 5), ('w', 10), ('w', 15)]:
        node.handle_key(key)
        msg = node.publisher.publish.call_args.args[0]
        assert msg.use_pwm_override and msg.drive_pwm_percent == expected
        if expected == 0:
            assert msg.speed_mps == 0.
    assert 'PWM   : 15 %' in node.screen()
