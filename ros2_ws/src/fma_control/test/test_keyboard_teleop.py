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
    assert (state.drive, state.steering, state.speed_mps) == ('STOP', 0., 0.)
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
    monkeypatch.setattr(teleop.Node, 'create_publisher', publisher)
    monkeypatch.setattr(teleop.Node, 'create_subscription', subscriber)
    monkeypatch.setattr(teleop.Node, 'create_timer', timer)
    monkeypatch.setattr(teleop.Node, 'get_clock', lambda self: clock)
    monkeypatch.setattr(teleop.time, 'sleep', lambda seconds: None)
    node = teleop.KeyboardTeleopNode()
    return SimpleNamespace(node=node, publisher=publisher, subscriber=subscriber, timer=timer)


def test_topics_timer_commands_headers(env):
    node = env.node
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
        assert call.args[0].speed_mps == 0.
        assert call.args[0].drive_pwm == 0
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
    assert all(c.args[0].speed_mps == 0. for c in node.publisher.publish.call_args_list[-3:])


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


@pytest.mark.parametrize('up,down,direction', [('w', 's', 'FORWARD'), ('s', 'w', 'REVERSE')])
def test_pwm_full_range_and_zero_before_reversal(up, down, direction):
    state = teleop.TeleopState()
    assert state.throttle_percent == 0
    for step in range(1, 181):
        state.handle_key(up)
        assert state.throttle_percent == min(step * 5, 100)
        assert state.drive == direction
    assert state.throttle_percent == 100 > 50
    while state.throttle_percent:
        previous = state.throttle_percent
        state.handle_key(down)
        assert state.throttle_percent == max(0, previous - 5)
        assert state.drive == (direction if state.throttle_percent else 'STOP')
    state.handle_key(down)
    assert state.throttle_percent == 5 and state.drive != direction
    state.handle_key(up)
    assert state.throttle_percent == 0 and state.drive == 'STOP'
    state.handle_key(up)
    assert state.throttle_percent == 5 and state.drive == direction


@pytest.mark.parametrize('key', [' ', 'x', 'q', '\x03'])
def test_pwm_stop_is_immediate(env, key):
    node = env.node
    for _ in range(20):
        node.handle_key('w')
    node.handle_key(key)
    msg = node.publisher.publish.call_args.args[0]
    assert node.state.throttle_percent == msg.drive_pwm == 0
    assert node.state.drive == 'STOP' and msg.speed_mps == 0
    assert msg.pwm_control
    if key in ('q', '\x03'):
        assert node.state.quit_requested


def test_pwm_message_and_random_bounds(env):
    import random
    rng = random.Random(7)
    for _ in range(2000):
        env.node.handle_key(rng.choice('wwssaxdc '))
        msg = env.node.publisher.publish.call_args.args[0]
        assert 0 <= msg.drive_pwm <= 799
        assert 0 <= env.node.state.throttle_percent <= 100
        assert msg.drive_pwm == min(799, env.node.state.throttle_percent * 8)
        assert msg.pwm_control
        assert (msg.speed_mps == 0) == (msg.drive_pwm == 0)


@pytest.mark.parametrize('percent,ccr', [(-5, 0), (0, 0), (5, 40), (10, 80),
                                       (15, 120), (20, 160), (50, 400),
                                       (100, 799), (105, 799)])
def test_percentage_mapping(percent, ccr):
    assert teleop.throttle_to_ccr(percent) == ccr


@pytest.mark.parametrize('key,prefix', [('w', b'F'), ('s', b'B')])
@pytest.mark.parametrize('percent,ccr', [(0, 0), (5, 40), (10, 80), (15, 120),
                                       (50, 400), (100, 799)])
def test_percentage_scaled_once_through_ros_to_serial(env, monkeypatch, key, prefix, percent, ccr):
    """Run real callbacks at every layer, replacing only ROS transport/serial."""
    from fma_control.command_arbiter_node import CommandArbiterNode
    from fma_vehicle.vehicle_controller_node import VehicleControllerNode
    from fma_vehicle import stm32_bridge_node as bridge

    now = [100.]
    monkeypatch.setattr(teleop.time, 'monotonic', lambda: now[0])
    monkeypatch.setattr(teleop.Node, 'declare_parameter',
                        lambda self, name, value, descriptor: SimpleNamespace(value=value))
    monkeypatch.setattr(teleop.Node, 'get_logger', lambda self: MagicMock())
    port = MagicMock()
    port.write.side_effect = len
    monkeypatch.setattr(bridge.serial, 'Serial', MagicMock(return_value=port))
    arbiter = CommandArbiterNode()
    controller = VehicleControllerNode()
    stm32 = bridge.STM32BridgeNode()
    for node in (env.node, arbiter, controller, stm32):
        node.publisher = MagicMock()
    env.node.publisher.publish.side_effect = arbiter.on_manual
    arbiter.publisher.publish.side_effect = controller.on_command
    controller.publisher.publish.side_effect = stm32.on_command

    if percent == 0:
        env.node.publish_command()
        arbiter.publish_output()
    else:
        for _ in range(percent // 5):
            now[0] += .1
            env.node.handle_key(key)
            arbiter.publish_output()
    assert env.node.state.throttle_percent == percent
    for node in (env.node, arbiter, controller):
        msg = node.publisher.publish.call_args.args[0]
        assert msg.pwm_control and msg.drive_pwm == ccr
    expected = prefix + f'{ccr:04d}'.encode('ascii') if ccr else b'X'
    assert port.write.call_args.args[0] == expected
    assert f'THROTTLE TARGET : {percent}%' in env.node.screen()

    # Repeated publication cannot scale already-converted counts again.
    for _ in range(3):
        now[0] += .21
        env.node.publish_command()
        arbiter.publish_output()
        assert port.write.call_args.args[0] == expected
        assert env.node.state.throttle_percent == percent
    now[0] += .5
    arbiter.publish_output()
    assert port.write.call_args.args[0] == b'X'
    assert controller.publisher.publish.call_args.args[0].drive_pwm == 0
