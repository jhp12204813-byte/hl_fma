"""Real bridge callbacks with mocked ROS transport and serial; no DDS/hardware."""
import math
from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest
from builtin_interfaces.msg import Time
from fma_interfaces.msg import VehicleCommand, VehicleFeedback
from rclpy.clock import ClockType
from fma_vehicle import stm32_bridge_node as bridge


@pytest.mark.parametrize('speed', [456, -456, 0])
def test_parser(speed):
    assert bridge.parse_telemetry(f'ENC=-123 SPEED={speed}mm/s STEER=2182 DRIVE=1') == (
        -123, speed / 1000., 2182, 1)


@pytest.mark.parametrize('line', [
    'FORWARD', 'ENC=1 SPEED=2 STEER=3 DRIVE=1',
    'ENC=1 SPEED=2mm/s STEER=65536 DRIVE=1',
    'ENC=1 SPEED=2mm/s STEER=3 DRIVE=3',
    'ENC=2147483648 SPEED=2mm/s STEER=3 DRIVE=1',
    'ENC=1 SPEED=-2147483649mm/s STEER=3 DRIVE=1',
    'ENC=1 SPEED=2mm/s STEER=3 DRIVE=1 extra',
    ' ENC=1 SPEED=2mm/s STEER=3 DRIVE=1'])
def test_parser_rejects(line):
    with pytest.raises(ValueError):
        bridge.parse_telemetry(line)


@pytest.fixture
def env(monkeypatch):
    port = MagicMock()
    port.write.side_effect = len
    port.in_waiting = 0
    port.read.return_value = b''
    serial_factory = MagicMock(return_value=port)
    publisher, subscriber, timer, logger, clock = [MagicMock() for _ in range(5)]
    clock.now.return_value.to_msg.return_value = Time(sec=123, nanosec=456)
    now, overrides, nodes = [100.0], {}, []
    monkeypatch.setattr(bridge.Node, '__init__', lambda self, name: None)
    monkeypatch.setattr(bridge.Node, 'destroy_node', lambda self: None)
    monkeypatch.setattr(bridge.Node, 'declare_parameter',
                        lambda self, name, value, descriptor:
                        SimpleNamespace(value=overrides.get(name, value)))
    monkeypatch.setattr(bridge.Node, 'create_publisher', publisher)
    monkeypatch.setattr(bridge.Node, 'create_subscription', subscriber)
    monkeypatch.setattr(bridge.Node, 'create_timer', timer)
    monkeypatch.setattr(bridge.Node, 'get_logger', lambda self: logger)
    monkeypatch.setattr(bridge.Node, 'get_clock', lambda self: clock)
    monkeypatch.setattr(bridge.serial, 'Serial', serial_factory)
    monkeypatch.setattr(bridge.time, 'monotonic', lambda: now[0])
    monkeypatch.setattr(bridge.time, 'sleep', lambda seconds: None)

    def make(**parameters):
        overrides.update(parameters)
        node = bridge.STM32BridgeNode()
        nodes.append(node)
        return node

    yield SimpleNamespace(make=make, port=port, serial=serial_factory, now=now,
                          publisher=publisher, subscriber=subscriber,
                          timer=timer, logger=logger)
    for node in nodes:
        node.destroy_node()


def command(state=VehicleCommand.DRIVE_FORWARD, adc=2132, emergency=False):
    msg = VehicleCommand()
    msg.drive_state, msg.steering_adc, msg.emergency_stop = state, adc, emergency
    return msg


def packets(env):
    return [call.args[0] for call in env.port.write.call_args_list]


def advance(env, node, now):
    env.now[0] = now
    node.poll()


def test_topics_and_defaults(env):
    node = env.make()
    assert env.subscriber.call_args.args[:2] == (VehicleCommand, '/vehicle/command')
    assert env.publisher.call_args.args[:2] == (VehicleFeedback, '/vehicle/feedback')
    assert node.config['port'] == '/dev/fma_stm32'
    assert env.serial.call_args.kwargs['port'] == '/dev/fma_stm32'
    assert node.config['vehicle_command_timeout_sec'] == .5
    assert (node.drive_period, node.steering_period) == (.2, .1)
    assert env.timer.call_args.kwargs['clock'].clock_type == ClockType.STEADY_TIME
    for removed in ('command_topic', 'ros_command_timeout_sec', 'speed_deadband_mps',
                    'steering_calibration_enabled', 'steering_right_angle_rad',
                    'steering_left_angle_rad', 'steering_zero_tolerance_rad'):
        assert removed not in node.config


def test_topic_parameters(env):
    env.make(vehicle_command_topic='/test/command', feedback_topic='/test/feedback')
    assert env.subscriber.call_args.args[1] == '/test/command'
    assert env.publisher.call_args.args[1] == '/test/feedback'


@pytest.mark.parametrize('state,packet', [(VehicleCommand.DRIVE_FORWARD, b'W'),
                                         (VehicleCommand.DRIVE_REVERSE, b'S'),
                                         (VehicleCommand.DRIVE_STOP, b'X')])
def test_drive_mapping(env, state, packet):
    node = env.make()
    node.on_command(command(state))
    assert packets(env) == [packet]
    if state == VehicleCommand.DRIVE_STOP:
        for now in (100.03, 100.21, 100.4):
            advance(env, node, now)
        assert set(packets(env)) == {b'X'}


@pytest.mark.parametrize('adc,packet', [(2132, b'T2132'), (150, b'T0150'), (3950, b'T3950')])
def test_steering_and_serialized_refresh(env, adc, packet):
    node = env.make()
    node.on_command(command(adc=adc))
    advance(env, node, 100.01)
    assert packets(env) == [b'W']
    for now in (100.03, 100.14, 100.21):
        advance(env, node, now)
    assert packets(env) == [b'W', packet, packet, b'W']


@pytest.mark.parametrize('state,adc,emergency', [
    (3, 2182, False), (255, 2182, False), (1, 149, False), (1, 3951, False),
    (1, 7, False), (1, 4095, False), (1, 50, False), (1, 4040, False),
    (1, 2182, True), (2, 150, True), (255, 65535, True)])
def test_invalid_and_emergency_stop_only(env, state, adc, emergency):
    node = env.make()
    node.on_command(command())
    env.port.write.reset_mock()
    env.now[0] = 100.01
    node.on_command(command(state, adc, emergency))
    assert node.last_command == 100.0
    for now in (100.03, 100.14, 100.25, 100.5):
        advance(env, node, now)
    assert packets(env) and set(packets(env)) == {b'X'}
    assert node.steering is None


def test_startup_timeout_and_recovery(env):
    node = env.make()
    node.poll()
    assert packets(env) == [b'X']
    env.now[0] = 100.21
    node.on_command(command())
    assert packets(env)[-1] == b'W'
    advance(env, node, 100.709)
    assert packets(env)[-1] != b'X'
    advance(env, node, 100.74)
    assert packets(env)[-1] == b'X'
    env.port.write.reset_mock()
    advance(env, node, 101.)
    assert packets(env) == [b'X']
    env.now[0] = 101.21
    node.on_command(command(VehicleCommand.DRIVE_REVERSE))
    assert packets(env)[-1] == b'S'


@pytest.mark.parametrize('timeout', [.5, .25])
def test_timeout_boundary(env, timeout):
    node = env.make(vehicle_command_timeout_sec=timeout)
    node.on_command(command())
    advance(env, node, 100. + timeout)
    assert packets(env) == [b'W', b'X']


@pytest.mark.parametrize('timeout', [0., -1., math.nan, math.inf])
def test_invalid_timeout(env, timeout):
    with pytest.raises(ValueError):
        env.make(vehicle_command_timeout_sec=timeout)
    env.serial.assert_not_called()


def test_receive_only_fragmented_rx_and_shutdown(env):
    node = env.make(receive_only=True)
    node.on_command(command())
    env.port.in_waiting = 100
    for raw in (b'FORWARD\r\nENC=-123 SPEED=456mm/s STE', b'ER=2182 DRIVE=1\r'):
        env.port.read.return_value = raw
        node.poll()
        node.publisher.publish.assert_not_called()
    env.port.read.return_value = b'\n'
    node.poll()
    msg = node.publisher.publish.call_args.args[0]
    assert isinstance(msg, VehicleFeedback)
    assert msg.encoder_count == -123
    assert msg.speed_mps == pytest.approx(.456)
    assert (msg.steering_adc, msg.drive_state, msg.header.frame_id) == (2182, 1, '')
    assert msg.header.stamp == Time(sec=123, nanosec=456)
    node.on_command(command(emergency=True))
    advance(env, node, 101.)
    node.destroy_node()
    env.port.write.assert_not_called()


def test_rx_strict_crlf_warning_throttle_and_status(env):
    node = env.make(receive_only=True)
    for raw in (b'ENC=1 SPEED=2mm/s STEER=3 DRIVE=1\n', b'ENC=bad\r\n', b'\xff\r\n'):
        node.process_line(raw)
    assert env.logger.warning.call_count == 1
    node.publisher.publish.assert_not_called()
    node.process_line(b'FORWARD\r\n')
    env.logger.debug.assert_called_once()
    env.now[0] = 105.
    node.process_line(b'ENC=bad\r\n')
    assert env.logger.warning.call_count == 2


def test_oversized_rx_recovers(env):
    node = env.make(receive_only=True)
    env.port.in_waiting = 400
    env.port.read.return_value = b'E' * 300
    node.poll()
    assert node.discard_line
    env.port.read.return_value = b'junk\r\nENC=1 SPEED=-2mm/s STEER=3 DRIVE=2\r\n'
    node.poll()
    assert not node.discard_line
    assert node.publisher.publish.call_args.args[0].speed_mps == pytest.approx(-.002)


@pytest.mark.parametrize('receive_only', [False, True])
def test_disconnect(env, receive_only):
    node = env.make(receive_only=receive_only)
    env.port.read.side_effect = OSError('unplugged')
    node.poll()
    assert not node.connected
    assert node.serial is None
    env.port.close.assert_called_once()
    if receive_only:
        env.port.write.assert_not_called()
    env.port.write.reset_mock()
    node.on_command(command())
    node.poll()
    env.port.write.assert_not_called()


@pytest.mark.parametrize('partial', [False, True])
def test_write_failure(env, partial):
    node = env.make()
    env.port.write.side_effect = (lambda packet: 0) if partial else OSError('unplugged')
    node.on_command(command())
    assert not node.connected
    assert node.serial is None
    env.port.close.assert_called_once()


def test_open_failure(env):
    env.serial.side_effect = OSError('missing')
    node = env.make()
    assert not node.connected
    node.on_command(command())
    node.poll()
    env.port.write.assert_not_called()


@pytest.mark.parametrize('adc', [7, 50, 150, 2132, 3950, 4040, 4095])
def test_telemetry_adc_is_not_limited_to_command_targets(adc):
    # Measured feedback can be outside safe command limits and must remain visible.
    assert bridge.parse_telemetry(f'ENC=0 SPEED=0mm/s STEER={adc} DRIVE=0') == (
        0, 0.0, adc, 0)


def test_explicit_port_override(env):
    node = env.make(port='/some/device')
    assert node.config['port'] == '/some/device'
    assert env.serial.call_args.kwargs['port'] == '/some/device'
