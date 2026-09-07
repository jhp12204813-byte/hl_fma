"""Pure conversion and node callbacks tested without DDS or hardware."""
from dataclasses import replace
import math
from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest
from builtin_interfaces.msg import Time
from fma_interfaces.msg import DriveCommand, VehicleCommand
from rclpy.clock import ClockType

from fma_vehicle import vehicle_controller_node as controller
from fma_vehicle.vehicle_controller_node import (
    ConversionConfig, VehicleControllerNode, speed_to_drive_state, steering_angle_to_adc)


@pytest.mark.parametrize('speed,state', [
    (1.0, 1), (0.2, 1), (0.0, 0), (-0.2, 2), (0.01, 0), (-0.01, 0)])
def test_speed(speed, state):
    assert speed_to_drive_state(speed) == state


@pytest.mark.parametrize('speed', [math.nan, math.inf, -math.inf])
def test_invalid_speed(speed):
    with pytest.raises(ValueError):
        speed_to_drive_state(speed)


@pytest.mark.parametrize('angle', [0.0, 0.001, -0.001])
def test_default_center_tolerance(angle):
    assert steering_angle_to_adc(angle) == 2132


@pytest.mark.parametrize('angle', [0.01, -0.01, math.nan, math.inf, -math.inf])
def test_uncalibrated_reject(angle):
    with pytest.raises(ValueError):
        steering_angle_to_adc(angle, ConversionConfig(steering_calibration_enabled=False))


# Synthetic mathematical fixtures only: NOT measured vehicle calibration.
CALIBRATED = ConversionConfig(steering_calibration_enabled=True,
                               steering_right_angle_rad=-0.2,
                               steering_left_angle_rad=0.6)


@pytest.mark.parametrize('angle,adc', [
    (-0.2, 150), (0.0, 2132), (0.6, 3950), (-0.1, 1141), (0.3, 3041),
    (-1.0, 150), (1.0, 3950)])
def test_calibrated_mapping(angle, adc):
    assert steering_angle_to_adc(angle, CALIBRATED) == adc


@pytest.mark.parametrize('right,left', [
    (math.nan, math.nan), (0.0, 0.6), (0.2, 0.6), (-0.2, -0.6),
    (-math.inf, 0.6), (-0.2, math.inf)])
def test_invalid_calibration(right, left):
    with pytest.raises(ValueError):
        steering_angle_to_adc(0.0, replace(
            CALIBRATED, steering_right_angle_rad=right, steering_left_angle_rad=left))


@pytest.mark.parametrize('changes', [
    {'steering_right_adc': 149}, {'steering_left_adc': 3951},
    {'steering_center_adc': 150}, {'steering_zero_tolerance_rad': math.nan},
    {'speed_deadband_mps': -0.1}])
def test_invalid_configuration(changes):
    with pytest.raises(ValueError):
        controller.validate_config(replace(ConversionConfig(), **changes))


@pytest.fixture
def node_factory(monkeypatch):
    """Replace ROS transport only; execute the real constructor and callbacks."""
    publisher = MagicMock()
    timer = MagicMock()
    subscriber = MagicMock()
    clock = MagicMock()
    clock.now.return_value.to_msg.return_value = Time(sec=123, nanosec=456)
    now = [10.0]
    overrides = {}
    monkeypatch.setattr(controller.Node, '__init__', lambda self, name: None)
    monkeypatch.setattr(controller.Node, 'declare_parameter',
                        lambda self, name, value, descriptor:
                        SimpleNamespace(value=overrides.get(name, value)))
    monkeypatch.setattr(controller.Node, 'create_publisher', publisher)
    monkeypatch.setattr(controller.Node, 'create_subscription', subscriber)
    monkeypatch.setattr(controller.Node, 'create_timer', timer)
    monkeypatch.setattr(controller.Node, 'get_logger', lambda self: MagicMock())
    monkeypatch.setattr(controller.Node, 'get_clock', lambda self: clock)
    monkeypatch.setattr(controller.time, 'monotonic', lambda: now[0])

    def make(**parameters):
        overrides.update(parameters)
        return VehicleControllerNode()
    return SimpleNamespace(make=make, publisher=publisher, subscriber=subscriber,
                           timer=timer, now=now)


def request(speed=1.0, angle=0.0, emergency=False):
    msg = DriveCommand()
    msg.speed_mps = speed
    msg.steering_angle_rad = angle
    msg.emergency_stop = emergency
    msg.header.stamp.sec = 999
    msg.header.frame_id = 'input_frame'
    return msg


def output(node):
    return node.publisher.publish.call_args.args[0]


def assert_stop(msg):
    assert (msg.drive_state, msg.steering_adc, msg.emergency_stop) == (0, 2132, True)


def test_node_topics_timer_and_header(node_factory):
    node = node_factory.make()
    assert node_factory.publisher.call_args.args[:2] == (VehicleCommand, '/vehicle/command')
    assert node_factory.subscriber.call_args.args[:2] == (DriveCommand, '/cmd/final')
    assert node_factory.timer.call_args.args[0] == 0.05
    assert node_factory.timer.call_args.kwargs['clock'].clock_type == ClockType.STEADY_TIME
    node.on_command(request())
    msg = output(node)
    assert (msg.drive_state, msg.steering_adc, msg.emergency_stop) == (1, 2132, False)
    assert msg.header.stamp == Time(sec=123, nanosec=456)
    assert msg.header.frame_id == ''


def test_topic_parameters(node_factory):
    node_factory.make(command_topic='/test/input', vehicle_command_topic='/test/output')
    assert node_factory.publisher.call_args.args[1] == '/test/output'
    assert node_factory.subscriber.call_args.args[1] == '/test/input'


@pytest.mark.parametrize('speed,angle,emergency', [
    (math.nan, math.nan, True), (1.0, 0.1, True), (math.nan, 0.0, False),
    (math.inf, 0.0, False), (1.0, math.nan, False), (1.0, math.inf, False),
    (-math.inf, 0.0, False), (1.0, -math.inf, False)])
def test_node_fail_safe(node_factory, speed, angle, emergency):
    node = node_factory.make()
    node.on_command(request(speed, angle, emergency))
    assert_stop(output(node))
    assert node.last_valid_command is None


def test_node_invalid_enabled_calibration(node_factory):
    node = node_factory.make(steering_calibration_enabled=True, steering_right_angle_rad=math.nan)
    node.on_command(request())
    assert_stop(output(node))


def test_startup_timeout_and_recovery(node_factory):
    node = node_factory.make()
    node.check_timeout()
    assert_stop(output(node))
    node.on_command(request())
    node.publisher.publish.reset_mock()
    node_factory.now[0] = 10.499
    node.check_timeout()
    node.publisher.publish.assert_not_called()
    node_factory.now[0] = 10.5
    node.check_timeout()
    assert_stop(output(node))
    node.on_command(request(-0.2))
    assert output(node).drive_state == VehicleCommand.DRIVE_REVERSE
    assert not output(node).emergency_stop
    assert node.last_valid_command == 10.5


def test_invalid_input_does_not_refresh_timeout(node_factory):
    node = node_factory.make()
    node.on_command(request())
    node_factory.now[0] = 10.4
    node.on_command(request(angle=math.nan))
    assert_stop(output(node))
    assert node.last_valid_command == 10.0
    node_factory.now[0] = 10.5
    node.check_timeout()
    assert_stop(output(node))


@pytest.mark.parametrize('angle,adc', [
    (-0.3054, 150), (0.0, 2132), (0.2810, 3950),
    (-0.1527, 1141), (0.1405, 3041), (-1.0, 150), (1.0, 3950)])
def test_measured_default_mapping(angle, adc):
    config = ConversionConfig()
    assert config.steering_calibration_enabled
    assert config.steering_right_angle_rad == -0.3054
    assert config.steering_left_angle_rad == 0.2810
    assert steering_angle_to_adc(angle) == adc


@pytest.mark.parametrize('speed,state', [(1.0, VehicleCommand.DRIVE_FORWARD),
                                       (0.0, VehicleCommand.DRIVE_STOP)])
@pytest.mark.parametrize('angle,adc', [(-0.3054, 150), (0.0, 2132), (0.2810, 3950),
                                     (-0.1527, 1141), (0.1405, 3041),
                                     (-1.0, 150), (1.0, 3950)])
def test_node_measured_mapping_and_zero_speed(node_factory, speed, state, angle, adc):
    node = node_factory.make()
    node.on_command(request(speed, angle))
    msg = output(node)
    assert (msg.drive_state, msg.steering_adc, msg.emergency_stop) == (state, adc, False)
    assert node.last_valid_command == 10.0


def test_explicit_disabled_calibration(node_factory):
    node = node_factory.make(steering_calibration_enabled=False)
    node.on_command(request())
    assert output(node).steering_adc == 2132
    for angle in (-0.1527, 0.1405):
        node.on_command(request(angle=angle))
        assert_stop(output(node))


@pytest.mark.parametrize('parameters', [
    {'steering_right_angle_rad': 0.3054}, {'steering_right_angle_rad': 0.0},
    {'steering_left_angle_rad': -0.2810}, {'steering_left_angle_rad': 0.0},
    {'steering_left_angle_rad': math.inf}, {'steering_left_angle_rad': math.nan}])
def test_node_invalid_angle_parameters(node_factory, parameters):
    node = node_factory.make(**parameters)
    node.on_command(request(angle=0.1405))
    assert_stop(output(node))
    assert node.last_valid_command is None
