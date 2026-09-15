"""All hardware mocked; no camera or motor is opened."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import MagicMock
import re

import pytest

spec = importlib.util.spec_from_file_location('steering_tool', Path(__file__).parents[1] / 'steering_only_lane_test.py')
tool = importlib.util.module_from_spec(spec)
spec.loader.exec_module(tool)


@pytest.fixture
def wire(monkeypatch):
    port = MagicMock()
    port.write.side_effect = len
    monkeypatch.setattr(tool.serial, 'Serial', MagicMock(return_value=port))
    return port


def payloads(wire):
    return [c.args[0] for c in wire.write.call_args_list]


def test_dry_run_never_opens_serial(wire):
    output = tool.SteeringPort('unused', False)
    output.stop()
    output.steer(.1)
    output.close()
    tool.serial.Serial.assert_not_called()
    wire.write.assert_not_called()


def test_only_calibrated_steering_and_stop(wire):
    output = tool.SteeringPort('unused', True)
    assert output.steer(-100) == 150
    assert output.steer(100) == 3950
    output.close()
    assert payloads(wire) == [b'X', b'T0150', b'T3950', b'X']
    wire.close.assert_called_once()


@pytest.mark.parametrize('payload', [b'F0000', b'F0040', b'B0040', b'W', b'S', b'T2132W', b'XW'])
def test_drive_payloads_rejected(wire, payload):
    output = tool.SteeringPort('unused', True)
    with pytest.raises(ValueError): output._write(payload)
    output.close()
    assert payloads(wire) == [b'X', b'X']


def test_five_distinct_fresh_frames_and_invalid_reset():
    gate = tool.Gate()
    for sequence in range(1, 6):
        now = sequence * .03
        gate.update(sequence, now, {'valid': True}, now)
        assert gate.fresh(now) == (sequence == 5)
    gate.update(6, .18, {'valid': False}, .18)
    assert not gate.fresh(.18)
    gate.update(7, .21, {'valid': True}, .21)
    assert gate.streak == 1
    assert not gate.fresh(.21)
    assert not gate.fresh(1.)
    assert gate.state == 'INVALID' and gate.streak == 0


def test_missing_frames_break_acquisition():
    gate = tool.Gate()
    for sequence in (1, 2, 3, 4, 6):
        gate.update(sequence, .01 * sequence, {'valid': True}, .01 * sequence)
    assert gate.streak == 1
    assert not gate.fresh(.06)


@pytest.mark.parametrize('error', [KeyboardInterrupt, RuntimeError])
@pytest.mark.parametrize('case', ['valid', 'invalid', 'stale', 'camera_error'])
def test_run_cleanup_and_wire_allowlist(wire, monkeypatch, error, case):
    def fake_thread(**kwargs):
        _, samples, _, _ = kwargs['args']
        worker = MagicMock()
        def start():
            for sequence in range(1, 6):
                samples.append((sequence, 99.85 + sequence * .02, {
                    'valid': True, 'steering_cmd_rad': .1}))
            if case == 'invalid': samples.append((6, 99.99, {'valid': False}))
            if case == 'stale':
                samples.clear()
                samples.append((6, 99., {'valid': True, 'steering_cmd_rad': .1}))
            if case == 'camera_error': samples.append((6, 99.99, {'valid': False, 'error': 'read failed'}))
        worker.start.side_effect = start
        return worker
    monkeypatch.setattr(tool.threading, 'Thread', fake_thread)
    monkeypatch.setattr(tool.time, 'monotonic', lambda: 100.)
    monkeypatch.setattr(tool.time, 'sleep', MagicMock(side_effect=error))
    monkeypatch.setattr(tool.cv2, 'VideoCapture', MagicMock(side_effect=AssertionError('No camera')))
    args = SimpleNamespace(port='unused', device='unused', enable_steering=True, lane_min_width_m=3.0)
    with pytest.raises((error, RuntimeError)): tool.run(args)
    commands = payloads(wire)
    assert commands[0] == commands[-1] == b'X'
    assert all(re.fullmatch(rb'X|T[0-9]{4}', cmd) for cmd in commands)
    assert any(cmd.startswith(b'T') for cmd in commands) == (case == 'valid')
    wire.close.assert_called_once()
    tool.cv2.VideoCapture.assert_not_called()


def test_partial_serial_write_attempts_stop_and_closes(wire):
    wire.write.return_value = 0
    wire.write.side_effect = None
    with pytest.raises(IOError): tool.SteeringPort('unused', True)
    assert payloads(wire) == [b'X', b'X']
    wire.close.assert_called_once()


def test_steering_rate_limit_and_stale_stop(wire, monkeypatch):
    clock = [100.]
    writes = []
    wire.write.side_effect = lambda data: (writes.append((clock[0], data)) or len(data))
    def thread(**kwargs):
        samples = kwargs['args'][1]
        worker = MagicMock()
        worker.start.side_effect = lambda: samples.extend(
            (i, 99.95 + i * .01, {'valid': True, 'steering_cmd_rad': 0.}) for i in range(1, 6))
        return worker
    def sleep(_):
        clock[0] += .02
        if clock[0] > 100.42:
            raise KeyboardInterrupt
    monkeypatch.setattr(tool.threading, 'Thread', thread)
    monkeypatch.setattr(tool.time, 'monotonic', lambda: clock[0])
    monkeypatch.setattr(tool.time, 'sleep', sleep)
    args = SimpleNamespace(port='unused', device='unused', enable_steering=True, lane_min_width_m=3.)
    with pytest.raises(KeyboardInterrupt): tool.run(args)
    times = [stamp for stamp, data in writes if data.startswith(b'T')]
    assert len(times) >= 2
    assert all(b - a >= tool.COMMAND_PERIOD for a, b in zip(times, times[1:]))
    assert all(stamp < 100.25 for stamp in times)
    assert any(stamp >= 100.25 and data == b'X' for stamp, data in writes)
