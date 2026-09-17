import threading
from types import SimpleNamespace
from unittest.mock import Mock
import cv2
import numpy as np
from d435i_recorder.depth_png import write_depth_png
from d435i_recorder.recording import Recorder, EXPECTED
from test_recorder import build_session


def test_depth_png_all_uint16_values(tmp_path):
    depth = np.arange(65536, dtype=np.uint16).reshape(256, 256)
    path = tmp_path/'depth.png'
    write_depth_png(path, depth)
    actual = cv2.imread(str(path), cv2.IMREAD_UNCHANGED)
    assert actual.dtype == np.uint16
    assert np.array_equal(depth, actual)


def test_writer_latency_metrics(tmp_path):
    writer, result = build_session(tmp_path)
    metrics = result['writer_metrics']
    assert metrics['queue_capacity'] == 512
    assert 0 < metrics['queue_peak'] <= 512
    assert metrics['overflow_events'] == []
    assert metrics['latency_ms']['mp4_write']['count'] == 5
    assert metrics['latency_ms']['depth_png_write']['count'] == 5


def test_auto_stop_overflow_while_streams_fresh(monkeypatch):
    monkeypatch.setattr('d435i_recorder.recording.time.monotonic', lambda: 100.)
    rec = Recorder.__new__(Recorder)
    rec.lock = threading.RLock()
    rec.device_future = rec.param_future = None
    rec.device_client = rec.param_client = SimpleNamespace(service_is_ready=lambda: False)
    rec.node = SimpleNamespace(count_publishers=lambda topic: 1)
    rec.writer = SimpleNamespace(errors=['queue overflow'])
    rec.parameters = dict(EXPECTED)
    rec.started = 94.
    rec.last_seen = dict(rgb=99.99, depth=99.98, imu=99.999)
    rec.errors = []
    rec.stop = Mock()
    rec.poll()
    rec.stop.assert_called_once_with(reason='queue overflow', automatic=True)
    assert not rec.errors


def test_stop_persists_reason_and_receive_age(monkeypatch):
    monkeypatch.setattr('d435i_recorder.recording.time.monotonic', lambda: 100.)
    rec = Recorder.__new__(Recorder)
    rec.lock = threading.RLock()
    rec.writer = Mock()
    writer = rec.writer
    rec.camera_info = {}
    rec.errors = ['IMU timeout, last message 3.20s ago']
    rec.node = Mock()
    rec.started = 90.
    rec.last_seen = dict(rgb=99.99, depth=99.98, imu=96.8)
    rec.last_received = dict(imu=dict(timestamp_ns=123))
    rec.stop(reason=rec.errors[0], automatic=True)
    rec.finalizer.join()
    extra = writer.close.call_args.args[0]
    assert extra['stop_details']['automatic']
    assert extra['stop_details']['message'].startswith('AUTO STOP: IMU timeout')
    assert abs(extra['stop_details']['last_messages']['imu']['age_s'] - 3.2) < 1e-8
    assert extra['stop_details']['last_messages']['imu']['timestamp_ns'] == 123


def test_depth_worker_failure_finalization_does_not_deadlock(tmp_path, monkeypatch):
    from d435i_recorder.writer import Writer
    from test_recorder import metadata, meta
    def fail(*args):
        raise OSError('simulated depth disk failure')
    monkeypatch.setattr('d435i_recorder.writer.write_depth_png', fail)
    writer = Writer(tmp_path, metadata())
    writer.submit('depth', np.zeros((480, 640), np.uint16), meta(1))
    result = []
    finish = threading.Thread(target=lambda: result.append(writer.close()), daemon=True)
    finish.start()
    finish.join(timeout=5)
    assert not finish.is_alive()
    assert result[0]['status'] == 'FAIL'
    assert any('depth disk failure' in error for error in writer.errors)


def test_depth_disk_work_does_not_block_imu(tmp_path, monkeypatch):
    import time
    from d435i_recorder.writer import Writer
    from test_recorder import metadata, meta, imu_data, Imu
    gate = threading.Event()
    original = write_depth_png
    def slow(*args):
        gate.wait(5)
        original(*args)
    monkeypatch.setattr('d435i_recorder.writer.write_depth_png', slow)
    writer = Writer(tmp_path, metadata())
    try:
        writer.submit('depth', np.zeros((480, 640), np.uint16), meta(1))
        writer.submit('imu', imu_data(Imu()), meta(1))
        end = time.monotonic() + 2
        while writer.counts['imu'] == 0 and time.monotonic() < end:
            time.sleep(.01)
        assert writer.counts['imu'] == 1
    finally:
        gate.set()
        writer.close()


def test_fast_imu_serialization_preserves_all_values():
    from sensor_msgs.msg import Imu
    from rosidl_runtime_py.convert import message_to_ordereddict
    from d435i_recorder.recording import imu_data
    msg = Imu()
    msg.angular_velocity.x = -1.25
    msg.linear_acceleration.z = 9.81
    msg.orientation.w = .5
    msg.orientation_covariance[0] = -1.
    msg.angular_velocity_covariance[3] = .75
    expected = dict(message_to_ordereddict(msg))
    expected.pop('header')
    assert imu_data(msg) == expected


def test_peak_tracks_full_queue_before_consumer():
    from d435i_recorder.writer import MeasuredQueue
    q = MeasuredQueue(2)
    q.put_nowait(1)
    q.put_nowait(2)
    assert q.peak == 2
    q.get_nowait()
    q.get_nowait()
    assert q.peak == 2


def test_candidate_work_does_not_block_imu(tmp_path, monkeypatch):
    import time
    from d435i_recorder.writer import Writer
    from test_recorder import metadata, meta, imu_data, Imu
    gate = threading.Event()
    entered = threading.Event()
    def slow_candidate(image):
        entered.set()
        gate.wait(5)
        return []
    monkeypatch.setattr('d435i_recorder.writer.candidates', slow_candidate)
    writer = Writer(tmp_path, metadata())
    try:
        writer.submit('rgb', np.zeros((480, 640, 3), np.uint8), meta(1_000_000_000))
        assert entered.wait(2)
        writer.submit('imu', imu_data(Imu()), meta(1_000_000_000))
        end = time.monotonic() + 2
        while writer.counts['imu'] == 0 and time.monotonic() < end:
            time.sleep(.01)
        assert writer.counts['imu'] == 1
    finally:
        gate.set()
        writer.close()
    assert writer.counts['stop_lines'] == 1
