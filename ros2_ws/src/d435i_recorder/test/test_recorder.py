import json
import threading
from types import SimpleNamespace
import cv2
import numpy as np
import pytest
from d435i_recorder.stop_line import candidates, measure
from d435i_recorder.writer import Writer
from d435i_recorder.verification import verify
from d435i_recorder.recording import header, imu_data, Recorder, EXPECTED
from sensor_msgs.msg import Imu


def meta(stamp):
    return dict(timestamp_ns=stamp, frame_id='color_optical', receive_wall_ns=stamp,
                receive_monotonic_ns=stamp)


def metadata():
    return dict(depth_scale_m=.001, depth_scale_source='synthetic fixture',
                camera_info={k: dict(width=640, height=480) for k in ('color', 'depth')})


def build_session(tmp_path):
    writer = Writer(tmp_path, metadata())
    bgr = np.zeros((480, 640, 3), np.uint8)
    bgr[350:370, 100:500] = 255
    depth = np.full((480, 640), 1500, np.uint16)
    for i in range(5):
        stamp = 1_000_000_000 + i*33_333_333
        writer.submit('depth', depth, meta(stamp))
        writer.submit('rgb', bgr, meta(stamp))
        writer.submit('imu', imu_data(Imu()), meta(stamp))
        writer.submit('device_metadata', {'json_data': '{"frame_number":42,"gain":16}'}, meta(stamp))
    result = writer.close()
    return writer, result


def test_session_mp4_png_jsonl_pass(tmp_path):
    writer, result = build_session(tmp_path)
    assert result['status'] == 'PASS', result
    assert result['decoded_rgb_frames'] == 5
    depth = cv2.imread(str(writer.path/'depth/000000.png'), -1)
    assert depth.dtype == np.uint16 and np.all(depth == 1500)
    rows = [json.loads(v) for v in (writer.path/'frames.jsonl').read_text().splitlines()]
    assert [r['index'] for r in rows] == list(range(5))
    assert rows[0]['receive_monotonic_ns'] == 1_000_000_000
    stops = [json.loads(v) for v in (writer.path/'stop_lines.jsonl').read_text().splitlines()]
    assert stops[0]['optical_z_m'] == 1.5
    raw = json.loads((writer.path/'device_metadata.jsonl').read_text().splitlines()[0])
    assert json.loads(raw['json_data'])['frame_number'] == 42
    second = Writer(tmp_path, metadata())
    second.close()
    assert second.path != writer.path


def test_imu_serialization():
    message = Imu()
    message.header.stamp.sec = 12
    message.header.stamp.nanosec = 34
    message.header.frame_id = 'imu_frame'
    message.angular_velocity.x = 2.5
    message.orientation_covariance[0] = -1
    assert header(message)['timestamp_ns'] == 12_000_000_034
    result = imu_data(message)
    assert result['angular_velocity']['x'] == 2.5
    assert result['orientation_covariance'][0] == -1


def test_white_candidate_yellow_excluded():
    image = np.zeros((480, 640, 3), np.uint8)
    image[340:360, 100:550] = (255, 255, 255)
    assert candidates(image) == [[100, 340, 450, 20]]
    image[340:360, 100:550] = (0, 255, 255)
    assert candidates(image) == []


@pytest.mark.parametrize('delta,expected', [(20_000_000, 'valid'), (20_000_001, 'no_match'), (-20_000_000, 'valid')])
def test_matching_boundary(delta, expected):
    depth = np.full((10, 10), 2000, np.uint16)
    result = measure(100_000_000, [0, 0, 10, 10], [(100_000_000+delta, depth)], .001)
    assert result['depth_status'] == expected


def test_median_ratio_and_minimum_pixels():
    depth = np.zeros((10, 10), np.uint16)
    depth[:3] = 1234
    result = measure(10, [0, 0, 10, 10], [(10, depth)], .001)
    assert result['optical_z_m'] == 1.234 and result['valid_ratio'] == .3
    depth[2, 0] = 0
    assert measure(10, [0, 0, 10, 10], [(10, depth)], .001)['optical_z_m'] is None
    assert measure(10, [0, 0, 3, 3], [(10, np.ones((10, 10), np.uint16)*1000)], .001)['optical_z_m'] is None
    assert measure(10, [0, 0, 10, 10], [], .001)['depth_status'] == 'no_match'


def test_nearest_future_depth():
    depth = np.ones((10, 10), np.uint16)*3000
    result = measure(100_000_000, [0, 0, 10, 10], [(90_000_000, depth), (102_000_000, depth)], .001)
    assert result['matched_depth_timestamp_ns'] == 102_000_000


def test_queue_overflow(tmp_path, monkeypatch):
    gate = threading.Event()
    original = Writer._run
    def paused(self):
        gate.wait(5)
        original(self)
    monkeypatch.setattr(Writer, '_run', paused)
    writer = Writer(tmp_path, metadata(), capacity=1)
    try:
        assert writer.submit('imu', imu_data(Imu()), meta(1))
        assert not writer.submit('imu', imu_data(Imu()), meta(2))
        assert writer.dropped['imu'] == 1
    finally:
        gate.set()
    assert writer.close()['status'] == 'FAIL'
    assert not writer.submit('imu', {}, {})


@pytest.mark.parametrize('corruption', ['png', 'mp4', 'nan', 'reverse', 'stop', 'camera_info'])
def test_verification_failures(tmp_path, corruption):
    writer, _ = build_session(tmp_path)
    path = writer.path
    if corruption == 'png':
        cv2.imwrite(str(path/'depth/000000.png'), np.zeros((480, 640), np.uint8))
    elif corruption == 'mp4':
        (path/'color.mp4').write_bytes(b'broken')
    elif corruption in ('nan', 'reverse'):
        rows = [json.loads(v) for v in (path/'imu.jsonl').read_text().splitlines()]
        if corruption == 'nan':
            rows[0]['linear_acceleration']['x'] = float('nan')
        else:
            rows[1]['timestamp_ns'] = 1
        (path/'imu.jsonl').write_text(''.join(json.dumps(v)+'\n' for v in rows))
    elif corruption == 'stop':
        rows = [json.loads(v) for v in (path/'stop_lines.jsonl').read_text().splitlines()]
        rows[0]['timestamp_difference_ns'] = 21_000_000
        (path/'stop_lines.jsonl').write_text(''.join(json.dumps(v)+'\n' for v in rows))
    else:
        session = json.loads((path/'session.json').read_text())
        session['camera_info'] = {}
        (path/'session.json').write_text(json.dumps(session))
    assert verify(path)['status'] == 'FAIL'


def test_warn_missing_metadata_and_duplicate(tmp_path):
    writer, _ = build_session(tmp_path)
    (writer.path/'device_metadata.jsonl').write_text('')
    session = json.loads((writer.path/'session.json').read_text())
    session['stream_counts']['device_metadata'] = 0
    (writer.path/'session.json').write_text(json.dumps(session))
    assert verify(writer.path)['status'] == 'WARN'
    rows = [json.loads(v) for v in (writer.path/'imu.jsonl').read_text().splitlines()]
    rows[1]['timestamp_ns'] = rows[0]['timestamp_ns']
    (writer.path/'imu.jsonl').write_text(''.join(json.dumps(v)+'\n' for v in rows))
    assert any('duplicate' in v for v in verify(writer.path)['warnings'])


def test_watchdog_absent_stream():
    rec = Recorder.__new__(Recorder)
    rec.lock = threading.RLock()
    rec.device_future = rec.param_future = None
    rec.device_client = rec.param_client = SimpleNamespace(service_is_ready=lambda: False)
    rec.node = SimpleNamespace(count_publishers=lambda topic: 1)
    rec.writer = SimpleNamespace(errors=[])
    rec.parameters = dict(EXPECTED)
    rec.last_seen = {}
    rec.started = 0
    rec.errors = []
    stopped = []
    rec.stop = lambda **kwargs: stopped.append(kwargs)
    rec.poll()
    assert stopped and len(rec.errors) == 3


def test_writer_bad_encoding_fails(tmp_path):
    writer = Writer(tmp_path, metadata())
    writer.submit('depth', np.zeros((480, 640), np.uint8), meta(1))
    result = writer.close()
    assert result['status'] == 'FAIL'
    assert 'uint16' in writer.errors[0]


def test_package_manifest():
    from pathlib import Path
    from catkin_pkg.package import parse_package
    package = parse_package(str(Path(__file__).parents[1] / 'package.xml'))
    assert package.get_build_type() == 'ament_python'
    assert any(e.tagname == 'rqt_gui' for e in package.exports)


def test_writer_fatal_drains_queue(tmp_path, monkeypatch):
    def fail(*args, **kwargs):
        raise RuntimeError('simulated codec unavailable')
    monkeypatch.setattr(cv2, 'VideoWriter', fail)
    writer = Writer(tmp_path, metadata(), capacity=1)
    writer.submit('imu', imu_data(Imu()), meta(1))
    assert writer.close()['status'] == 'FAIL'
    assert any('codec unavailable' in e for e in writer.errors)


def test_watchdog_startup_grace():
    import time
    rec = Recorder.__new__(Recorder)
    rec.lock = threading.RLock()
    rec.device_future = rec.param_future = None
    rec.device_client = rec.param_client = SimpleNamespace(service_is_ready=lambda: False)
    rec.node = SimpleNamespace(count_publishers=lambda topic: 1)
    rec.writer = SimpleNamespace(errors=[])
    rec.parameters = dict(EXPECTED)
    rec.last_seen = {}
    rec.started = time.monotonic()
    rec.errors = []
    rec.stop = lambda: pytest.fail('startup must not trigger watchdog')
    rec.poll()
    assert not rec.errors


def test_raw_rgb_not_overlay(tmp_path):
    writer = Writer(tmp_path, metadata())
    image = np.zeros((480, 640, 3), np.uint8)
    image[350:370, 100:500] = 255
    writer.submit('rgb', image.copy(), meta(1_000_000_000))
    writer.close()
    capture = cv2.VideoCapture(str(writer.path/'color.mp4'))
    ok, decoded = capture.read()
    capture.release()
    assert ok
    # A preview's green candidate box must never be painted onto stored RGB.
    assert np.max(np.abs(decoded[:, :, 1].astype(int) - decoded[:, :, 0].astype(int))) < 12


def test_retry_parameter_query_after_empty_startup_response():
    rec = Recorder.__new__(Recorder)
    rec.lock = threading.RLock()
    rec.writer = None
    rec.device_future = rec.param_future = None
    rec.device_client = SimpleNamespace(service_is_ready=lambda: False)
    values = []
    for value in EXPECTED.values():
        values.append(SimpleNamespace(type=1 if isinstance(value, bool) else 2 if isinstance(value, int) else 4,
                                      bool_value=value, integer_value=value, string_value=value))
    responses = iter([[], values])
    def request(_):
        response = SimpleNamespace(values=next(responses))
        return SimpleNamespace(done=lambda: True, result=lambda: response)
    rec.param_client = SimpleNamespace(service_is_ready=lambda: True, call_async=request)
    rec.poll()
    assert rec.parameters == {} and rec.param_future is None
    rec.param_poll_due = 0
    rec.poll()
    assert rec.parameters == EXPECTED
