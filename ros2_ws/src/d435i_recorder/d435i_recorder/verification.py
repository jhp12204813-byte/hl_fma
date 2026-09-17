"""Offline integrity verification; PASS does not certify detection or synchronization."""
import json
import math
from pathlib import Path
import cv2
import numpy as np
from .writer import dump


def verify(directory):
    path = Path(directory)
    failures, warnings, timing, counts = [], [], {}, {}
    def fail(message):
        failures.append(message)
    def read(name):
        try:
            return [json.loads(line) for line in (path / (name + '.jsonl')).read_text().splitlines()]
        except Exception as exc:
            fail(f'{name}: {exc}')
            return []
    try:
        session = json.loads((path / 'session.json').read_text())
    except Exception as exc:
        session = {}
        fail(f'session: {exc}')
    streams = {key: read(key) for key in ('frames', 'depth_frames', 'imu', 'device_metadata', 'stop_lines')}
    for key, rows in streams.items():
        counts[key] = len(rows)
        if key in ('stop_lines', 'device_metadata'):
            if not rows:
                warnings.append(f'{key}: empty')
            continue
        if not rows:
            fail(f'{key}: empty')
            continue
        try:
            stamps = np.array([r['timestamp_ns'] for r in rows], dtype=np.int64)
            if np.any(stamps <= 0):
                fail(f'{key}: nonpositive timestamp')
            delta = np.diff(stamps) / 1e9
            if np.any(delta < 0):
                fail(f'{key}: timestamp reversal')
            if np.any(delta == 0):
                warnings.append(f'{key}: duplicate timestamps')
            span = float((stamps[-1] - stamps[0]) / 1e9)
            rate = (len(stamps) - 1) / span if span > 0 else None
            timing[key] = dict(first_timestamp_ns=int(stamps[0]), last_timestamp_ns=int(stamps[-1]),
                               span_s=span, actual_hz=rate,
                               interval_mean_s=float(delta.mean()) if delta.size else None,
                               interval_p95_s=float(np.percentile(delta, 95)) if delta.size else None,
                               max_gap_s=float(delta.max()) if delta.size else None)
            if key != 'imu':
                missed = int(np.maximum(np.rint(delta * 30).astype(int) - 1, 0).sum())
                timing[key]['estimated_missing_frames'] = missed
                timing[key]['intervals_over_1_5_periods'] = int((delta > 1.5/30).sum())
                timing[key]['configured_rate_tolerance_hz'] = 3
                if missed:
                    warnings.append(f'{key}: estimated {missed} missing frame periods (timestamp inference)')
            if delta.size and delta.max() >= 3:
                fail(f'{key}: gap >= 3 seconds')
            elif delta.size and delta.max() > (.2 if key != 'imu' else .1):
                warnings.append(f'{key}: long gap')
            if key != 'imu' and (rate is None or abs(rate - 30) > 3):
                warnings.append(f'{key}: timestamp rate differs from configured 30 FPS')
        except Exception as exc:
            fail(f'{key}: invalid timestamps: {exc}')
    if session.get('state') != 'closed':
        fail('session was not closed cleanly')
    for stream, file_key in (('rgb', 'frames'), ('depth', 'depth_frames'), ('imu', 'imu'),
                             ('device_metadata', 'device_metadata'), ('stop_lines', 'stop_lines')):
        if session.get('stream_counts', {}).get(stream, 0) != counts[file_key]:
            fail(f'{stream}: session count mismatch')
    capture = cv2.VideoCapture(str(path / 'color.mp4'))
    decoded = 0
    while True:
        ok, frame = capture.read()
        if not ok:
            break
        decoded += 1
        if frame.shape != (480, 640, 3):
            fail('MP4 resolution')
    capture.release()
    if decoded == 0 or decoded != len(streams['frames']):
        fail('MP4 empty/corrupt or decoded count differs from frames.jsonl')
    for i, row in enumerate(streams['frames']):
        if row.get('index') != i or not all(k in row for k in ('frame_id', 'receive_monotonic_ns', 'receive_wall_ns')):
            fail(f'RGB metadata/index: {i}')
    for i, row in enumerate(streams['depth_frames']):
        filename = f'depth/{i:06d}.png'
        image = cv2.imread(str(path / filename), cv2.IMREAD_UNCHANGED)
        if row.get('filename') != filename or row.get('index') != i:
            fail(f'depth index/filename: {i}')
        if image is None or image.dtype != np.uint16 or image.shape != (480, 640):
            fail(f'depth missing/corrupt/type/resolution: {filename}')
        if row.get('depth_scale_m') != session.get('depth_scale_m'):
            fail(f'depth scale mismatch: {i}')
    for i, row in enumerate(streams['imu']):
        try:
            for key, size in (('angular_velocity', 3), ('linear_acceleration', 3),
                              ('angular_velocity_covariance', 9), ('linear_acceleration_covariance', 9),
                              ('orientation', 4), ('orientation_covariance', 9)):
                value = row[key]
                values = list(value.values()) if isinstance(value, dict) else value
                if len(values) != size or not np.isfinite(values).all():
                    raise ValueError(key)
        except Exception as exc:
            fail(f'IMU {i}: {exc}')
    infos = session.get('camera_info', {})
    for key in ('color', 'depth'):
        if key not in infos:
            fail(f'missing CameraInfo: {key}')
        elif (infos[key].get('width'), infos[key].get('height')) != (640, 480):
            fail(f'CameraInfo resolution: {key}')
    scale = session.get('depth_scale_m')
    if not isinstance(scale, (int, float)) or not math.isfinite(scale) or scale <= 0 or not session.get('depth_scale_source'):
        fail('depth scale not established')
    if session.get('writer_errors') or any(session.get('dropped_counts', {}).values()):
        fail('writer errors/queue overflow')
    if session.get('recording_errors'):
        fail('recording/watchdog errors: ' + str(session['recording_errors']))
    rgb_stamps = {r.get('timestamp_ns') for r in streams['frames']}
    depth_stamps = {r.get('timestamp_ns') for r in streams['depth_frames']}
    for i, row in enumerate(streams['stop_lines']):
        try:
            required = {'rgb_timestamp_ns', 'bbox', 'detection_status', 'depth_status',
                        'matched_depth_timestamp_ns', 'timestamp_difference_ns', 'optical_z_m',
                        'valid_pixel_count', 'valid_ratio'}
            if not required.issubset(row):
                raise ValueError('missing fields')
            if row['rgb_timestamp_ns'] not in rgb_stamps:
                raise ValueError('RGB timestamp missing')
            if row['detection_status'] == 'no_candidate':
                if (row['bbox'] is not None or row['optical_z_m'] is not None
                        or row['depth_status'] != 'not_applicable'
                        or row['matched_depth_timestamp_ns'] is not None
                        or row['timestamp_difference_ns'] is not None):
                    raise ValueError('no_candidate payload')
                continue
            if row['detection_status'] != 'candidate':
                raise ValueError('detection status')
            x, y, w, h = row['bbox']
            if min(x, y) < 0 or min(w, h) <= 0 or x+w > 640 or y+h > 480:
                raise ValueError('bbox')
            if not 0 <= row['valid_pixel_count'] <= w*h or not 0 <= row['valid_ratio'] <= 1:
                raise ValueError('pixel count/ratio range')
            if abs(row['valid_pixel_count'] / (w*h) - row['valid_ratio']) > 1e-8:
                raise ValueError('pixel count/ratio mismatch')
            stamp = row['matched_depth_timestamp_ns']
            if stamp is not None:
                delta = abs(stamp - row['rgb_timestamp_ns'])
                if stamp not in depth_stamps or delta > 20_000_000 or delta != row['timestamp_difference_ns']:
                    raise ValueError('depth matching')
            if row['depth_status'] not in ('valid', 'no_match', 'insufficient_valid_depth'):
                raise ValueError('depth status')
            if (row['depth_status'] == 'no_match') != (stamp is None):
                raise ValueError('match status mismatch')
            if stamp is None and row['timestamp_difference_ns'] is not None:
                raise ValueError('unmatched delta must be null')
            if row['depth_status'] == 'valid':
                if stamp is None or not .1 <= row['optical_z_m'] <= 10 or row['valid_pixel_count'] < 20 or not .3 <= row['valid_ratio'] <= 1:
                    raise ValueError('distance validity')
            elif row['optical_z_m'] is not None:
                raise ValueError('invalid distance must be null')
        except Exception as exc:
            fail(f'stop_lines {i}: {exc}')
    stop_details = session.get('stop_details')
    if stop_details and stop_details.get('automatic'):
        fail(stop_details['message'])
    warnings.extend(session.get('warnings', []))
    result = dict(status='FAIL' if failures else ('WARN' if warnings else 'PASS'),
                  failures=failures, warnings=warnings, counts=counts,
                  decoded_rgb_frames=decoded, timing=timing, stop_details=stop_details,
                  writer_metrics=session.get('writer_metrics'),
                  received_streams=session.get('received_streams'),
                  scope='File integrity only; not detection accuracy or perfect sensor synchronization')
    dump(path / 'verification.json', result)
    return result
