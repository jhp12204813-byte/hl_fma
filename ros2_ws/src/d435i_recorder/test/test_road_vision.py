import json

import cv2
import numpy as np
import pytest

from d435i_recorder.road_vision import detect
from d435i_recorder.vision_replay import nearest_depth, replay


def test_blank_and_longitudinal_yellow():
    frame = np.full((480, 640, 3), 60, np.uint8)
    assert not detect(frame)['lane_candidates']
    cv2.line(frame, (60, 470), (250, 250), (0, 220, 220), 8)
    result = detect(frame)
    assert any(c['color'] == 'yellow' for c in result['lane_candidates'])
    assert not result['stop_candidates']


def test_transverse_and_zebra_are_distinct_candidates():
    frame = np.full((480, 640, 3), 60, np.uint8)
    cv2.rectangle(frame, (50, 350), (590, 370), (240, 240, 240), -1)
    assert detect(frame)['stop_candidates']
    frame[:] = 60
    for x in (80, 260, 440):
        cv2.rectangle(frame, (x, 310), (x+65, 450), (240, 240, 240), -1)
    result = detect(frame)
    assert result['crosswalk_pattern']
    assert not result['stop_candidates']
    assert not result['lane_candidates']


def test_white_longitudinal_and_yellow_transverse_are_not_targets():
    frame = np.zeros((480, 640, 3), np.uint8)
    cv2.line(frame, (60, 470), (250, 250), (255, 255, 255), 8)
    assert not detect(frame)['lane_candidates']
    frame[:] = 0
    cv2.rectangle(frame, (50, 350), (590, 370), (0, 255, 255), -1)
    assert not detect(frame)['stop_candidates']


def test_depth_matching_boundary_and_no_future_index_assumption():
    stamps = [100_000_000, 150_000_000]
    rows = [{'index': 7}, {'index': 9}]
    assert nearest_depth(130_000_000, stamps, rows) == rows[1]
    assert nearest_depth(125_000_000, stamps, rows) is None
    assert nearest_depth(1, [], []) is None


@pytest.mark.parametrize('backend', ['baseline', 'fma'])
def test_replay_preserves_timestamps_and_rejects_overwrite(tmp_path, backend):
    session = tmp_path/'session'
    session.mkdir()
    out = tmp_path/'result'
    writer = cv2.VideoWriter(str(session/'color.mp4'), cv2.VideoWriter_fourcc(*'mp4v'), 30, (640, 480))
    rows = []
    for index, stamp in enumerate((1000000000, 1032000000, 1070000000)):
        writer.write(np.full((480, 640, 3), 60, np.uint8))
        rows.append(dict(index=index, timestamp_ns=stamp))
    writer.release()
    (session/'frames.jsonl').write_text(''.join(json.dumps(row)+'\n' for row in rows))
    (session/'depth_frames.jsonl').write_text('')
    (session/'session.json').write_text('{}')
    result = replay(session, out, lane_backend=backend)
    assert result['frames'] == result['output_decoded_frames'] == 3
    detections = [json.loads(line) for line in (out/'detections.jsonl').read_text().splitlines()]
    assert [r['timestamp_ns'] for r in detections] == [r['timestamp_ns'] for r in rows]
    if backend == 'fma':
        assert all('existing_lane' in r and r['lane_candidates'] == [] for r in detections)
    with pytest.raises(FileExistsError):
        replay(session, out)
    with pytest.raises(ValueError):
        replay(session, session/'new')
