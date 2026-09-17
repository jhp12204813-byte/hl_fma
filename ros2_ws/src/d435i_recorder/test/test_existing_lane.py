import cv2
import numpy as np

from d435i_recorder.existing_lane import ExistingLane
from fma_perception.lane_detection import LaneConfig, detect_lane


def test_adapter_matches_existing_algorithm_without_mutation():
    frame = np.zeros((480, 640, 3), np.uint8)
    cv2.line(frame, (130, 479), (230, 290), (0, 255, 255), 7)
    cv2.line(frame, (510, 479), (410, 290), (0, 255, 255), 7)
    original = frame.copy()
    expected = detect_lane(frame, LaneConfig(), yellow_only=True)
    adapter = ExistingLane()
    result, debug = adapter.detect(frame)
    assert np.array_equal(frame, original)
    assert np.array_equal(debug, expected.debug)
    assert result['visual_detected'] == expected.visual_detected
    assert result['visual_detected']
    assert not result['metric_calibrated']
    assert result['lateral_error_m'] is None
    assert len(adapter.provenance['sha256']) == 64


def test_white_markings_are_excluded_from_lane_mask():
    frame = np.zeros((480, 640, 3), np.uint8)
    cv2.line(frame, (130, 479), (230, 290), (255, 255, 255), 7)
    cv2.line(frame, (510, 479), (410, 290), (255, 255, 255), 7)
    assert detect_lane(frame).visual_detected
    estimate = detect_lane(frame, yellow_only=True)
    assert not estimate.visual_detected
    assert not np.any(estimate.debug_mask)
    result, _ = ExistingLane().detect(frame)
    assert not result['visual_detected']


def test_blank_does_not_invent_lane():
    result, _ = ExistingLane().detect(np.zeros((480, 640, 3), np.uint8))
    assert not result['visual_detected']
    assert not result['detected']
