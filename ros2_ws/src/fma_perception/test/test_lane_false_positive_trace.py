"""Offline diagnostic checks; no ROS nodes and no media dependencies."""
import importlib.util
from pathlib import Path

import cv2
import numpy as np


spec = importlib.util.spec_from_file_location(
    'lane_trace', Path(__file__).parents[1] / 'tools' / 'trace_lane_false_positives.py')
trace = importlib.util.module_from_spec(spec)
spec.loader.exec_module(trace)


def test_component_row_width_does_not_confuse_diagonal_displacement():
    mask = np.zeros((200, 300), np.uint8)
    cv2.line(mask, (20, 190), (240, 10), 255, 9)
    _, components = trace.components(mask)
    component, = components
    assert component['bbox_width_over_height'] > 1.
    assert component['row_width_p90_over_height'] < .1
    assert component['row_occupancy_within_bbox'] == 1.


def test_association_tolerates_canny_edge_outside_foreground():
    labels = np.zeros((100, 100), np.int32)
    labels[10:91, 50:60] = 7
    component, fraction, votes = trace.associate((49, 10, 49, 90), labels)
    assert component == 7 and fraction == 1. and votes == {7: 81}


def test_straight_lane_preservation_and_nonmutation():
    image = np.zeros((480, 640, 3), np.uint8)
    cv2.line(image, (100, 479), (230, 240), (255, 255, 255), 10)
    cv2.line(image, (540, 479), (410, 240), (0, 255, 255), 10)
    original = image.copy()
    data = trace.analyze(image)  # Includes exact match to original detector fit.
    assert data['trials']['baseline']['visual']
    assert data['trials']['row_width']['visual']
    assert data['trials']['row_width_y']['visual']
    np.testing.assert_array_equal(image, original)
    assert trace.CONFIG.lane_width_m == trace.CONFIG.meters_per_pixel_y == 0.
    assert trace.CONFIG.single_lane_width_px == 0.


def test_crosswalk_components_are_wide_despite_continuous_rows():
    image = np.zeros((480, 640, 3), np.uint8)
    # Two wide white blocks, each long enough to pass default endpoint support.
    cv2.rectangle(image, (100, 315), (210, 455), (255, 255, 255), -1)
    cv2.rectangle(image, (430, 315), (540, 455), (255, 255, 255), -1)
    data = trace.analyze(image)
    assert any(s['slope_accepted'] for s in data['segments'])
    assert data['trials']['row_width']['eligible_segment_ids'] == []
    assert all(c['row_occupancy_within_bbox'] == 1. for c in data['components'])
