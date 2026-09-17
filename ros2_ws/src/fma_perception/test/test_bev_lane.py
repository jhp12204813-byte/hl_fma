"""Synthetic geometry is test-only; it is never a D435i calibration profile."""
from dataclasses import replace
import math

import cv2
import numpy as np
import pytest

from fma_perception.bev_lane import (BEVConfig, BEVLaneDetector, metric_errors,
                                     narrow_runs, robust_curve)
from fma_perception.lane_detection import LaneConfig


UNIT = (0., 0., 1., 0., 1., 1., 0., 1.)
IMAGE = LaneConfig(roi_top_ratio=0., morph_kernel_size=1)


def config(**kwargs):
    return replace(BEVConfig(source_points=UNIT, destination_points=UNIT, undistort=False), **kwargs)


def scene(shift=0, bend=35, lean=0, right=True):
    image = np.zeros((480, 640, 3), np.uint8)
    y = np.arange(480)
    t = y/479
    for base in ([155, 475] if right else [155]):
        x = base+shift+bend*(1-t)**2+lean*(1-t)
        cv2.polylines(image, [np.c_[x, y].astype(np.int32)], False, (0, 255, 255), 7)
    return image


def test_curved_lanes_and_uncalibrated_contract():
    detector = BEVLaneDetector(config(), IMAGE)
    frame = scene()
    original = frame.copy()
    result = detector.process(frame, 1)
    assert result.estimate.visual_detected, result.diagnostics
    assert not result.estimate.detected
    assert result.diagnostics['visual_confidence'] > .5
    assert len(result.diagnostics['curves']) == 2
    assert abs(result.diagnostics['center_coefficients'][0]-35) < 3
    assert result.estimate.confidence == result.estimate.curvature == 0
    assert np.array_equal(frame, original)


def test_metric_signs_scale_and_curvature_formula():
    cfg = config(geometry_calibrated=True, meters_per_pixel_x=.01,
                 meters_per_pixel_y=.02, vehicle_x_px=320., vehicle_y_px=479.)
    center = np.array([-40., 10., 285.])
    lateral, heading, curvature, distance = metric_errors(center, cfg)
    t = cfg.lookahead_ratio
    slope = (-80*t+10)/479*.01/.02
    assert lateral == pytest.approx((320-np.polyval(center,t))*.01)
    assert heading == pytest.approx(math.atan(slope))
    assert curvature == pytest.approx((80/479**2*.01/.02**2)/(1+slope*slope)**1.5)
    assert curvature > 0 and lateral > 0 and distance > 0
    reverse = metric_errors(np.array([40., -10., 355.]), cfg)
    assert reverse[1] == pytest.approx(-heading)
    assert reverse[2] == pytest.approx(-curvature)


def test_confirmation_loss_and_no_stale_single_lane():
    cfg = config(geometry_calibrated=True, meters_per_pixel_x=.01,
                 meters_per_pixel_y=.02, vehicle_x_px=320., vehicle_y_px=479.)
    detector = BEVLaneDetector(cfg, IMAGE)
    assert not detector.process(scene(), 1).estimate.detected
    assert not detector.process(scene(), 33_000_001).estimate.detected
    assert detector.process(scene(), 66_000_001).estimate.detected
    result = detector.process(scene(right=False), 99_000_001)
    assert len(result.diagnostics['curves']) == 1
    assert not result.estimate.detected and not result.estimate.visual_detected
    assert result.estimate.confidence == 0
    assert not detector.process(scene(), 132_000_001).estimate.detected
    blank = detector.process(np.zeros_like(scene()), 165_000_001)
    assert blank.diagnostics['status'] == 'lane_lost'


def test_temporal_jump_gap_and_reversed_timestamp():
    detector = BEVLaneDetector(config(), IMAGE)
    detector.process(scene(), 1_000_000_000)
    result = detector.process(scene(shift=75), 1_033_000_000)
    assert result.diagnostics['status'] == 'temporal_jump_rejected'
    result = detector.process(scene(), 3_000_000_000)
    assert result.diagnostics['temporal_reset'] == 'timestamp_discontinuity'
    result = detector.process(scene(), 2_000_000_000)
    assert result.diagnostics['confirmation_frames'] == 1
    assert result.diagnostics['temporal_reset'] == 'timestamp_discontinuity'


def test_connected_stop_line_does_not_delete_lane_component():
    frame = scene(bend=0)
    cv2.rectangle(frame, (0, 305), (639, 322), (255,255,255), -1)
    result = BEVLaneDetector(config(lane_colors='white_yellow'), IMAGE).process(frame, 1)
    assert result.estimate.visual_detected, result.diagnostics
    assert result.diagnostics['wide_pixels_removed'] > 5000


def test_arrow_and_crosswalk_rejected_without_component_veto():
    arrow = np.zeros((480,640,3), np.uint8)
    for x in (170,470):
        cv2.rectangle(arrow,(x-10,270),(x+10,479),(255,255,255),-1)
        cv2.fillConvexPoly(arrow,np.int32([[x-65,270],[x,190],[x+65,270]]),(255,255,255))
    zebra = np.zeros_like(arrow)
    for x in (80,230,380,530):
        cv2.rectangle(zebra,(x,280),(x+65,470),(255,255,255),-1)
    for frame in (arrow,zebra):
        result = BEVLaneDetector(config(lane_colors='white_yellow'), IMAGE).process(frame,1)
        assert not result.estimate.visual_detected


def test_nonparallel_lane_width_rejected():
    frame = np.zeros((480,640,3),np.uint8)
    cv2.line(frame,(180,479),(70,0),(0,255,255),7)
    cv2.line(frame,(460,479),(570,0),(0,255,255),7)
    result = BEVLaneDetector(config(), IMAGE).process(frame,1)
    assert not result.estimate.visual_detected


def test_outliers_removed_and_bad_support_rejected():
    y = np.arange(480,dtype=float)
    x = 100+20*(y/479)**2
    x[::8] += 70
    fit = robust_curve(y,x,config())
    assert fit is not None
    np.testing.assert_allclose(fit['coefficients'],[20,0,100],atol=1e-5)
    assert fit['inlier_ratio'] < 1
    assert robust_curve(y[:100],x[:100],config()) is None


def test_camera_info_required_and_zero_distortion_identity():
    cfg = config(undistort=True)
    detector = BEVLaneDetector(cfg,IMAGE)
    assert detector.process(scene(),1).diagnostics['status'] == 'missing_camera_info'
    camera = dict(width=640,height=480,k=[600,0,320,0,600,240,0,0,1],
                  d=[0.,0.,0.,0.,0.],distortion_model='plumb_bob')
    assert detector.process(scene(),2,camera).estimate.visual_detected
    assert detector.process(scene(),3,dict(camera,width=1280)).diagnostics['status'] == 'invalid_camera_info'
    assert detector.previous is None


def test_missing_bev_points_does_not_substitute_identity():
    result = BEVLaneDetector(BEVConfig(undistort=False),IMAGE).process(scene(),1)
    assert result.diagnostics['status'] == 'unconfigured_bev'
    assert not result.estimate.visual_detected


@pytest.mark.parametrize('values', [
    {'source_points': (0.,)*8, 'destination_points': UNIT},
    {'source_points': UNIT}, {'source_points': (float('nan'),)*8},
    {'meters_per_pixel_x': -1}, {'meters_per_pixel_y': float('inf')},
    {'temporal_alpha':0.}, {'windows':0}, {'width':20},
    {'lane_width_min_ratio':.9}, {'lane_colors':'blue'},
])
def test_invalid_bev_configuration(values):
    with pytest.raises(ValueError):
        BEVConfig(**values)


def test_metric_values_alone_do_not_enable_uncalibrated_geometry():
    assert not config(meters_per_pixel_x=.01,meters_per_pixel_y=.02,
                      vehicle_x_px=320,vehicle_y_px=479).metric_ready


def test_sparse_thin_noise_tracks_rejected():
    frame = np.zeros((480,640,3),np.uint8)
    for x in (155,475):
        frame[:,x:x+2] = (0,255,255)
    result = BEVLaneDetector(config(),IMAGE).process(frame,1)
    assert not result.estimate.visual_detected


def test_resolution_mismatch_resets_state():
    detector = BEVLaneDetector(config(),IMAGE)
    detector.process(scene(),1)
    result = detector.process(np.zeros((720,1280,3),np.uint8),2)
    assert result.diagnostics['status'] == 'calibration_image_size_mismatch'
    assert detector.previous is None


def test_multiheight_recovers_middle_lane_without_inventing_center():
    from fma_perception.bev_lane import sliding_curves, choose_pair
    mask = np.zeros((480,640),np.uint8)
    cv2.line(mask,(100,240),(220,110),255,7)
    _, old, _ = sliding_curves(mask,config())
    cfg = config(multi_height_seeds=True)
    _, curves, _ = sliding_curves(mask,cfg)
    assert not old
    assert curves
    assert all(c['partial'] and c['y_max'] < 260 and c['y_min'] > 90 for c in curves)
    assert choose_pair(curves,cfg) is None


def test_multiheight_full_lanes_and_blank():
    detector = BEVLaneDetector(config(multi_height_seeds=True),IMAGE)
    assert detector.process(scene(),1).estimate.visual_detected
    assert not detector.process(np.zeros_like(scene()),2).diagnostics['curves']


def test_directional_tracking_follows_shallow_line_without_mutating_input():
    from fma_perception.bev_lane import sliding_curves
    mask = np.zeros((480,640),np.uint8)
    cv2.line(mask,(30,260),(440,120),255,7)
    original = mask.copy()
    cfg = config(multi_height_seeds=True,directional_tracking=True,margin_ratio=.12)
    _, curves, _ = sliding_curves(mask,cfg)
    assert curves
    assert any(abs(np.polyval(c['coefficients'],200/479)-(30+(260-200)*410/140)) < 12
               for c in curves if c['y_min'] <= 200 <= c['y_max'])
    assert np.array_equal(mask,original)


def test_directional_tracking_keeps_full_curve_contract():
    detector = BEVLaneDetector(config(multi_height_seeds=True,directional_tracking=True),IMAGE)
    assert detector.process(scene(),1).estimate.visual_detected
    assert not detector.process(np.zeros_like(scene()),2).estimate.detected
