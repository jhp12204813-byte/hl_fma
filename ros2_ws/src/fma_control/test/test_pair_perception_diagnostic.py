"""Read-only frame diagnostics; synthetic images and mocked ROS only."""
import copy
from unittest.mock import MagicMock

import numpy as np
import pytest

from test_lane_follow import bev, env, pair_with_side_ranges
from fma_control import lane_stop_test_node as stop
from fma_perception.paper_lane_tracker import PaperLaneTracker


@pytest.mark.parametrize('left_near,right_near,limiting', [
    (3.7, 3.4, 'LEFT'), (3.4, 3.7, 'RIGHT'), (3.7, 3.7, 'BOTH')])
def test_ranges_limiting_side_and_unselected_raw_pixels(bev, left_near, right_near, limiting):
    observation = pair_with_side_ranges(bev, right_near, left_near)
    for side in ('left', 'right'):
        fit = observation[side]
        fit['_inlier_y'] = np.arange(fit['y_min'], fit['y_max'] + 1)
    mask = np.zeros((bev.height, bev.width), np.uint8)
    mask[300:501, 80:83] = 255
    mask[299, :] = 255
    mask[501, :] = 255
    original = mask.copy()
    before = copy.deepcopy(observation)
    d = stop.pair_perception_diagnostic(observation, bev, mask)
    assert d['LEFT_observed_min_m'] == pytest.approx(left_near)
    assert d['RIGHT_observed_min_m'] == pytest.approx(right_near)
    assert d['LEFT_observed_max_m'] == pytest.approx(3.95)
    assert d['RIGHT_observed_max_m'] == pytest.approx(3.95)
    assert d['pair_common_observed_min_m'] == pytest.approx(max(left_near, right_near))
    assert d['pair_common_observed_max_m'] == pytest.approx(3.95)
    assert d['pair_near_limit_side'] == limiting
    assert d['raw_mask_pixels_1_3m'] == 201 * 3
    assert d['selected_inliers_1_3m'] == 0
    for side in ('left', 'right'):
        assert d[f'{side.upper()}_y_max_row'] == observation[side]['y_max']
        assert d[f'{side.upper()}_points'] == 100
        assert d[f'{side.upper()}_residual_px'] == 1.
        np.testing.assert_array_equal(observation[side]['_inlier_y'], before[side]['_inlier_y'])
    np.testing.assert_array_equal(mask, original)


def test_inlier_count_uses_final_selection_inclusive_endpoints(bev):
    observation = pair_with_side_ranges(bev, 1., 1.)
    # Final selected points can be sparse despite a broad reported fit range.
    observation['left']['_inlier_y'] = np.array([299, 300, 300, 400, 500, 501])
    observation['right']['_inlier_y'] = np.array([300, 500])
    d = stop.pair_perception_diagnostic(observation, bev, np.zeros((bev.height, bev.width), np.uint8))
    assert d['LEFT_inliers_1_3m'] == 4
    assert d['RIGHT_inliers_1_3m'] == 2
    assert d['selected_inliers_1_3m'] == 6
    del observation['right']['_inlier_y']
    d = stop.pair_perception_diagnostic(observation, bev, None)
    assert d['RIGHT_inliers_1_3m'] is None  # Missing telemetry must not look like zero.
    assert d['selected_inliers_1_3m'] is None and d['raw_mask_pixels_1_3m'] is None


def test_real_tracker_selected_inliers_match_diagnostic(bev):
    mask = np.zeros((bev.height, bev.width), np.uint8)
    mask[:, 97:104] = 255
    mask[:, 497:504] = 255
    observation = PaperLaneTracker().process(mask)
    assert observation['pair_state'] == 'PAIR_VALID'
    d = stop.pair_perception_diagnostic(observation, bev, mask)
    assert d['raw_mask_pixels_1_3m'] == 201 * 14
    for side in ('left', 'right'):
        fit = observation[side]
        expected = sum(1 <= bev.bev_pixel_to_ground(x, y)[1] <= 3
                       for x, y in zip(fit['_inlier_x'], fit['_inlier_y']))
        assert d[f'{side.upper()}_inliers_1_3m'] == expected
        assert expected > 0


@pytest.mark.parametrize('pair_state', ['PAIR_VALID', 'PAIR_WEAK'])
def test_every_pair_frame_logged_without_control_changes(env, monkeypatch, pair_state):
    node = env.make(_node_class=stop.LaneStopTestNode)
    observation = pair_with_side_ranges(node.bev, 3.7, 3.7)
    observation['pair_state'] = pair_state
    for side in ('left', 'right'):
        observation[side]['_inlier_y'] = np.array([220, 230])
    mask = np.zeros((node.bev.height, node.bev.width), np.uint8)
    mask[350, 100:120] = 255
    node.make_mask = MagicMock(return_value=(None, mask, mask))
    monkeypatch.setattr(stop, 'process_lane', lambda *args: (observation, 0))
    logger = MagicMock()
    monkeypatch.setattr(node, 'get_logger', lambda: logger)
    for _ in range(3):
        result, _ = node.process_frame(object())
        assert not result['valid']
        assert node.stop_pending[2] is observation
    assert logger.info.call_count == 3  # Same time; no timer tick needed.
    line = logger.info.call_args.args[0]
    assert 'PAIR_PERCEPTION frame_stamp=' in line
    assert f'pair_state={pair_state}' in line
    assert 'pair_near_limit_side=BOTH' in line
    assert 'raw_mask_pixels_1_3m=20' in line
    assert 'selected_inliers_1_3m=0' in line
    env.camera.assert_not_called()
    env.publisher.assert_not_called()


def test_single_side_has_no_pair_diagnostic(bev):
    assert stop.pair_perception_diagnostic({'state': 'LEFT_ONLY'}, bev, None) is None


def test_side_raw_split_and_candidate_lineage(bev):
    observation = pair_with_side_ranges(bev, 1., 1.)
    mask = np.zeros((bev.height, bev.width), np.uint8)
    mask[300, 299] = 255
    mask[500, 300:303] = 255
    observation['left']['_candidate_y'] = np.array([300, 400, 500, 501])
    observation['right']['_candidate_y'] = np.array([299, 300, 400, 500])
    observation['left']['_inlier_y'] = np.array([300, 500])
    observation['right']['_inlier_y'] = np.array([299])
    d = stop.pair_perception_diagnostic(observation, bev, mask)
    assert d['LEFT_raw_mask_pixels_1_3m'] == 1
    assert d['RIGHT_raw_mask_pixels_1_3m'] == 3
    assert d['LEFT_candidate_points_1_3m'] == 3
    assert d['RIGHT_candidate_points_1_3m'] == 3
    assert d['LEFT_selected_inliers_1_3m'] == 2
    assert d['RIGHT_selected_inliers_1_3m'] == 0
