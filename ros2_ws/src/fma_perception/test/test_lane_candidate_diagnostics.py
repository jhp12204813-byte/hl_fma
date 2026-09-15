"""Diagnostic provenance and overlay without hardware or algorithm changes."""
import copy
import numpy as np
from fma_perception.paper_lane_tracker import PaperLaneTracker, _joint_refit
from test_live_stop_thickness_option import live


def strip_diagnostics(value):
    if isinstance(value, dict):
        return {k: strip_diagnostics(v) for k, v in value.items() if not k.startswith('_candidate')}
    if isinstance(value, (tuple, list)):
        return [strip_diagnostics(v) for v in value]
    return value.tolist() if isinstance(value, np.ndarray) else value


def test_diagnostic_tracking_matches_production_across_reacquisition():
    production, diagnostic = PaperLaneTracker(), PaperLaneTracker(diagnostics=True)
    for positions in [(100, 500), (100, 500), (100,), (), (100, 500), (100, 450)]:
        mask = np.zeros((530, 600), np.uint8)
        for x in positions:
            mask[:, x-3:x+4] = 255
        original = mask.copy()
        a, b = production.process(mask), diagnostic.process(mask)
        assert strip_diagnostics(a) == strip_diagnostics(b)
        np.testing.assert_array_equal(mask, original)
        for side in ('left', 'right'):
            fit = b.get(side)
            if fit is not None:
                assert '_candidate_y' not in a[side]
                assert '_candidate_y' in fit
                assert len(fit['_candidate_y']) >= fit['points']
    assert strip_diagnostics(production.last_valid_pair) == strip_diagnostics(diagnostic.last_valid_pair)


def test_candidates_are_actual_search_input_not_all_raw_pixels():
    tracker = PaperLaneTracker(diagnostics=True)
    mask = np.zeros((530, 600), np.uint8)
    mask[:240, 497:504] = 255
    mask[300:501, 560:565] = 255  # Outside previous-fit search margin.
    fit = tracker._search_previous(mask, np.array([0., 0., 500.]))
    assert fit is not None
    assert not np.any((fit['_candidate_y'] >= 300) & (fit['_candidate_y'] <= 500))
    # Near candidates scattered within the search margin can be RANSAC rejects.
    mask[300:501:10, 529] = 255
    fit = tracker._search_previous(mask, np.array([0., 0., 500.]))
    assert np.count_nonzero(fit['_candidate_y'] >= 300) == 21
    assert np.count_nonzero(fit['_inlier_y'] >= 300) < 21


def test_joint_refit_keeps_candidate_provenance():
    tracker = PaperLaneTracker(diagnostics=True)
    ys = np.arange(530)
    left = tracker._fit_candidates(np.full(530, 100), ys)
    right = tracker._fit_candidates(np.full(530, 500), ys)
    final = _joint_refit(left, right, tracker.cfg)
    assert final is not None
    for initial, refined in zip((left, right), final):
        np.testing.assert_array_equal(initial['_candidate_y'], refined['_candidate_y'])
        np.testing.assert_array_equal(initial['_candidate_x'], refined['_candidate_x'])


def test_right_overlay_colors_and_no_mutation():
    from types import SimpleNamespace
    bev = SimpleNamespace(height=530, width=600, far_m=6., resolution_m=.01,
                          ground_to_bev_pixel=lambda x, y: (300+x*100, 600-y*100))
    raw = np.zeros((530, 600), np.uint8)
    raw[400, [100, 400, 450, 500]] = 255
    debug = np.zeros((530, 600, 3), np.uint8)
    lane = {'right': {'_candidate_x': np.array([450, 500]), '_candidate_y': np.array([400, 400]),
                      '_inlier_x': np.array([500]), '_inlier_y': np.array([400])}}
    before = copy.deepcopy(lane)
    out = live.right_near_overlay(debug, bev, raw, lane)
    assert tuple(out[400, 400]) == (255, 100, 0)
    assert tuple(out[399, 450]) == (255, 255, 0)
    assert tuple(out[400, 450]) == (0, 0, 255)
    assert tuple(out[400, 500]) == (0, 255, 0)
    assert not out[400, 100].any()
    assert not debug.any() and np.count_nonzero(raw) == 4
    for k in lane['right']:
        np.testing.assert_array_equal(lane['right'][k], before['right'][k])
