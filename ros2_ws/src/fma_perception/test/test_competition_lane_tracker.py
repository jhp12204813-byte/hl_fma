"""Competition contract tests: synthetic BEV only; no camera, GPS or vehicle."""
from pathlib import Path
import numpy as np
import pytest

from fma_perception.c920_bev import C920BEV
from fma_perception.competition_lane_tracker import CompetitionLaneTracker, MetricLaneConfig, normal_offset_curve


@pytest.fixture
def bev():
    return C920BEV(Path(__file__).resolve().parents[4] / 'config/c920_bev_calibration.yaml')


def road(bev, width=3.5, sides=('left', 'right'), shift=0., slope=0., curvature=0.):
    mask = np.zeros((bev.height, bev.width), np.uint8)
    for side in sides:
        offset = (-1 if side == 'left' else 1) * width / 2
        for row in range(20, bev.height - 1):
            y = bev.bev_pixel_to_ground(0, row)[1]
            x = shift + offset + slope*y + curvature*y*y
            col = int(round(bev.ground_to_bev_pixel(x, y)[0]))
            mask[row, max(0, col-4):min(bev.width, col+5)] = 255
    return mask


@pytest.mark.parametrize('width', [3.05, 3.2, 4.1, 4.6])
def test_same_configuration_learns_different_pair_widths(bev, width):
    tracker = CompetitionLaneTracker(bev)
    result = tracker.process(road(bev, width), timestamp=1.)
    assert result['source'] == 'PAIR_TRACK' and result['valid']
    assert result['selected_lane_width_m'] == pytest.approx(width, abs=.03)
    assert tracker.expected_lane_width_m == pytest.approx(width, abs=.03)
    assert abs(result['virtual_center']['coefficients'][-1]) < .02


@pytest.mark.parametrize('side', ['left', 'right'])
def test_single_tracks_indefinitely_without_relearning(bev, side):
    tracker = CompetitionLaneTracker(bev)
    tracker.process(road(bev, 4.1), timestamp=1.)
    learned = (tracker.expected_lane_width_m, tracker.left_offset_m, tracker.right_offset_m)
    for timestamp in (1.1, 2., 5., 30., 300.):
        result = tracker.process(road(bev, 4.1, (side,), shift=.03), timestamp=timestamp)
        assert result['valid'] and result['source'] == f'SINGLE_{side.upper()}_TRACK'
        assert result['confidence'] >= tracker.cfg.valid_confidence
        assert result['offset_source'] == 'NOMINAL'
        assert result['lateral_offset_target'] == pytest.approx(1.75)
        assert (tracker.expected_lane_width_m, tracker.left_offset_m, tracker.right_offset_m) == learned
        for key in ('virtual_center', 'boundary_heading', 'boundary_curvature', 'lateral_offset_target'):
            assert result[key] is not None


def test_local_normal_offset_is_not_constant_x_shift():
    coeff = np.array([0., .3, -2.])
    curve = normal_offset_curve(coeff, 1., 5., 1.5)
    points = curve['points_m']
    ys = curve['boundary_y_m']
    expected_x = np.polyval(coeff, ys) + 1.5 / np.sqrt(1 + .3**2)
    np.testing.assert_allclose(points[:, 0], expected_x)
    np.testing.assert_allclose(points[:, 1], ys - 1.5*.3/np.sqrt(1+.3**2))
    assert not np.allclose(points[:, 0], np.polyval(coeff, ys) + 1.5)


def test_cold_single_uses_configurable_prior_then_pair_learning(bev):
    tracker = CompetitionLaneTracker(bev, MetricLaneConfig(nominal_lane_width_m=3.2))
    first = tracker.process(road(bev, 3.2, ('right',)), timestamp=1.)
    assert first['valid'] and first['offset_source'] == 'NOMINAL'
    tracker.process(road(bev, 3.2), timestamp=2.)
    before = tracker.right_offset_m
    result = tracker.process(road(bev, 3.2, ('right',)), timestamp=100.)
    assert result['offset_source'] == 'NOMINAL'
    assert result['lateral_offset_target'] == pytest.approx(1.6)
    assert tracker.right_offset_m == before


def test_chunky_curb_blocks_rejected(bev):
    mask = road(bev, 3.5)
    for row in range(40, 490, 60):
        mask[row:row+25, 520:550] = 255
    tracker = CompetitionLaneTracker(bev)
    result = tracker.process(mask, timestamp=1.)
    assert result['valid'] and result['source'] == 'PAIR_TRACK'
    assert result['selected_lane_width_m'] == pytest.approx(3.5, abs=.03)
    assert any(c['reject_reason'] == 'chunky_component' for c in result['candidates'])
    blocks = mask.copy()
    blocks[:, :510] = 0
    assert not CompetitionLaneTracker(bev).process(blocks, timestamp=1.)['valid']


def test_identity_jump_does_not_refresh_learning(bev):
    tracker = CompetitionLaneTracker(bev)
    tracker.process(road(bev), timestamp=1.)
    width = tracker.expected_lane_width_m
    jumped = tracker.process(road(bev, sides=('right',), shift=.8), timestamp=1.1)
    assert not jumped['valid']
    assert tracker.expected_lane_width_m == width


def test_gradual_width_change_uses_ema(bev):
    tracker = CompetitionLaneTracker(bev)
    tracker.process(road(bev, 3.), timestamp=1.)
    for i, width in enumerate(np.linspace(3., 3.8, 20)):
        result = tracker.process(road(bev, width), timestamp=2.+i*.1)
        assert result['source'] == 'PAIR_TRACK'
    assert 3. < tracker.expected_lane_width_m < 3.8


def test_missing_boundary_invalid_and_pair_recovery_immediate(bev):
    tracker = CompetitionLaneTracker(bev)
    tracker.process(road(bev), timestamp=1.)
    tracker.process(road(bev, sides=('left',)), timestamp=2.)
    assert not tracker.process(np.zeros((530, 600), np.uint8), timestamp=3.)['valid']
    assert tracker.process(road(bev), timestamp=4.)['source'] == 'PAIR_TRACK'


def test_optional_heading_interface_is_not_required_and_can_veto(bev):
    tracker = CompetitionLaneTracker(bev)
    assert tracker.process(road(bev, sides=('right',)), timestamp=1.)['valid']
    tracker.consistency_check = lambda result: {'valid': False, 'reason': 'heading_mismatch'}
    assert not tracker.process(road(bev, sides=('right',)), timestamp=2.)['valid']


def test_pair_ambiguity_stops_without_learning(bev):
    mask = road(bev, 3.0)
    for x in (-2.2, 2.2):
        u = int(bev.ground_to_bev_pixel(x, 0)[0])
        mask[20:bev.height-1, u-4:u+5] = 255
    tracker = CompetitionLaneTracker(bev)
    result = tracker.process(mask, timestamp=1.)
    assert not result['valid'] and result['reject_reason'] == 'ambiguous_pair'
    assert result['second_pair_score'] is not None
    assert tracker.expected_lane_width_m is None


def test_slow_lateral_motion_single_then_pair_does_not_jump_identity(bev):
    tracker = CompetitionLaneTracker(bev)
    tracker.process(road(bev, 3.), timestamp=1.)
    learned = tracker.expected_lane_width_m
    for i, shift in enumerate(np.linspace(0., .8, 20)):
        result = tracker.process(road(bev, 3., ('right',), shift=shift), timestamp=2.+i)
        assert result['valid']
    assert tracker.expected_lane_width_m == learned
    result = tracker.process(road(bev, 3., shift=.8), timestamp=30.)
    assert result['source'] == 'PAIR_TRACK'


def test_degraded_thin_dashes_are_valid_but_slow(bev):
    mask = road(bev, 3.5, ('right',))
    for row in range(20, 529, 100):
        mask[row:row+50] = 0
    tracker = CompetitionLaneTracker(bev)
    result = tracker.process(mask, timestamp=1.)
    assert result['valid']
    assert result['confidence'] < 1.


def test_normal_offset_fold_rejected():
    assert normal_offset_curve(np.array([1., 0., 0.]), -.1, .1, 2.) is None


def test_optional_relative_heading_freshness(bev):
    from fma_perception.competition_lane_tracker import RelativeHeadingConsistency
    check = RelativeHeadingConsistency()
    tracker = CompetitionLaneTracker(bev, consistency_check=check)
    check.update(.8, 1.)
    result = tracker.process(road(bev, sides=('right',)), timestamp=1.1)
    assert not result['valid'] and result['reject_reason'] == 'secondary_heading_mismatch'
    assert tracker.expected_lane_width_m is None
    assert tracker.process(road(bev, sides=('right',)), timestamp=2.)['valid']


def test_single_default_degraded_confidence_band(bev):
    mask = np.zeros((530, 600), np.uint8)
    mask[400:492, 460:489] = 255
    result = CompetitionLaneTracker(bev).process(mask, timestamp=1.)
    assert result['valid'] and result['confidence_grade'] == 'DEGRADED'


def test_large_width_change_penalized_without_learning(bev):
    tracker = CompetitionLaneTracker(bev)
    tracker.process(road(bev, 3.), timestamp=1.)
    result = tracker.process(road(bev, 4.1), timestamp=1.1)
    assert result['confidence_grade'] == 'DEGRADED'
    assert tracker.expected_lane_width_m == pytest.approx(3.)

def test_pair_width_below_minimum_is_not_accepted_as_pair(bev):
    tracker = CompetitionLaneTracker(bev)
    result = tracker.process(road(bev, 2.7), timestamp=1.)
    assert result['source'] != 'PAIR_TRACK'
