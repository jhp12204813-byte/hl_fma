"""Synthetic masks only: no camera, ROS publisher, or vehicle access."""
import numpy as np
import pytest

from fma_perception.paper_lane_tracker import PaperLaneConfig, PaperLaneTracker


def mask(*lines):
    image = np.zeros((180, 600), dtype=np.uint8)
    for x in lines:
        image[:, x-3:x+4] = 255
    return image


def positions(result):
    return [np.polyval(result[side]['coefficients'], 150) for side in ('left', 'right')]


@pytest.mark.parametrize('extra', [None, 220, 380, 15, 585])
def test_centered_pair_with_adjacent_candidate(extra):
    tracker = PaperLaneTracker()
    image = mask(100, 500, *([] if extra is None else [extra]))
    before = image.copy()
    result = tracker.process(image)
    assert result['pair_state'] == 'PAIR_VALID'
    np.testing.assert_allclose(positions(result), [100, 500], atol=1)
    assert positions(result)[0] < 300 < positions(result)[1]
    assert result['pair_quality']['median_width_px'] >= 350
    np.testing.assert_array_equal(image, before)


def test_stronger_wrong_narrow_pair_does_not_hide_valid_pair():
    image = mask(100, 240, 360, 500)
    image[:60, 97:104] = 0
    image[:60, 497:504] = 0
    result = PaperLaneTracker().process(image)
    assert result['pair_state'] == 'PAIR_VALID'
    np.testing.assert_allclose(positions(result), [100, 500], atol=1)


def test_narrow_pair_still_rejected():
    result = PaperLaneTracker().process(mask(220, 380))
    assert result['pair_state'] == 'PAIR_WEAK'
    assert result['pair_quality']['reason'] == 'lane_width_too_narrow'
    assert result['center'] is None
    assert PaperLaneConfig().min_pair_width_px == 350.0


@pytest.mark.parametrize('visible,source', [(100, 'LEFT_ONLY_OFFSET'), (500, 'RIGHT_ONLY_OFFSET')])
def test_single_line_fallback_expires_and_recovers(visible, source):
    tracker = PaperLaneTracker()
    tracker.process(mask(100, 500))
    for _ in range(tracker.cfg.reacquire_after_single_frames):
        result = tracker.process(mask(visible))
        assert result['center'] is None
        assert result.get('pair_state') != 'PAIR_VALID'
        assert result['center_source'] == source
        assert np.polyval(result['temporary_center']['coefficients'], 150) == pytest.approx(300, abs=1)
    result = tracker.process(mask(visible))
    assert result['temporary_center'] is None
    assert result['center_source'] == 'NONE'
    assert result.get('pair_state') != 'PAIR_VALID'
    assert tracker.process(mask(100, 500))['pair_state'] == 'PAIR_VALID'


def test_no_fallback_without_confirmed_pair_or_after_empty_gap():
    tracker = PaperLaneTracker()
    assert tracker.process(mask(100))['temporary_center'] is None
    tracker.process(mask(100, 500))
    for _ in range(tracker.cfg.reacquire_after_single_frames):
        tracker.process(mask())
    assert tracker.process(mask(100))['temporary_center'] is None
    tracker.reset()
    assert tracker.process(mask(100))['temporary_center'] is None


def test_locked_pair_ignores_new_more_centered_pair():
    tracker = PaperLaneTracker()
    tracker.process(mask(70, 460))
    for _ in range(5):
        result = tracker.process(mask(70, 140, 460, 530))
        assert result['pair_state'] == 'PAIR_VALID'
        np.testing.assert_allclose(positions(result), [70, 460], atol=1)


def test_missing_locked_boundary_does_not_immediately_switch_to_adjacent():
    tracker = PaperLaneTracker()
    tracker.process(mask(70, 460))
    for _ in range(4):
        result = tracker.process(mask(70, 540))
        assert result['pair_state'] == 'PAIR_WEAK'
        assert result['pair_quality']['reason'] == 'pair_lock_mismatch'
        assert result['center'] is None
    recovered = tracker.process(mask(70, 460, 540))
    assert recovered['pair_state'] == 'PAIR_VALID'
    np.testing.assert_allclose(positions(recovered), [70, 460], atol=1)


def test_pair_does_not_enclose_vehicle_is_never_valid():
    tracker = PaperLaneTracker()
    result = tracker.process(mask(20, 240))
    assert result.get('pair_state') != 'PAIR_VALID'
    assert result['center'] is None


def test_configured_vehicle_origin():
    tracker = PaperLaneTracker(PaperLaneConfig(vehicle_center_x_px=200))
    result = tracker.process(mask(10, 390, 580))
    assert result['pair_state'] == 'PAIR_VALID'
    np.testing.assert_allclose(positions(result), [10, 390], atol=1)


def test_lock_can_reacquire_after_bounded_loss():
    tracker = PaperLaneTracker()
    tracker.process(mask(70, 460))
    for _ in range(tracker.cfg.reacquire_after_single_frames):
        assert tracker.process(mask(70, 540))['pair_state'] == 'PAIR_WEAK'
    result = tracker.process(mask(70, 540))
    assert result['pair_state'] == 'PAIR_VALID'
    np.testing.assert_allclose(positions(result), [70, 540], atol=1)


def test_single_adjacent_line_cannot_use_locked_width():
    tracker = PaperLaneTracker()
    tracker.process(mask(100, 500))
    result = tracker.process(mask(220))
    assert result['temporary_center'] is None
    assert result['center'] is None
