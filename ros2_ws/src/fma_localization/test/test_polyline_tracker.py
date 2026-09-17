import pytest

from fma_localization.polyline_tracker import (
    PolylineTracker,
    RoutePoint,
)


def straight_route():
    return PolylineTracker([
        RoutePoint(37.0, 127.0),
        RoutePoint(37.0001, 127.0),
        RoutePoint(37.0002, 127.0),
    ])


def test_straight_route_lookahead_moves_forward():
    tracker = straight_route()

    result = tracker.track(
        37.00005,
        127.0,
        lookahead_m=2.0,
    )

    assert result.projection.distance_m < 0.05
    assert result.target.s_m > result.progress_s_m
    assert result.target.latitude > 37.00005
    assert not result.finished


def test_off_route_projects_to_nearest_segment():
    tracker = straight_route()

    result = tracker.track(
        37.00005,
        127.00001,
        lookahead_m=2.0,
    )

    assert result.projection.distance_m > 0.5
    assert result.projection.distance_m < 1.5
    assert result.projection.segment_index == 0


def test_progress_never_moves_backward():
    tracker = straight_route()

    first = tracker.track(
        37.00015,
        127.0,
        lookahead_m=2.0,
    )

    second = tracker.track(
        37.00005,
        127.0,
        previous_s_m=first.progress_s_m,
        previous_segment=first.projection.segment_index,
        lookahead_m=2.0,
    )

    assert second.progress_s_m == pytest.approx(
        first.progress_s_m
    )


def test_lookahead_crosses_corner():
    tracker = PolylineTracker([
        RoutePoint(37.0, 127.0),
        RoutePoint(37.00005, 127.0),
        RoutePoint(37.00005, 127.0001),
    ])

    corner_s = tracker.cumulative_s[1]

    target = tracker.point_at_s(corner_s + 2.0)

    assert target.segment_index == 1
    assert target.longitude > 127.0


def test_target_clamps_to_route_end():
    tracker = straight_route()

    target = tracker.point_at_s(
        tracker.total_length_m + 100.0
    )

    assert target.s_m == pytest.approx(
        tracker.total_length_m
    )
    assert target.latitude == pytest.approx(37.0002)
    assert target.longitude == pytest.approx(127.0)


def test_finish_near_last_point():
    tracker = straight_route()

    result = tracker.track(
        37.0002,
        127.0,
        previous_s_m=tracker.total_length_m - 0.5,
        previous_segment=1,
        lookahead_m=1.8,
        finish_tolerance_m=1.0,
    )

    assert result.finished


def test_rejects_too_close_consecutive_points():
    with pytest.raises(ValueError):
        PolylineTracker([
            RoutePoint(37.0, 127.0),
            RoutePoint(37.0, 127.0),
        ])
