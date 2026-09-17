from fma_mission.waypoint_progress import (
    DenseRouteProgress,
    Waypoint,
)


def route():
    return tuple(
        Waypoint(
            id=f'P{i:03d}',
            latitude=37.0 + i * 0.00001,
            longitude=127.0,
            activation_radius_m=2.0,
            missions=(),
            number=i,
            role='dense_route',
            waypoint_type='route',
            mission=None,
            recommended_control_mode='GPS',
            hard_point=True,
            phase=None,
            completion_policy='gps',
        )
        for i in range(1, 5)
    )


def test_dense_start_and_segment_target():
    progress = DenseRouteProgress(route())

    assert progress.state == 'START'

    progress.start()

    assert progress.state == 'NORMAL_DRIVE'

    changed = progress.route_progress(
        segment_index=0,
        progress_s_m=3.0,
        total_length_m=15.0,
        finished=False,
    )

    assert changed
    assert progress.target.id == 'P002'
    assert progress.progress_s_m == 3.0


def test_dense_progress_never_moves_backward():
    progress = DenseRouteProgress(route())
    progress.start()

    progress.route_progress(
        1, 7.0, 15.0, False
    )

    changed = progress.route_progress(
        0, 4.0, 15.0, False
    )

    assert not changed
    assert progress.segment_index == 1
    assert progress.progress_s_m == 7.0
    assert progress.target.id == 'P003'


def test_dense_finish_does_not_depend_on_number_90():
    progress = DenseRouteProgress(route())
    progress.start()

    progress.route_progress(
        segment_index=2,
        progress_s_m=14.5,
        total_length_m=15.0,
        finished=True,
    )

    assert progress.state == 'FINISH'
    assert progress.target.id == 'P004'
    assert progress.target.number == 4
