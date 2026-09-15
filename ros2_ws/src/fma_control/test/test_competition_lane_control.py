"""Competition control contracts: no physical camera, serial or DDS."""
import numpy as np
from test_lane_follow import env, lane
from test_lane_stop_test import fresh
from fma_control import lane_stop_test_node as stop


def test_competition_default_speed_and_stop_priority(env):
    node = env.make(_node_class=lambda: stop.LaneStopTestNode(competition_tracking=True))
    assert node.options['drive_pwm'] == 120
    assert node.options['single_drive_pwm'] == 64
    assert node.options['degraded_pwm'] == 40
    assert node.publisher is None
    env.camera.assert_not_called()


def test_competition_obstacle_and_emergency_stop(env):
    from types import SimpleNamespace
    node = env.make(_node_class=lambda: stop.LaneStopTestNode(competition_tracking=True), enable_drive=True)
    fresh(node)
    node.tick()
    node.on_obstacle(SimpleNamespace(data=True))
    node.tick()
    assert node.publisher.publish.call_args.args[0].drive_pwm == 0
    node.on_obstacle(SimpleNamespace(data=False))
    node.on_emergency(SimpleNamespace(emergency_stop=True))
    fresh(node)
    node.tick()
    assert node.publisher.publish.call_args.args[0].drive_pwm == 0


def test_competition_blend_does_not_change_validity_or_control_window():
    from fma_control.competition_lane_control import PairReturnBlend
    blend = PairReturnBlend(.3)
    single = {'valid': True, 'tracking_source': 'SINGLE_LEFT_TRACK', 'steering_cmd_rad': -.08, 'used_lookahead': 2.}
    blend.apply(single, 1.)
    pair = {**single, 'tracking_source': 'PAIR_TRACK', 'steering_cmd_rad': .08}
    first = blend.apply(pair, 1.1)
    assert first['steering_cmd_rad'] == -.08
    assert blend.apply(pair, 1.25)['steering_cmd_rad'] < .08
    assert blend.apply(pair, 1.5)['steering_cmd_rad'] == .08
    assert first['used_lookahead'] == 2.
    assert not blend.apply({'valid': False}, 1.6)['valid']


def test_competition_single_pwm_policy_and_existing_acquisition(env):
    node = env.make(_node_class=lambda: stop.LaneStopTestNode(competition_tracking=True), enable_drive=True)
    for i in range(5):
        env.now[0] += .03
        node.stop_snapshot = (env.now[0], {})
        # The adapter's single uses the existing temporary-center distance gate.
        result = stop.calculate_command(lane(node.bev), node.bev)
        result.update(control_source='TEMPORARY_CENTER', requested_lane_pwm=64,
                      tracking_source='SINGLE_RIGHT_TRACK', tracking_confidence=.9)
        node.observe_motion_frame(result, env.now[0], env.now[0])
        node.latest = (env.now[0], result, 30.)
        node.tick()
        assert node.publisher.publish.call_args.args[0].drive_pwm == (64 if i == 4 else 0)
    result['requested_lane_pwm'] = 40
    node.tick()
    assert node.publisher.publish.call_args.args[0].drive_pwm == 40
    node.stop_latched.set()
    node.tick()
    assert node.publisher.publish.call_args.args[0].drive_pwm == 0


def test_competition_real_process_and_no_boundary_loss_timeout(env):
    from unittest.mock import MagicMock
    node = env.make(_node_class=lambda: stop.LaneStopTestNode(competition_tracking=True))
    mask = np.zeros((node.bev.height, node.bev.width), np.uint8)
    mask[20:529, 471:480] = 255
    node.make_mask = MagicMock(return_value=(None, mask, mask))
    for timestamp in (100., 101., 105., 200.):
        env.now[0] = timestamp
        result, _ = node.process_frame(object())
        assert result['valid'] and result['tracking_source'] == 'SINGLE_RIGHT_TRACK'
        assert result['requested_lane_pwm'] == 64
        assert 1.8 <= result['used_lookahead'] <= 3.30
    mask[:] = 0
    env.now[0] += .03
    result, _ = node.process_frame(object())
    assert not result['valid']
    env.camera.assert_not_called()


import pytest

@pytest.mark.parametrize('side', ['left', 'right'])
def test_far_pair_uses_near_selected_boundary(env, side):
    from fma_control.competition_lane_control import calculate_competition_command
    node = env.make(_node_class=lambda: stop.LaneStopTestNode(competition_tracking=True), enable_drive=True)
    tracker = node.competition_tracker
    mask = np.zeros((node.bev.height, node.bev.width), np.uint8)
    mask[20:529, 121:130] = 255
    mask[20:529, 471:480] = 255
    tracking = tracker.process(mask, timestamp=1.)
    assert tracking['source'] == 'PAIR_TRACK'
    tracking['virtual_center']['y_min_m'] = 4.05
    tracking['right' if side == 'left' else 'left']['y_min_m'] = 4.05
    result, stop_lane = calculate_competition_command(tracking, tracker, node.follow_config, node.options)
    assert result['valid']
    assert result['control_source'] == f'SINGLE_{side.upper()}_TRACK'
    assert result['tracking_source'] == 'PAIR_TRACK'
    assert result['observed_min'] < 3.3
    assert result['requested_lane_pwm'] == 64
    assert stop_lane['pair_state'] == 'PAIR_VALID'
    for _ in range(5):
        env.now[0] += .03
        node.stop_snapshot = (env.now[0], {})
        node.observe_motion_frame(result, env.now[0], env.now[0])
        node.latest = (env.now[0], result, 30.)
        node.tick()
    assert node.publisher.publish.call_args.args[0].drive_pwm == 64
    assert node.published_pwm == 64
    node.stop_snapshot = (env.now[0], {'stop_detected': True, 'stop_confirmed': True,
                          'stop_near_edge_m': 1.6, 'stop_confidence': .9})
    node.tick()
    assert node.test_state == 'STOP_APPROACH' and node.published_pwm == 40
    if side == 'left':
        node.stop_snapshot[1]['stop_near_edge_m'] = .3
        node.tick()
        assert node.test_state == 'STOP_LINE' and node.published_pwm == 0
    else:
        from types import SimpleNamespace
        node.on_emergency(SimpleNamespace(emergency_stop=True))
        node.tick()
        assert node.test_state == 'INVALID' and node.published_pwm == 0
    diagnostic = node.diagnostic(result, 30.)
    assert f'motion_control_source=SINGLE_{side.upper()}_TRACK' in diagnostic
    assert 'requested_pwm=64' in diagnostic and 'published_pwm=0' in diagnostic


@pytest.mark.parametrize('kind,pwm', [('PAIR_TRACK',120), ('DEGRADED',40), ('OUTSIDE',0), ('NONE',0)])
def test_selected_geometry_pwm(env, kind, pwm):
    from fma_control.competition_lane_control import calculate_competition_command
    node = env.make(_node_class=lambda: stop.LaneStopTestNode(competition_tracking=True))
    tracker = node.competition_tracker
    mask = np.zeros((node.bev.height, node.bev.width), np.uint8)
    if kind != 'NONE':
        mask[20:529, 471:480] = 255
    if kind in ('PAIR_TRACK', 'DEGRADED'):
        mask[20:529, 121:130] = 255
    tracking = tracker.process(mask, timestamp=1.)
    if kind == 'DEGRADED':
        tracking['confidence_grade'] = 'DEGRADED'
    if kind == 'OUTSIDE':
        tracking['virtual_center']['y_min_m'] = 4.05
    result, _ = calculate_competition_command(tracking, tracker, node.follow_config, node.options)
    assert result['valid'] == (pwm > 0)
    assert result['requested_lane_pwm'] == pwm
    if kind == 'PAIR_TRACK':
        assert result['control_source'] == 'PAIR_TRACK'
