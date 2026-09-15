"""Field drive safety via mocked ROS/camera, real existing stop confirmation."""
from unittest.mock import MagicMock
import numpy as np
import pytest

from test_lane_follow import env, lane
from fma_control import lane_stop_test_node as stop
from fma_control.lane_follow_node import calculate_command


def make(env, enabled=True):
    return env.make(_node_class=stop.LaneStopTestNode, enable_drive=enabled, lane_min_width_m=3.0)


def fresh(node, now=100.):
    result = calculate_command(lane(node.bev, .2), node.bev)
    node.stop_snapshot = (now-.12, {})
    for i in range(5):
        stamp = now-.12+i*.03
        node.observe_motion_frame(result, stamp, stamp)
    node.stop_snapshot = (now, {})
    node.latest = (now, result, 30.)
    return result


def test_normal_40_steering_and_cleanup(env):
    node = make(env)
    node.tick()
    assert node.test_state == 'ACQUIRING'
    assert node.publisher.publish.call_args.args[0].emergency_stop
    result = fresh(node)
    node.tick()
    msg = node.publisher.publish.call_args.args[0]
    assert msg.drive_pwm == 40 and msg.pwm_control and msg.speed_mps > 0
    assert msg.steering_angle_rad == pytest.approx(result['steering_cmd_rad'])
    node.destroy_node()
    assert all(c.args[0].emergency_stop and c.args[0].drive_pwm == 0
               for c in node.publisher.publish.call_args_list[-3:])


@pytest.mark.parametrize('failure', ['lane', 'stale', 'camera'])
def test_failure_stops_and_latches(env, failure):
    node = make(env)
    fresh(node)
    node.tick()
    if failure == 'lane': node.latest = (100., {'valid': False, 'reason': 'invalid_pair',
        'lateral_error_m': None, 'heading_error_deg': None, 'steering_cmd_rad': 0., 'expected_adc': None}, 30.)
    if failure == 'stale': env.now[0] = 100.3
    if failure == 'camera': node.failure = 'camera read failed'
    node.tick()
    assert node.test_state == 'INVALID'
    assert node.publisher.publish.call_args.args[0].drive_pwm == 0
    node.failure = None
    env.now[0] += .2
    fresh(node, env.now[0])
    node.tick()
    assert node.publisher.publish.call_args.args[0].emergency_stop == (failure == 'camera')


def test_existing_stop_confirmation_shared_frame_and_latch(env, monkeypatch):
    node = make(env)
    frame = np.zeros((480, 640, 3), np.uint8)
    lane_mask, white = object(), object()
    observation = lane(node.bev)
    node.make_mask = MagicMock(return_value=(None, lane_mask, white))
    monkeypatch.setattr(stop, 'process_lane', MagicMock(return_value=(observation, 0)))
    detection = {'stop_line_detected': True, 'stop_line_distance_m': .8,
                 'stop_line_confidence': .9, 'candidates': [{'distance_m': .8, 'height': 46}]}
    node.stop_detector = MagicMock()
    node.stop_detector.detect.return_value = detection
    for i in range(node.stop_tracker.cfg.confirm_frames):
        result, _ = node.process_frame(frame)
        node.process_stop(*node.stop_pending)
        assert node.stop_latched.is_set() == (i == node.stop_tracker.cfg.confirm_frames-1)
    assert node.stop_snapshot[1]['stop_near_edge_m'] == pytest.approx(.57)
    node.make_mask.assert_called_with(frame, node.bev)
    stop.process_lane.assert_called_with(node.tracker, lane_mask)
    node.stop_detector.detect.assert_called_with(white, observation)
    # Even if snapshot is replaced by a clear frame, the worker-set event survives.
    fresh(node)
    for _ in range(3):
        node.tick()
        assert node.test_state == 'STOP_LINE'
        assert node.publisher.publish.call_args.args[0].emergency_stop
        assert node.publisher.publish.call_args.args[0].drive_pwm == 0
    env.camera.assert_not_called()


def test_dry_run_stop_latch_no_publish(env):
    node = make(env, False)
    fresh(node)
    node.tick()
    node.stop_latched.set()
    node.tick()
    assert node.test_state == 'STOP_LINE'
    node.destroy_node()
    env.publisher.assert_not_called()


def test_main_ctrl_c_cleanup(env, monkeypatch):
    node = make(env)
    fresh(node)
    node.tick()
    monkeypatch.setattr(stop, 'LaneStopTestNode', lambda: node)
    monkeypatch.setattr(stop.rclpy, 'init', MagicMock())
    monkeypatch.setattr(stop.rclpy, 'spin', MagicMock(side_effect=KeyboardInterrupt))
    monkeypatch.setattr(stop.rclpy, 'ok', lambda: True)
    monkeypatch.setattr(stop.rclpy, 'shutdown', MagicMock())
    stop.main()
    assert all(c.args[0].drive_pwm == 0 for c in node.publisher.publish.call_args_list[-3:])


def test_stop_latch_through_existing_raw_command_path(env, monkeypatch):
    from fma_control.command_arbiter_node import CommandArbiterNode
    from fma_vehicle.vehicle_controller_node import VehicleControllerNode
    from fma_vehicle import stm32_bridge_node as bridge
    node = make(env)
    arbiter = CommandArbiterNode()
    arbiter.config['allow_lane_pwm'] = True
    controller = VehicleControllerNode()
    wire = MagicMock()
    wire.write.side_effect = len
    monkeypatch.setattr(bridge.serial, 'Serial', MagicMock(return_value=wire))
    stm32 = bridge.STM32BridgeNode()
    node.publisher = MagicMock()
    arbiter.publisher = MagicMock()
    controller.publisher = MagicMock()
    node.publisher.publish.side_effect = arbiter.on_lane
    arbiter.publisher.publish.side_effect = controller.on_command
    controller.publisher.publish.side_effect = stm32.on_command
    fresh(node)
    node.tick()
    arbiter.publish_output()
    assert wire.write.call_args.args[0] == b'F0040'
    node.stop_latched.set()
    for _ in range(3):
        env.now[0] += .05  # existing bridge enforces a 20 ms serial interval
        fresh(node, env.now[0])
        node.tick()
        arbiter.publish_output()
        assert wire.write.call_args.args[0] == b'X'


@pytest.mark.parametrize('used', [1.7, 3.51, 4., 5.6, float('nan'), None])
def test_pwm_blocked_outside_control_window(env, used):
    node = make(env)
    result = fresh(node)
    result['used_lookahead'] = used
    node.tick()
    assert node.publisher.publish.call_args.args[0].drive_pwm == 0


def test_five_real_lane_frames_required_no_stop_in_lane_thread(env, monkeypatch):
    node = make(env)
    node.make_mask = MagicMock(return_value=(None, object(), object()))
    monkeypatch.setattr(stop, 'process_lane', MagicMock(return_value=(lane(node.bev), 0)))
    node.stop_detector = MagicMock()
    node.stop_snapshot = (100., {})
    for i in range(5):
        env.now[0] += .03
        result, _ = node.process_frame(object(), frame_stamp=env.now[0])
        node.latest = (env.now[0], result, 30.)
        node.tick()
        msg = node.publisher.publish.call_args.args[0]
        assert msg.drive_pwm == (40 if i == 4 else 0)
        node.tick()  # repeated timer ticks do not acquire another frame
        assert node.publisher.publish.call_args.args[0].drive_pwm == msg.drive_pwm
    node.stop_detector.detect.assert_not_called()


def test_stop_snapshot_stale_blocks_drive(env):
    node = make(env)
    fresh(node)
    node.stop_snapshot = (99., {})
    node.tick()
    assert node.publisher.publish.call_args.args[0].drive_pwm == 0


def test_late_stop_result_not_confirmed_or_cached(env):
    node = make(env)
    node.stop_detector = MagicMock()
    node.process_stop(99., object(), {})
    assert node.stop_snapshot is None
    assert not node.stop_latched.is_set()


@pytest.mark.parametrize('near,triggered', [(.61, False), (.60, True), (.59, True)])
def test_near_edge_distance_boundary(env, near, triggered):
    node = make(env)
    node.stop_detector = MagicMock()
    node.stop_detector.detect.return_value = {
        'stop_line_detected': True, 'stop_line_distance_m': near + .1,
        'stop_line_confidence': .9, 'candidates': [{'distance_m': near + .1, 'height': 20}]}
    for _ in range(node.stop_tracker.cfg.confirm_frames):
        node.process_stop(100., object(), {})
    assert node.stop_latched.is_set() == triggered
    fresh(node)
    node.tick()
    assert node.publisher.publish.call_args.args[0].drive_pwm == (0 if triggered else 40)


def test_far_then_near_then_missing_keeps_stop(env):
    node = make(env)
    node.stop_detector = MagicMock()
    for near in (3., 3., 3., .59):
        node.stop_detector.detect.return_value = {
            'stop_line_detected': True, 'stop_line_distance_m': near + .1,
            'stop_line_confidence': .9, 'candidates': [{'distance_m': near + .1, 'height': 20}]}
        node.process_stop(100., object(), {})
        assert node.stop_latched.is_set() == (near < .6)
    node.stop_detector.detect.return_value = {
        'stop_line_detected': False, 'stop_line_distance_m': None,
        'stop_line_confidence': 0., 'candidates': []}
    node.process_stop(100., object(), {})
    fresh(node)
    node.tick()
    assert node.test_state == 'STOP_LINE'
    assert node.publisher.publish.call_args.args[0].drive_pwm == 0
    assert 'stop_triggered=True' in node.diagnostic({'valid': True}, 30.)


@pytest.mark.parametrize('value', [0., -1., float('nan'), float('inf')])
def test_bad_trigger_parameter_rejected_before_hardware(env, value):
    with pytest.raises(ValueError):
        env.make(_node_class=stop.LaneStopTestNode, stop_trigger_distance_m=value)
    env.camera.assert_not_called()
    env.publisher.assert_not_called()


def test_custom_trigger_and_unconfirmed_near_does_not_stop(env):
    node = env.make(_node_class=stop.LaneStopTestNode, stop_trigger_distance_m=1.0)
    node.stop_detector = MagicMock()
    node.stop_detector.detect.return_value = {
        'stop_line_detected': True, 'stop_line_distance_m': .9,
        'stop_line_confidence': .9, 'candidates': [{'distance_m': .9, 'height': 20}]}
    node.process_stop(100., object(), {})
    assert not node.stop_latched.is_set()
    for _ in range(2): node.process_stop(100., object(), {})
    assert node.stop_latched.is_set()


@pytest.mark.parametrize('side', ['LEFT_ONLY', 'RIGHT_ONLY'])
@pytest.mark.parametrize('enabled', [False, True])
def test_single_side_opt_in_existing_temporary_center(env, monkeypatch, side, enabled):
    node = env.make(_node_class=stop.LaneStopTestNode, allow_single_side_test=enabled)
    center = lane(node.bev)['center']
    observation = {'state': side, 'temporary_center': center, 'center_source': side+'_OFFSET',
                   side.split('_')[0].lower(): {**center, 'points': 100, 'residual_px': 1.}}
    node.make_mask = MagicMock(return_value=(None, object(), object()))
    monkeypatch.setattr(stop, 'process_lane', MagicMock(return_value=(observation, 0)))
    result, _ = node.process_frame(object())
    assert result['valid'] == enabled
    assert 'pair_state' not in observation
    assert node.stop_pending[2] is observation


def test_temporary_expiry_stops_until_pair_reacquisition(env):
    node = env.make(_node_class=stop.LaneStopTestNode, allow_single_side_test=True, enable_drive=True)
    mask = np.zeros((node.bev.height, node.bev.width), np.uint8)
    mask[:, 97:104] = 255
    mask[:, 497:504] = 255
    node.make_mask = MagicMock(side_effect=lambda frame, bev: (None, mask.copy(), mask.copy()))
    for _ in range(5):
        env.now[0] += .03
        node.stop_snapshot = (env.now[0], {})
        result, _ = node.process_frame(object())
        node.latest = (env.now[0], result, 30.)
        node.tick()
    assert result['control_source'] == 'PAIR'
    mask[:, 97:104] = 0
    for _ in range(node.tracker.cfg.reacquire_after_single_frames):
        env.now[0] += .03
        result, _ = node.process_frame(object())
        assert result['valid'] and result['control_source'] == 'TEMPORARY_CENTER'
    env.now[0] += .03
    result, _ = node.process_frame(object())
    assert node.tracker.last_valid_pair is not None
    assert not result['valid'] and result['steering_cmd_rad'] == 0
    node.latest = (env.now[0], result, 30.)
    node.tick()
    assert node.publisher.publish.call_args.args[0].drive_pwm == 0
    mask[:, 97:104] = 255
    result, _ = node.process_frame(object())
    assert result['control_source'] == 'PAIR'


@pytest.mark.parametrize('height,detected', [(6, False), (44, True), (45, True), (48, True)])
def test_field_stop_thickness_with_real_detector(env, height, detected):
    node = env.make(_node_class=stop.LaneStopTestNode, stop_min_thickness_m=.30)
    mask = np.zeros((node.bev.height, node.bev.width), np.uint8)
    mask[330:330+height, 100:501] = 255
    for _ in range(3): node.process_stop(100., mask, lane(node.bev))
    assert node.stop_snapshot[1]['stop_detected'] == detected
    assert node.stop_snapshot[1]['stop_confirmed'] == detected
    assert stop.StopLineConfig().min_thickness_m == .04
    assert node.stop_detector.config.max_thickness_m == .55


@pytest.mark.parametrize('value', [0., -.1, .56, float('nan')])
def test_invalid_thickness_before_hardware(env, value):
    with pytest.raises(ValueError):
        env.make(_node_class=stop.LaneStopTestNode, stop_min_thickness_m=value)
    env.camera.assert_not_called()
    env.publisher.assert_not_called()


@pytest.mark.parametrize('source,used,allowed', [('SINGLE_RIGHT', 3.08, False),
    ('SINGLE_LEFT', 3.30, False), ('SINGLE_RIGHT', 3.31, False),
    ('PAIR', 3.08, True), ('TEMPORARY_CENTER', 3.08, True)])
def test_extended_window_publish_guard(env, source, used, allowed):
    node = env.make(_node_class=stop.LaneStopTestNode, allow_single_side_test=True, enable_drive=True)
    result = fresh(node)
    result.update(control_source=source, used_lookahead=used)
    node.tick()
    assert node.publisher.publish.call_args.args[0].drive_pwm == (40 if allowed else 0)


@pytest.mark.parametrize('value', [5.21, 6., float('inf'), float('nan'), 1.7])
def test_control_max_rejected_before_hardware(env, value):
    with pytest.raises(ValueError):
        env.make(_node_class=stop.LaneStopTestNode, single_side_control_max_m=value)
    env.camera.assert_not_called()
    env.publisher.assert_not_called()


@pytest.mark.parametrize('transient', ['lane', 'stop_stale', 'frame_stale'])
def test_transient_reacquires_five_new_motion_safe_frames(env, transient):
    node = make(env)
    fresh(node)
    node.tick()
    assert node.test_state == 'DRIVING'
    if transient == 'lane': node.latest[1]['valid'] = False
    if transient == 'stop_stale': node.stop_snapshot = (99., {})
    if transient == 'frame_stale': env.now[0] += .3
    node.tick()
    assert node.frame_valid_streak == 0 and not node.fault_latched.is_set()
    assert node.publisher.publish.call_args.args[0].drive_pwm == 0
    for i in range(5):
        env.now[0] += .03
        now = env.now[0]
        result = calculate_command(lane(node.bev), node.bev)
        node.stop_snapshot = (now, {})
        node.observe_motion_frame(result, now, now)
        node.latest = (now, result, 30.)
        for _ in range(3): node.tick()
        assert node.frame_valid_streak == i+1
        assert node.publisher.publish.call_args.args[0].drive_pwm == (40 if i == 4 else 0)


def test_stop_worker_exception_permanently_blocks(env, monkeypatch):
    node = make(env)
    node.stop_pending = (100., object(), {})
    monkeypatch.setattr(node, 'process_stop', MagicMock(side_effect=RuntimeError('worker failed')))
    node.stop_loop()
    assert node.fault_latched.is_set()
    fresh(node)
    node.tick()
    assert node.publisher.publish.call_args.args[0].drive_pwm == 0


def test_no_stop_state_cannot_accumulate_safe_streak(env):
    node = make(env)
    for i in range(8):
        stamp = 100.+i*.03
        result = calculate_command(lane(node.bev), node.bev)
        node.observe_motion_frame(result, stamp, stamp)
        assert node.frame_valid_streak == 0
    assert not node.fault_latched.is_set()


@pytest.mark.parametrize('pwm', [0, 40, 64, 799])
def test_field_pwm_parameter_and_all_stop_states(env, pwm):
    node = env.make(_node_class=stop.LaneStopTestNode, enable_drive=True, drive_pwm=pwm)
    node.tick()
    assert node.test_state == 'ACQUIRING'
    assert node.publisher.publish.call_args.args[0].drive_pwm == 0
    result = fresh(node)
    node.tick()
    assert node.test_state == 'DRIVING'
    assert node.publisher.publish.call_args.args[0].drive_pwm == pwm
    assert f'drive_pwm={pwm}' in node.diagnostic(result, 30.)
    result['valid'] = False
    node.tick()
    assert node.test_state == 'INVALID'
    assert node.publisher.publish.call_args.args[0].drive_pwm == 0
    node.stop_latched.set()
    node.tick()
    assert node.test_state == 'STOP_LINE'
    assert node.publisher.publish.call_args.args[0].drive_pwm == 0
    node.destroy_node()
    assert all(c.args[0].drive_pwm == 0 for c in node.publisher.publish.call_args_list[-3:])


@pytest.mark.parametrize('pwm', [-1, 800, 64.5, True])
def test_bad_field_pwm_before_hardware(env, pwm):
    with pytest.raises(ValueError):
        env.make(_node_class=stop.LaneStopTestNode, enable_drive=True, drive_pwm=pwm)
    env.camera.assert_not_called()
    env.publisher.assert_not_called()


def test_production_pwm_not_overridden(env):
    node = env.make(enable_drive=True, drive_pwm=64)
    assert 'drive_pwm' not in node.options
    assert node.requested_drive_pwm() == 40


@pytest.mark.parametrize('near,triggered', [(.31, False), (.30, True), (.29, True)])
def test_point30_trigger_with_64_pwm(env, near, triggered):
    node = env.make(_node_class=stop.LaneStopTestNode, enable_drive=True,
                    drive_pwm=64, stop_trigger_distance_m=.30)
    node.stop_detector = MagicMock()
    node.stop_detector.detect.return_value = {
        'stop_line_detected': True, 'stop_line_distance_m': near + .2,
        'stop_line_confidence': .96, 'candidates': [{'distance_m': near + .2, 'height': 40}]}
    for _ in range(3): node.process_stop(100., object(), {})
    fresh(node)
    node.tick()
    assert node.stop_latched.is_set() == triggered
    assert node.publisher.publish.call_args.args[0].drive_pwm == (0 if triggered else 64)


@pytest.mark.parametrize('side,x', [('RIGHT', 500), ('LEFT', 100)])
@pytest.mark.parametrize('enabled', [False, True])
def test_current_side_with_old_pair_history(env, side, x, enabled):
    node = env.make(_node_class=stop.LaneStopTestNode, allow_single_side_test=enabled)
    mask = np.zeros((node.bev.height, node.bev.width), np.uint8)
    mask[:, 97:104] = 255
    mask[:, 497:504] = 255
    node.tracker.process(mask)
    assert node.tracker.last_valid_pair is not None
    for _ in range(node.tracker.cfg.reacquire_after_single_frames):
        node.tracker.process(np.zeros_like(mask))
    mask[:] = 0
    mask[:, x-3:x+4] = 255
    node.make_mask = MagicMock(return_value=(None, mask, mask))
    result, _ = node.process_frame(object())
    assert not result['valid']
    assert result['control_source'] == 'NONE'


def approach_node(env):
    node = env.make(_node_class=stop.LaneStopTestNode, enable_drive=True,
                    drive_pwm=64, stop_trigger_distance_m=.40)
    result = fresh(node)
    node.tick()
    assert node.test_state == 'DRIVING'
    node.stop_snapshot = (100., {'stop_detected': True, 'stop_confirmed': True,
                                'stop_near_edge_m': 1.6, 'stop_confidence': .9})
    node.tick()
    assert node.test_state == 'STOP_APPROACH'
    assert node.publisher.publish.call_args.args[0].drive_pwm == 40
    return node, result


def test_approach_lane_dropout_then_stop_latch(env):
    node, result = approach_node(env)
    result['valid'] = False
    node.tick()
    assert node.publisher.publish.call_args.args[0].drive_pwm == 40
    assert abs(node.publisher.publish.call_args.args[0].steering_angle_rad) <= .10
    node.stop_snapshot[1]['stop_near_edge_m'] = .39
    node.tick()
    assert node.test_state == 'STOP_LINE' and node.stop_latched.is_set()
    for _ in range(3):
        fresh(node)
        node.tick()
        assert node.publisher.publish.call_args.args[0].drive_pwm == 0


@pytest.mark.parametrize('failure', ['stop_stale', 'lane_stale', 'timeout', 'system', 'jump', 'nan', 'missing'])
def test_approach_failure_never_restarts(env, failure):
    node, result = approach_node(env)
    if failure == 'stop_stale': node.stop_snapshot = (99., node.stop_snapshot[1])
    if failure == 'lane_stale': result['frame_stamp'] = 99.
    if failure == 'timeout': node.approach_started = 96.
    if failure == 'system': node.fault_latched.set()
    if failure == 'jump': node.stop_snapshot[1]['stop_near_edge_m'] = 2.5
    if failure == 'nan': node.stop_snapshot[1]['stop_near_edge_m'] = float('nan')
    if failure == 'missing': node.stop_snapshot[1]['stop_confirmed'] = False
    node.tick()
    assert node.test_state == 'INVALID'
    assert node.publisher.publish.call_args.args[0].drive_pwm == 0
    fresh(node)
    node.tick()
    assert node.publisher.publish.call_args.args[0].drive_pwm == 0


def test_approach_updates_valid_steering_and_cleanup(env):
    node, result = approach_node(env)
    result['steering_cmd_rad'] = .25
    node.tick()
    assert node.publisher.publish.call_args.args[0].steering_angle_rad == pytest.approx(.1)
    result['steering_cmd_rad'] = -.04
    node.tick()
    assert node.publisher.publish.call_args.args[0].steering_angle_rad == pytest.approx(-.04)
    node.destroy_node()
    assert all(c.args[0].drive_pwm == 0 for c in node.publisher.publish.call_args_list[-3:])


@pytest.mark.parametrize('side,x', [('LEFT', 100), ('RIGHT', 500)])
@pytest.mark.parametrize('enabled', [False, True])
def test_cold_single_boundary_never_drives(env, side, x, enabled):
    node = env.make(_node_class=stop.LaneStopTestNode, enable_drive=True,
                    drive_pwm=120, allow_single_side_test=enabled)
    mask = np.zeros((node.bev.height, node.bev.width), np.uint8)
    mask[:, x-3:x+4] = 255
    node.make_mask = MagicMock(return_value=(None, mask, mask))
    for _ in range(6):
        env.now[0] += .03
        node.stop_snapshot = (env.now[0], {})
        result, _ = node.process_frame(object())
        node.latest = (env.now[0], result, 30.)
        node.tick()
        assert not result['valid']
        assert node.frame_valid_streak == 0
        assert node.publisher.publish.call_args.args[0].drive_pwm == 0
    env.camera.assert_not_called()


def test_driving_one_invalid_stops_and_requires_five_new_frames(env):
    node = env.make(_node_class=stop.LaneStopTestNode, enable_drive=True, drive_pwm=120)
    fresh(node)
    node.tick()
    assert node.publisher.publish.call_args.args[0].drive_pwm == 120
    env.now[0] += .03
    result = calculate_command({}, node.bev)
    node.observe_motion_frame(result, env.now[0], env.now[0])
    node.latest = (env.now[0], result, 30.)
    node.tick()
    assert node.test_state == 'INVALID' and node.frame_valid_streak == 0
    assert node.publisher.publish.call_args.args[0].drive_pwm == 0
    for i in range(5):
        env.now[0] += .03
        node.stop_snapshot = (env.now[0], {})
        result = calculate_command(lane(node.bev), node.bev)
        node.observe_motion_frame(result, env.now[0], env.now[0])
        node.latest = (env.now[0], result, 30.)
        for _ in range(3):
            node.observe_motion_frame(result, env.now[0], env.now[0])
            node.tick()
        assert node.frame_valid_streak == i+1
        assert node.publisher.publish.call_args.args[0].drive_pwm == (120 if i == 4 else 0)


@pytest.mark.parametrize('source', ['SINGLE_LEFT', 'SINGLE_RIGHT', 'MEMORY_LEFT', 'MEMORY_RIGHT', 'NONE'])
def test_removed_sources_cannot_pass_pwm_guard(env, source):
    node = env.make(_node_class=stop.LaneStopTestNode, enable_drive=True, allow_single_side_test=True)
    result = fresh(node)
    result['control_source'] = source
    node.tick()
    assert node.publisher.publish.call_args.args[0].drive_pwm == 0


def test_experimental_options_not_declared(env):
    node = env.make(_node_class=stop.LaneStopTestNode)
    assert not any('memory' in key or 'lane_hold' in key for key in node.options)
    assert 'single_side_lane_width_m' not in node.options
    env.camera.assert_not_called()


@pytest.mark.parametrize('maximum', [1.79, 4., 5., float('inf'), float('nan')])
def test_pair_max_validation_before_hardware(env, maximum):
    with pytest.raises(ValueError, match='pair_control_max_m'):
        env.make(_node_class=stop.LaneStopTestNode, pair_control_max_m=maximum)
    env.camera.assert_not_called()
    env.publisher.assert_not_called()


def test_production_pair_max_default_and_override_rejection(env):
    assert env.make().options['pair_control_max_m'] == 3.
    with pytest.raises(ValueError, match='field-test only'):
        env.make(pair_control_max_m=3.5)


def test_field_pair_max_passed_to_calculation_and_five_frame_gate(env, monkeypatch):
    node = env.make(_node_class=stop.LaneStopTestNode, enable_drive=True, drive_pwm=120)
    assert node.options['pair_control_max_m'] == 3.5
    observation = lane(node.bev)
    observation['center']['y_max'] = node.bev.ground_to_bev_pixel(0, 3.39)[1]
    node.make_mask = MagicMock(return_value=(None, object(), object()))
    monkeypatch.setattr(stop, 'process_lane', lambda *args: (observation, 0))
    for i in range(5):
        env.now[0] += .03
        node.stop_snapshot = (env.now[0], {})
        result, _ = node.process_frame(object())
        node.latest = (env.now[0], result, 30.)
        node.tick()
        assert result['valid'] and result['used_lookahead'] == pytest.approx(3.49)
        assert node.publisher.publish.call_args.args[0].drive_pwm == (120 if i == 4 else 0)
    assert 'pair_control_max_m=3.50' in node.diagnostic(result, 30.)
    observation['center']['y_max'] = node.bev.ground_to_bev_pixel(0, 3.51)[1]
    env.now[0] += .03
    result, _ = node.process_frame(object())
    node.latest = (env.now[0], result, 30.)
    node.tick()
    assert not result['valid'] and node.frame_valid_streak == 0
    assert node.publisher.publish.call_args.args[0].drive_pwm == 0
    env.camera.assert_not_called()


@pytest.mark.parametrize('observed_min', [3.51, float('nan'), None])
def test_pair_observed_min_rechecked_before_pwm(env, observed_min):
    node = env.make(_node_class=stop.LaneStopTestNode, enable_drive=True)
    result = fresh(node)
    result['observed_min'] = observed_min
    node.tick()
    assert node.publisher.publish.call_args.args[0].drive_pwm == 0
