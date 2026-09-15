"""Controller and ROS callbacks without camera, DDS, serial or motor access."""
import copy
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import MagicMock
import warnings

import numpy as np
import pytest
from builtin_interfaces.msg import Time
from fma_control import lane_follow_node as follow
from fma_perception.c920_bev import C920BEV

REPO = Path(__file__).resolve().parents[4]


@pytest.fixture
def bev():
    return C920BEV(REPO / 'config/c920_bev_calibration.yaml')


def lane(bev, x=0, slope=0):
    u, v = bev.ground_to_bev_pixel(x, 2)
    return {'pair_state': 'PAIR_VALID', 'pair_quality': {'valid': True},
            'center': {'coefficients': np.array([0., slope, u-slope*v]),
                       'y_min': 0, 'y_max': bev.height-1}}


@pytest.mark.parametrize('x,sign', [(0, 0), (.5, -1), (-.5, 1)])
def test_lateral_sign(bev, x, sign):
    result = follow.calculate_command(lane(bev, x), bev)
    assert result['valid']
    assert result['lateral_error_m'] == pytest.approx(-x)
    assert np.sign(result['steering_cmd_rad']) == sign
    assert result['heading_error_deg'] == 0


@pytest.mark.parametrize('slope', [-.2, .2])
def test_heading_sign(bev, slope):
    result = follow.calculate_command(lane(bev, slope=slope), bev)
    assert result['heading_error_deg'] == pytest.approx(np.degrees(np.arctan(-slope)))
    assert np.sign(result['steering_cmd_rad']) == np.sign(-slope)


@pytest.mark.parametrize('x,expected,adc', [(-2, .2810, 3950), (2, -.3054, 150)])
def test_saturation(bev, x, expected, adc):
    result = follow.calculate_command(lane(bev, x), bev)
    assert result['steering_cmd_rad'] == expected
    assert result['expected_adc'] == adc


@pytest.mark.parametrize('kind', ['weak', 'invalid_quality', 'missing', 'unsupported', 'nan', 'outside'])
def test_invalid_no_trusted_steering(bev, kind):
    result = lane(bev)
    if kind == 'weak': result['pair_state'] = 'PAIR_WEAK'
    if kind == 'invalid_quality': result['pair_quality']['valid'] = False
    if kind == 'missing': result['center'] = None
    if kind == 'unsupported': result['center']['y_max'] = 10
    if kind == 'nan': result['center']['coefficients'][0] = np.nan
    if kind == 'outside': result['center']['coefficients'][2] = 10000
    command = follow.calculate_command(result, bev)
    assert not command['valid']
    assert command['steering_cmd_rad'] == 0 and command['expected_adc'] is None


def test_rank_warning_aggregated_without_changing_results():
    rank_warning = np.exceptions.RankWarning if hasattr(np, 'exceptions') else np.RankWarning
    expected = {'state': 'PAIR_WEAK'}
    tracker = MagicMock()
    def process(mask):
        warnings.warn('Polyfit may be poorly conditioned', rank_warning)
        return expected
    tracker.process.side_effect = process
    with warnings.catch_warnings(record=True) as output:
        warnings.simplefilter('always')
        result, count = follow.process_lane(tracker, np.zeros((2,2), np.uint8))
    assert result is expected and count == 1 and not output


def test_real_tracker_warning_wrapper_equivalence(bev):
    mask = np.zeros((bev.height, bev.width), np.uint8)
    mask[:, 90:95] = 255
    mask[:, 490:495] = 255
    a, b = follow.PaperLaneTracker(), follow.PaperLaneTracker()
    with warnings.catch_warnings():
        warnings.simplefilter('ignore')
        before = a.process(mask)
    after, _ = follow.process_lane(b, mask)
    def equal(a, b):
        if isinstance(a, dict):
            assert a.keys() == b.keys()
            for key in a: equal(a[key], b[key])
        elif isinstance(a, np.ndarray): np.testing.assert_array_equal(a,b)
        else: assert a == b
    equal(before, after)


@pytest.fixture
def env(monkeypatch):
    overrides, now = {}, [100.]
    monkeypatch.setattr(follow.Node, '__init__', lambda self, name: None)
    monkeypatch.setattr(follow.Node, 'destroy_node', lambda self: None)
    monkeypatch.setattr(follow.Node, 'declare_parameter',
                        lambda self, name, value, descriptor=None: SimpleNamespace(value=overrides.get(name,value)))
    publisher = MagicMock()
    monkeypatch.setattr(follow.Node, 'create_publisher', publisher)
    monkeypatch.setattr(follow.Node, 'create_timer', MagicMock())
    monkeypatch.setattr(follow.Node, 'create_subscription', MagicMock())
    monkeypatch.setattr(follow.Node, 'get_logger', lambda self: MagicMock())
    clock = MagicMock()
    clock.now.return_value.to_msg.return_value = Time(sec=100)
    monkeypatch.setattr(follow.Node, 'get_clock', lambda self: clock)
    monkeypatch.setattr(follow.threading, 'Thread', MagicMock())
    monkeypatch.setattr(follow.time, 'monotonic', lambda: now[0])
    monkeypatch.setattr(follow.time, 'sleep', lambda seconds: None)
    camera = MagicMock(side_effect=AssertionError('Camera must not open'))
    monkeypatch.setattr(follow.cv2, 'VideoCapture', camera)
    def make(_node_class=follow.LaneFollowNode, **values):
        overrides.update(values)
        return _node_class()
    return SimpleNamespace(make=make, publisher=publisher, now=now, camera=camera)


def test_dry_run_has_no_control_publisher(env):
    node = env.make()
    node.latest = (100., follow.calculate_command(lane(node.bev), node.bev), 30.)
    node.tick()
    node.destroy_node()
    assert node.publisher is None
    env.publisher.assert_not_called()
    env.camera.assert_not_called()


def test_drive_invalid_stale_failure_shutdown(env):
    node = env.make(enable_drive=True)
    assert env.publisher.call_args.args[1] == '/cmd/lane'
    for result in ({'valid': False}, follow.calculate_command(lane(node.bev), node.bev)):
        if not result['valid']:
            node.tick()  # startup STOP
        else:
            node.latest = (100., result, 30.)
            node.tick()
        msg = node.publisher.publish.call_args.args[0]
        assert msg.drive_pwm == (40 if result['valid'] else 0)
        assert msg.emergency_stop == (not result['valid'])
    env.now[0] = 100.25
    node.tick()
    assert node.publisher.publish.call_args.args[0].drive_pwm == 0
    node.latest = (100.25, result, 30.)
    node.failure = 'camera disconnected'
    node.tick()
    assert node.publisher.publish.call_args.args[0].emergency_stop
    node.destroy_node()
    assert all(c.args[0].drive_pwm == 0 for c in node.publisher.publish.call_args_list[-3:])


@pytest.mark.parametrize('value', [0., -.1, float('nan'), float('inf')])
def test_invalid_config_before_camera(env, value):
    with pytest.raises(ValueError): env.make(lookahead_m=value)
    env.publisher.assert_not_called()
    env.camera.assert_not_called()


def test_drive_path_keeps_40_counts_and_steering(env, monkeypatch):
    from fma_control.command_arbiter_node import CommandArbiterNode
    from fma_vehicle.vehicle_controller_node import VehicleControllerNode
    from fma_vehicle import stm32_bridge_node as bridge
    source = env.make(enable_drive=True)
    arbiter = CommandArbiterNode()
    arbiter.config['allow_lane_pwm'] = True
    controller = VehicleControllerNode()
    port = MagicMock()
    port.write.side_effect = len
    monkeypatch.setattr(bridge.serial, 'Serial', MagicMock(return_value=port))
    stm32 = bridge.STM32BridgeNode()
    for node in (source, arbiter, controller, stm32): node.publisher = MagicMock()
    source.publisher.publish.side_effect = arbiter.on_lane
    arbiter.publisher.publish.side_effect = controller.on_command
    controller.publisher.publish.side_effect = stm32.on_command
    result = follow.calculate_command(lane(source.bev, .2), source.bev)
    source.latest = (100., result, 30.)
    source.tick()
    arbiter.publish_output()
    assert port.write.call_args.args[0] == b'F0040'
    assert controller.publisher.publish.call_args.args[0].steering_adc == result['expected_adc']
    assert controller.publisher.publish.call_args.args[0].drive_pwm == 40
    env.now[0] += .3
    source.tick()
    arbiter.publish_output()
    assert port.write.call_args.args[0] == b'X'


def test_worker_failure_releases_camera_and_invalidates(env, monkeypatch):
    node = env.make()
    port = MagicMock()
    port.isOpened.return_value = True
    port.read.return_value = (False, None)
    monkeypatch.setattr(follow.cv2, 'VideoCapture', MagicMock(return_value=port))
    node.capture()
    assert node.failure and node.latest is None
    port.release.assert_called_once()


@pytest.mark.parametrize('enabled,width,expected', [(False, 3.5, 350), (True, 3.5, 350),
                                                   (False, 3.0, 300)])
def test_lane_min_width_mode_and_metric_conversion(env, enabled, width, expected):
    node = env.make(enable_drive=enabled, lane_min_width_m=width)
    assert node.tracker.cfg.min_pair_width_px == pytest.approx(expected)
    assert follow.PaperLaneConfig().min_pair_width_px == 350.0
    if not enabled:
        env.publisher.assert_not_called()


@pytest.mark.parametrize('enabled,width', [(True, 3.0), (True, 4.0), (False, 0.0),
                                          (False, -1.0), (False, float('nan')),
                                          (False, float('inf'))])
def test_lane_min_width_invalid_before_publisher_or_camera(env, enabled, width):
    with pytest.raises(ValueError):
        env.make(enable_drive=enabled, lane_min_width_m=width)
    env.publisher.assert_not_called()
    env.camera.assert_not_called()


def test_narrow_lane_override_only_affects_local_dry_run_tracker(env):
    node = env.make(lane_min_width_m=3.0)
    mask = np.zeros((180, 600), np.uint8)
    mask[:, 137:144] = 255
    mask[:, 457:464] = 255
    assert node.tracker.process(mask)['pair_state'] == 'PAIR_VALID'
    production = follow.PaperLaneTracker().process(mask)
    assert production['pair_quality']['reason'] == 'lane_width_too_narrow'
    env.publisher.assert_not_called()


@pytest.mark.parametrize('enabled,width,accepted', [('false', '3.0', True),
    ('true', '3.50', True), ('true', '3.0', False), ('false', 'nan', False)])
def test_launch_validates_width_before_starting_nodes(enabled, width, accepted, monkeypatch, tmp_path):
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path))
    import importlib.util
    from launch import LaunchContext
    from launch.actions import OpaqueFunction
    from launch_ros.actions import Node
    path = REPO / 'ros2_ws/src/fma_control/launch/lane_follow.launch.py'
    spec = importlib.util.spec_from_file_location('lane_launch_test', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    context = LaunchContext()
    context.launch_configurations.update(enable_drive=enabled, lane_min_width_m=width)
    if accepted:
        assert module.validate_width(context) == []
    else:
        with pytest.raises(ValueError): module.validate_width(context)
    entities = module.generate_launch_description().entities
    guard_index = next(i for i, action in enumerate(entities) if isinstance(action, OpaqueFunction))
    node_index = next(i for i, action in enumerate(entities) if isinstance(action, Node))
    assert guard_index < node_index


@pytest.mark.parametrize('near,far,desired,expected', [
    (1.7, 3.8, 2.0, 2.0), (2.3, 4.0, 2.0, 2.4),
    (1.0, 2.0, 2.0, 1.9), (1.7, 3.8, 1.0, 1.8)])
def test_adaptive_lookahead_inside_observed_range(bev, near, far, desired, expected):
    observation = lane(bev)
    observation['center'].update(y_min=bev.ground_to_bev_pixel(0, far)[1],
                                 y_max=bev.ground_to_bev_pixel(0, near)[1])
    before = copy.deepcopy(observation)
    result = follow.calculate_command(observation, bev, follow.FollowConfig(lookahead_m=desired))
    assert result['valid']
    assert result['desired_lookahead'] == desired
    assert result['used_lookahead'] == pytest.approx(expected)
    assert result['observed_min'] == pytest.approx(near)
    assert result['observed_max'] == pytest.approx(far)
    assert near < bev.bev_pixel_to_ground(*result['target_px'])[1] < far
    np.testing.assert_array_equal(before['center']['coefficients'], observation['center']['coefficients'])


@pytest.mark.parametrize('low,high', [(300, 320), (320, 300), (float('nan'), 400),
                                     (0, float('inf')), (600, 650)])
def test_adaptive_rejects_short_or_invalid_observed_range(bev, low, high):
    observation = lane(bev)
    observation['center'].update(y_min=low, y_max=high)
    result = follow.calculate_command(observation, bev)
    assert not result['valid']
    assert result['used_lookahead'] is None
    assert result['steering_cmd_rad'] == 0


def test_adaptive_dry_run_logs_without_publishing(env, monkeypatch):
    node = env.make(enable_drive=False)
    observation = lane(node.bev)
    observation['center'].update(y_min=200, y_max=370)
    result = follow.calculate_command(observation, node.bev, node.follow_config)
    node.latest = (100., result, 30.)
    logger = MagicMock()
    monkeypatch.setattr(follow.Node, 'get_logger', lambda self: logger)
    node.tick()
    log = logger.info.call_args.args[0]
    assert 'desired_lookahead=2.00m used_lookahead=2.40m observed_min=2.30m observed_max=4.00m' in log
    env.publisher.assert_not_called()
    env.camera.assert_not_called()


@pytest.mark.parametrize('near,far', [(3.1, 5.6), (4., 6.), (.8, 1.7)])
def test_no_steering_outside_control_distance_window(bev, near, far):
    observation = lane(bev)
    observation['center'].update(y_min=bev.ground_to_bev_pixel(0, far)[1],
                                 y_max=bev.ground_to_bev_pixel(0, near)[1])
    result = follow.calculate_command(observation, bev)
    assert not result['valid']
    assert result['used_lookahead'] is None
    assert result['steering_cmd_rad'] == 0


@pytest.mark.parametrize('field,allow', [(False, False), (False, True), (True, False), (True, True)])
@pytest.mark.parametrize('side,x', [('right', 500), ('left', 100)])
def test_calculate_single_side_explicit_two_flags(bev, field, allow, side, x):
    fit = {'coefficients': np.array([0., 0., float(x)]), 'y_min': 0,
           'y_max': bev.height-1, 'points': 100, 'residual_px': 1.}
    observation = {'state': side.upper()+'_ONLY', side: fit}
    result = follow.calculate_command(observation, bev, field_stop_test=field,
                                      allow_single_side_test=allow)
    assert not result['valid']
    assert result['steering_cmd_rad'] == 0
    assert result['control_source'] == 'NONE'


def test_calculate_pair_priority_over_single(bev):
    observation = lane(bev)
    observation.update(state='RIGHT_ONLY', temporary_center={'coefficients': [0,0,500]})
    result = follow.calculate_command(observation, bev, field_stop_test=True, allow_single_side_test=True)
    assert result['valid'] and result['control_source'] == 'PAIR'
    assert result['steering_cmd_rad'] == pytest.approx(0.)


@pytest.mark.parametrize('near,valid', [(2.98, True), (3.33, False)])
def test_temporary_center_uses_field_window_with_margin(bev, near, valid):
    center = {'coefficients': np.array([0., 0., 300.]), 'y_min': 8,
              'y_max': bev.ground_to_bev_pixel(0, near)[1]}
    observation = {'state': 'RIGHT_ONLY', 'center_source': 'RIGHT_ONLY_OFFSET',
                   'temporary_center': center, 'right': {**center, 'points': 100, 'residual_px': 1.}}
    result = follow.calculate_command(observation, bev, field_stop_test=True,
                                      allow_single_side_test=True)
    assert result['control_source'] == 'TEMPORARY_CENTER'
    assert result['valid'] == valid
    if valid:
        assert result['used_lookahead'] == pytest.approx(3.08)
    else:
        assert result['reason'] == 'outside_control_window'
        assert result['used_lookahead'] is None
    assert follow.calculate_command(observation, bev)['reason'] == 'invalid_pair'


def pair_with_side_ranges(bev, right_near=2.9, left_near=3.44):
    def fit(x, near):
        return {'coefficients': np.array([0., 0., x]), 'y_min': bev.ground_to_bev_pixel(0, 3.95)[1],
                'y_max': bev.ground_to_bev_pixel(0, near)[1], 'points': 100, 'residual_px': 1.}
    return {'pair_state': 'PAIR_VALID', 'pair_quality': {'valid': True},
            'center': fit(300., 3.44), 'left': fit(100., left_near), 'right': fit(500., right_near)}


@pytest.mark.parametrize('field,allow', [(True, True), (True, False), (False, True), (False, False)])
def test_outside_pair_right_fallback_is_field_only(bev, field, allow):
    observation = pair_with_side_ranges(bev)
    result = follow.calculate_command(observation, bev, field_stop_test=field,
                                      allow_single_side_test=allow)
    assert not result['valid']
    assert result['reason'] == 'outside_control_window'
    assert result['control_source'] == 'PAIR'


def test_usable_pair_always_wins_over_sides(bev):
    observation = pair_with_side_ranges(bev, 1.8, 1.8)
    observation['center']['y_max'] = bev.ground_to_bev_pixel(0, 1.8)[1]
    result = follow.calculate_command(observation, bev, field_stop_test=True, allow_single_side_test=True)
    assert result['valid'] and result['control_source'] == 'PAIR'
    assert result['used_lookahead'] <= 3.0


def test_outside_pair_and_sides_stay_invalid(bev):
    result = follow.calculate_command(pair_with_side_ranges(bev, 3.44, 3.44), bev,
                                      field_stop_test=True, allow_single_side_test=True)
    assert not result['valid'] and result['steering_cmd_rad'] == 0
    assert result['reason'] == 'outside_control_window'


@pytest.mark.parametrize('right_x', [460, 540])
def test_real_pair_center_tracks_measured_width(bev, right_x):
    mask = np.zeros((bev.height, bev.width), np.uint8)
    mask[:, 97:104] = 255
    mask[:, right_x-3:right_x+4] = 255
    observation = follow.PaperLaneTracker().process(mask)
    assert observation['pair_state'] == 'PAIR_VALID'
    result = follow.calculate_command(observation, bev, field_stop_test=True, allow_single_side_test=True)
    assert result['control_source'] == 'PAIR' and result['valid']
    expected_x = bev.bev_pixel_to_ground((100 + right_x)/2, 0)[0]
    assert result['lateral_error_m'] == pytest.approx(-expected_x, abs=.01)


@pytest.mark.parametrize('failure', ['missing', 'nonfinite', 'far', 'weak_pair', 'boundary_lost'])
def test_unusable_temporary_never_uses_visible_fit_as_fixed_center(bev, failure):
    pair = pair_with_side_ranges(bev, 1.8, 1.8)
    center = {**pair['center'], 'y_max': pair['right']['y_max']}
    observation = {'state': 'RIGHT_ONLY', 'right': pair['right'],
                   'temporary_center': center, 'center_source': 'RIGHT_ONLY_OFFSET'}
    if failure == 'missing': observation['temporary_center'] = None
    if failure == 'nonfinite': center['coefficients'] = [0., 0., float('nan')]
    if failure == 'far': center['y_max'] = bev.ground_to_bev_pixel(0, 3.44)[1]
    if failure == 'weak_pair': observation['pair_state'] = 'PAIR_WEAK'
    if failure == 'boundary_lost': observation['right'] = None
    result = follow.calculate_command(observation, bev, field_stop_test=True, allow_single_side_test=True)
    assert not result['valid'] and result['steering_cmd_rad'] == 0


@pytest.mark.parametrize('near,maximum,valid', [
    (3.39, 3.5, True), (3.42, 3.5, False), (3.43, 3.5, False),
    (3.5, 3.5, False), (3.51, 3.5, False), (3.39, 3.0, False),
    (3.43, 3.6, True),
])
def test_field_pair_max_preserves_observation_margin(bev, near, maximum, valid):
    observation = lane(bev)
    observation['center']['y_max'] = bev.ground_to_bev_pixel(0, near)[1]
    result = follow.calculate_command(observation, bev, field_stop_test=True,
                                      pair_control_max_m=maximum)
    assert result['valid'] == valid
    assert result['control_source'] == 'PAIR'
    if valid:
        assert near + .1 <= result['used_lookahead'] <= maximum
    else:
        assert result['reason'] == 'outside_control_window'
        assert result['steering_cmd_rad'] == 0
    production = follow.calculate_command(observation, bev, pair_control_max_m=maximum)
    assert not production['valid']  # Production always retains 3.0 m.


@pytest.mark.parametrize('maximum', [1.79, 4., 5., float('inf'), float('nan')])
def test_calculator_rejects_unsafe_field_pair_max(bev, maximum):
    result = follow.calculate_command(lane(bev), bev, field_stop_test=True,
                                      pair_control_max_m=maximum)
    assert not result['valid'] and result['reason'] == 'invalid_pair_control_max'


def test_pair_max_does_not_extend_temporary_center(bev):
    center = lane(bev)['center']
    center['y_max'] = bev.ground_to_bev_pixel(0, 3.39)[1]
    observation = {'state': 'RIGHT_ONLY', 'right': center, 'temporary_center': center,
                   'center_source': 'RIGHT_ONLY_OFFSET'}
    result = follow.calculate_command(observation, bev, field_stop_test=True,
        allow_single_side_test=True, pair_control_max_m=3.5)
    assert not result['valid'] and result['control_source'] == 'TEMPORARY_CENTER'
