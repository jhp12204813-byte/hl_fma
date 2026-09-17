"""Offline perception tests; all camera and ROS node access is mocked."""
from types import SimpleNamespace
from unittest.mock import MagicMock

import cv2
import numpy as np
import pytest
from builtin_interfaces.msg import Time
from sensor_msgs.msg import Image
from fma_interfaces.msg import Lane

from fma_perception.lane_detection import LaneConfig, detect_lane
from fma_perception import c920_camera_node as camera
from fma_perception import lane_detector_node as detector


def lane_image(color_left=(255, 255, 255), color_right=(0, 255, 255)):
    image = np.zeros((720, 1280, 3), dtype=np.uint8)
    cv2.line(image, (260, 719), (440, 360), color_left, 12)
    cv2.line(image, (1020, 719), (840, 360), color_right, 12)
    return image


@pytest.mark.parametrize('colors', [
    ((255, 255, 255), (0, 255, 255)),
    ((255, 255, 255), (255, 255, 255)),
    ((0, 255, 255), (0, 255, 255)),
])
def test_synthetic_lanes(colors):
    image = lane_image(*colors)
    result = detect_lane(image, LaneConfig(lane_width_m=1.0,
                                           meters_per_pixel_y=.005))
    assert result.detected
    assert result.visual_detected
    assert abs(result.lateral_error_m) < .03
    assert abs(result.heading_error_rad) < .03
    assert 0 < result.confidence <= 1
    assert result.curvature == 0
    assert result.debug.shape == image.shape
    assert not np.array_equal(result.debug, image)


def test_uncalibrated_visual_only():
    result = detect_lane(lane_image())
    assert result.visual_detected
    assert not result.detected
    assert (result.lateral_error_m, result.heading_error_rad, result.confidence) == (0., 0., 0.)


@pytest.mark.parametrize('shift', [-30, 0, 30])
def test_laptop_pixel_overlay_values_and_unavailable(monkeypatch, shift):
    image = np.zeros((480, 640, 3), np.uint8)
    for x in (160 + shift, 480 + shift):
        cv2.line(image, (x, 479), (x, 240), (255, 255, 255), 8)
    texts = []
    put_text = cv2.putText

    def record(frame, text, *args, **kwargs):
        texts.append(text)
        # All labels must fit horizontally on the target 640 px image.
        assert args[0][0] + cv2.getTextSize(text, args[1], args[2], args[4])[0][0] < 640
        return put_text(frame, text, *args, **kwargs)

    monkeypatch.setattr(cv2, 'putText', record)
    result = detect_lane(image)
    coordinates = next(text for text in texts if text.startswith('y='))
    values = dict(item.split('=') for item in coordinates.split())
    assert values['y'] == '312'
    assert float(values['image_center_px']) == 320.
    assert float(values['lane_center_px']) == pytest.approx(320 + shift, abs=1.)
    error = next(text for text in texts if text.startswith('pixel_error='))
    assert float(error.split('=')[1].split('px')[0]) == pytest.approx(-shift, abs=1.)
    assert result.visual_detected and not result.detected
    assert (result.lateral_error_m, result.heading_error_rad, result.confidence) == (0., 0., 0.)
    for frame in (np.zeros_like(image), image.copy()):
        frame[:, 320:] = 0
        texts.clear()
        result = detect_lane(frame)
        assert not result.visual_detected and not result.detected
        assert any('lane_center_px=N/A' in text for text in texts)
        assert any('pixel_error=N/Apx' in text for text in texts)


@pytest.mark.parametrize('shift', [-30, 0, 30])
def test_laptop_rgb8_debug_publish_and_loss(ros, shift):
    """Real cv_bridge RGB conversion, both outputs, and no stale metric result."""
    node = detector.LaneDetectorNode()
    assert node.config.lane_width_m == node.config.meters_per_pixel_y == 0.
    assert node.config.single_lane_width_px == 0.
    image = np.zeros((480, 640, 3), np.uint8)
    cv2.line(image, (130 + shift, 479), (220 + shift, 240), (255, 255, 255), 8)
    cv2.line(image, (510 + shift, 479), (420 + shift, 240), (0, 255, 255), 8)
    one_side = image.copy()
    one_side[:, 320:] = 0
    for index, (frame, visual) in enumerate([
            (image, True), (one_side, False), (np.zeros_like(image), False)]):
        estimate = detect_lane(frame)
        assert estimate.visual_detected == visual
        msg = node.bridge.cv2_to_imgmsg(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB), 'rgb8')
        msg.header.frame_id = 'camera_color_optical_frame'
        msg.header.stamp.sec = 42 + index
        node.on_image(msg)
        lane = node.publisher.publish.call_args.args[0]
        assert lane.header == msg.header
        assert not lane.detected
        assert (lane.lateral_error_m, lane.heading_error_rad,
                lane.curvature, lane.confidence) == (0., 0., 0., 0.)
        for publisher, expected in [(node.debug_publisher, estimate.debug),
                                    (node.mask_publisher, estimate.debug_mask)]:
            assert publisher.publish.call_count == index + 1
            output = publisher.publish.call_args.args[0]
            assert (output.width, output.height, output.encoding) == (640, 480, 'bgr8')
            assert output.header == msg.header
            np.testing.assert_array_equal(node.bridge.imgmsg_to_cv2(output), expected)
        assert not np.any(estimate.debug_mask[:288])
    assert np.any(np.all(detect_lane(image).debug_mask == (0, 255, 255), axis=2))


def test_blank_and_upper_light():
    image = np.zeros((720, 1280, 3), np.uint8)
    image[:300] = 255
    result = detect_lane(image)
    assert not result.detected and not result.visual_detected


def test_one_side_is_not_reliable():
    image = lane_image(color_right=(0, 0, 0))
    assert not detect_lane(image, LaneConfig(lane_width_m=1., meters_per_pixel_y=.005)).detected


def test_noise_and_nonmutation():
    image = np.random.default_rng(5).integers(0, 256, (720, 1280, 3), dtype=np.uint8)
    original = image.copy()
    result = detect_lane(image)
    assert 0 <= result.confidence <= 1
    assert np.array_equal(image, original)


def test_lateral_sign():
    image = cv2.warpAffine(lane_image(), np.float32([[1, 0, -50], [0, 1, 0]]),
                          (1280, 720))
    result = detect_lane(image, LaneConfig(lane_width_m=1., meters_per_pixel_y=.005))
    assert result.detected
    assert result.lateral_error_m > .05


@pytest.mark.parametrize('kwargs', [
    {'roi_top_ratio': 1.}, {'roi_bottom_ratio': 0.},
    {'lane_width_m': -1.}, {'meters_per_pixel_y': float('nan')},
])
def test_invalid_config(kwargs):
    with pytest.raises(ValueError):
        LaneConfig(**kwargs)


@pytest.fixture
def ros(monkeypatch):
    publishers = []
    subscriptions = []
    clock = MagicMock()
    clock.now.return_value.to_msg.return_value = Time(sec=123)
    monkeypatch.setattr(camera.Node, '__init__', lambda self, name: None)
    parameters = {}
    def declare(self, name, value, desc):
        parameters[name] = value
        return SimpleNamespace(value=value)
    monkeypatch.setattr(camera.Node, 'declare_parameter', declare)
    monkeypatch.setattr(camera.Node, 'get_parameter',
                        lambda self, name: SimpleNamespace(value=parameters[name]))
    monkeypatch.setattr(camera.Node, 'add_on_set_parameters_callback', MagicMock())
    def publisher(self, kind, topic, qos):
        pub = MagicMock()
        publishers.append((kind, topic, pub))
        return pub
    monkeypatch.setattr(camera.Node, 'create_publisher', publisher)
    monkeypatch.setattr(camera.Node, 'create_subscription',
                        lambda self, *args: subscriptions.append(args))
    monkeypatch.setattr(camera.Node, 'create_timer', MagicMock())
    monkeypatch.setattr(camera.Node, 'get_clock', lambda self: clock)
    monkeypatch.setattr(camera.Node, 'get_logger', lambda self: MagicMock())
    monkeypatch.setattr(camera.Node, 'destroy_node', MagicMock())
    return publishers, subscriptions, parameters


def test_camera_defaults_capture_release(ros, monkeypatch):
    capture = MagicMock()
    capture.isOpened.return_value = True
    capture.read.return_value = (True, lane_image())
    factory = MagicMock(return_value=capture)
    monkeypatch.setattr(camera.cv2, 'VideoCapture', factory)
    node = camera.C920CameraNode()
    factory.assert_called_once_with('/dev/fma_c920', cv2.CAP_V4L2)
    assert (node.config['width'], node.config['height'], node.config['fps']) == (1280, 720, 30.)
    capture.set.assert_any_call(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    capture.set.assert_any_call(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    capture.set.assert_any_call(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    capture.set.assert_any_call(cv2.CAP_PROP_FPS, 30.)
    assert ros[0][0][:2] == (Image, '/camera/front/image_raw')
    node.publish_frame()
    msg = node.publisher.publish.call_args.args[0]
    assert msg.encoding == 'bgr8'
    assert msg.header.frame_id == 'front_camera' and msg.header.stamp.sec == 123
    capture.read.return_value = (False, None)
    node.publish_frame()
    assert node.publisher.publish.call_count == 1
    capture.read.return_value = (True, lane_image())
    node.publish_frame()
    assert node.publisher.publish.call_count == 2
    node.destroy_node()
    node.destroy_node()
    capture.release.assert_called_once()


def test_camera_open_failure_releases(ros, monkeypatch):
    capture = MagicMock()
    capture.isOpened.return_value = False
    monkeypatch.setattr(camera.cv2, 'VideoCapture', lambda *args: capture)
    with pytest.raises(RuntimeError, match='/dev/fma_c920'):
        camera.C920CameraNode()
    capture.release.assert_called_once()


def test_detector_contract_header_and_no_commands(ros):
    node = detector.LaneDetectorNode()
    assert [(kind, topic) for kind, topic, _ in ros[0]] == [
        (Lane, '/perception/lane'), (Image, '/perception/lane/debug_image'),
        (Image, '/perception/lane/debug_mask')]
    assert ros[1][0][:2] == (Image, '/front/color/image_raw')
    msg = node.bridge.cv2_to_imgmsg(lane_image(), encoding='bgr8')
    msg.header.frame_id = 'front_camera'
    msg.header.stamp.sec = 42
    node.on_image(msg)
    result = node.publisher.publish.call_args.args[0]
    assert not result.detected
    assert result.header == msg.header
    assert node.debug_publisher.publish.call_args.args[0].header == msg.header


def test_detector_calibrated_then_invalid_image_does_not_reuse_lane(ros):
    node = detector.LaneDetectorNode()
    ros[2].update(lane_width_m=1., meters_per_pixel_y=.005)
    msg = node.bridge.cv2_to_imgmsg(lane_image(), encoding='bgr8')
    node.on_image(msg)
    assert node.publisher.publish.call_args.args[0].detected
    node.bridge.imgmsg_to_cv2 = MagicMock(side_effect=ValueError('bad frame'))
    node.on_image(msg)
    result = node.publisher.publish.call_args.args[0]
    assert not result.detected
    assert result.confidence == 0.


def test_camera_main_interrupt_releases(ros, monkeypatch):
    capture = MagicMock()
    capture.isOpened.return_value = True
    monkeypatch.setattr(camera.cv2, 'VideoCapture', lambda *args: capture)
    monkeypatch.setattr(camera.rclpy, 'init', MagicMock())
    monkeypatch.setattr(camera.rclpy, 'spin', MagicMock(side_effect=KeyboardInterrupt))
    monkeypatch.setattr(camera.rclpy, 'ok', lambda: True)
    shutdown = MagicMock()
    monkeypatch.setattr(camera.rclpy, 'shutdown', shutdown)
    assert camera.main() == 0
    capture.release.assert_called_once()
    shutdown.assert_called_once()


@pytest.mark.parametrize('shift,sign', [(-70, 1), (70, -1)])
def test_shifted_lane_sign_and_metric_width(shift, sign):
    image = cv2.warpAffine(lane_image(), np.float32([[1, 0, shift], [0, 1, 0]]),
                          (1280, 720))
    config = LaneConfig(lane_width_m=1., meters_per_pixel_y=.005)
    result = detect_lane(image, config)
    assert result.detected and result.lateral_error_m * sign > 0
    double_width = detect_lane(image, LaneConfig(lane_width_m=2., meters_per_pixel_y=.005))
    assert double_width.lateral_error_m == pytest.approx(result.lateral_error_m * 2)


@pytest.mark.parametrize('lean,sign', [(-80, 1), (80, -1)])
def test_heading_sign(lean, sign):
    image = np.zeros((720, 1280, 3), np.uint8)
    cv2.line(image, (260, 719), (440 + lean, 360), (255, 255, 255), 12)
    cv2.line(image, (1020, 719), (840 + lean, 360), (0, 255, 255), 12)
    result = detect_lane(image, LaneConfig(lane_width_m=1., meters_per_pixel_y=.005))
    assert result.detected
    assert result.heading_error_rad * sign > 0
    assert result.lateral_error_m * sign > 0


@pytest.mark.parametrize('kwargs', [
    {'lane_width_m': .001}, {'lane_width_m': float('inf')},
    {'roi_left_ratio': .8, 'roi_right_ratio': .2},
    {'lookahead_ratio': .1},
])
def test_d435i_invalid_calibration_and_roi(kwargs):
    with pytest.raises(ValueError):
        LaneConfig(**kwargs)


def test_narrow_pixel_width_rejected():
    image = np.zeros((720, 1280, 3), np.uint8)
    for x in (620, 660):
        cv2.line(image, (x, 719), (x, 360), (255, 255, 255), 12)
    result = detect_lane(image, LaneConfig(lane_width_m=1., meters_per_pixel_y=.005))
    assert not result.detected and result.confidence == 0.


def test_roi_horizontal_bounds():
    config = LaneConfig(roi_left_ratio=.4, roi_right_ratio=.6,
                        lane_width_m=1., meters_per_pixel_y=.005)
    assert not detect_lane(lane_image(), config).detected


def test_single_lane_requires_explicit_width_and_has_low_confidence():
    image = lane_image(color_right=(0, 0, 0))
    no_width = LaneConfig(lane_width_m=1., meters_per_pixel_y=.005)
    assert not detect_lane(image, no_width).detected
    calibrated = LaneConfig(lane_width_m=1., meters_per_pixel_y=.005,
                            single_lane_width_px=550.)
    result = detect_lane(image, calibrated)
    assert result.detected and result.confidence == .35
    assert not detect_lane(image, LaneConfig(single_lane_width_px=550.)).detected


def test_night_tuning_defaults_and_mask_without_calibration():
    config = LaneConfig()
    assert config.roi_top_ratio == .60
    assert (config.white_h_min, config.white_h_max, config.white_s_min,
            config.white_s_max, config.white_v_min, config.white_v_max) == (0, 179, 0, 65, 170, 255)
    result = detect_lane(lane_image())
    assert result.debug_mask.shape == (720, 1280, 3)
    assert result.debug_mask.dtype == np.uint8
    assert np.any(np.all(result.debug_mask == (255, 255, 255), axis=2))
    assert np.any(np.all(result.debug_mask == (0, 255, 255), axis=2))
    assert np.count_nonzero(result.debug_mask[:432]) == 0
    assert not result.detected
    assert (result.lateral_error_m, result.heading_error_rad, result.confidence) == (0., 0., 0.)


@pytest.mark.parametrize('color', ['white', 'yellow'])
@pytest.mark.parametrize('channel,limit', [('h', 179), ('s', 255), ('v', 255)])
def test_hsv_bounds_validation(color, channel, limit):
    for suffix, value in [('min', -1), ('max', limit + 1), ('min', .5), ('max', True)]:
        with pytest.raises(ValueError):
            LaneConfig(**{f'{color}_{channel}_{suffix}': value})
    with pytest.raises(ValueError):
        LaneConfig(**{f'{color}_{channel}_min': 100, f'{color}_{channel}_max': 90})


@pytest.mark.parametrize('kwargs', [
    {'morph_kernel_size': 0}, {'morph_kernel_size': 2}, {'morph_kernel_size': 33},
    {'morph_kernel_size': 3.5}, {'min_candidate_area_px': -1},
    {'min_candidate_area_px': .5}, {'min_line_length_px': -1},
    {'max_line_gap_px': -1}, {'min_abs_slope': -1.}, {'min_abs_slope': float('nan')},
])
def test_filter_validation(kwargs):
    with pytest.raises(ValueError):
        LaneConfig(**kwargs)


def test_low_saturation_light_is_not_yellow():
    image = np.zeros((720, 1280, 3), np.uint8)
    image[500:650, 500:800] = (220, 240, 240)
    result = detect_lane(image)
    assert not np.any(np.all(result.debug_mask == (0, 255, 255), axis=2))


def test_threshold_and_component_filter_effect():
    image = lane_image(color_left=(150, 150, 150), color_right=(0, 0, 0))
    assert np.count_nonzero(detect_lane(image).debug_mask) == 0
    tuned = detect_lane(image, LaneConfig(white_v_min=140))
    assert np.count_nonzero(tuned.debug_mask) > 0
    removed = detect_lane(image, LaneConfig(white_v_min=140, min_candidate_area_px=1000000))
    assert np.count_nonzero(removed.debug_mask) == 0


def test_runtime_validation_commit_and_next_frame(ros):
    node = detector.LaneDetectorNode()
    image = lane_image(color_left=(150, 150, 150), color_right=(0, 0, 0))
    msg = node.bridge.cv2_to_imgmsg(image, encoding='bgr8')
    msg.header.stamp.sec = 42
    node.on_image(msg)
    def mask():
        output = node.mask_publisher.publish.call_args.args[0]
        assert output.header == msg.header and output.encoding == 'bgr8'
        return node.bridge.imgmsg_to_cv2(output, desired_encoding='bgr8')
    assert not np.any(mask())
    parameter = SimpleNamespace(name='white_v_min', value=140)
    assert node.validate_parameters([parameter]).successful
    # Validation alone does not affect frames before the parameter server commits.
    node.on_image(msg)
    assert not np.any(mask())
    ros[2]['white_v_min'] = 140
    node.on_image(msg)
    assert np.any(mask())
    bad = [SimpleNamespace(name='white_v_min', value=140),
           SimpleNamespace(name='yellow_h_max', value=180)]
    assert not node.validate_parameters(bad).successful
    assert ros[2]['yellow_h_max'] == 40
    assert node.read_config().white_v_min == 140
    assert not node.validate_parameters(
        [SimpleNamespace(name='lane_width_m', value=1.)]).successful
    roi = [SimpleNamespace(name='roi_top_ratio', value=.7),
           SimpleNamespace(name='lookahead_ratio', value=.8)]
    assert node.validate_parameters(roi).successful
    assert not node.validate_parameters(roi[:1]).successful
    ros[2].update(roi_top_ratio=.7, lookahead_ratio=.8)
    node.on_image(msg)
    assert not np.any(mask()[:int(720 * .7)])


def test_bev_node_topics_header_and_unconfigured_fail_closed(ros, monkeypatch):
    original = detector.Node.declare_parameter
    def declare(self,name,value,descriptor):
        return original(self,name,'bev' if name == 'pipeline' else value,descriptor)
    monkeypatch.setattr(detector.Node,'declare_parameter',declare)
    node = detector.LaneDetectorNode()
    from sensor_msgs.msg import CameraInfo
    info = CameraInfo()
    info.width,info.height = 640,480
    info.k = [600.,0.,320.,0.,600.,240.,0.,0.,1.]
    info.d = [0.]*5
    info.distortion_model = 'plumb_bob'
    info.header.frame_id = 'camera_color_optical_frame'
    node.on_camera_info(info)
    msg = node.bridge.cv2_to_imgmsg(np.zeros((480,640,3),np.uint8),'bgr8')
    msg.header.frame_id = info.header.frame_id
    msg.header.stamp.sec = 12
    node.on_image(msg)
    import json
    diagnostics = json.loads(node.diagnostics_publisher.publish.call_args.args[0].data)
    assert diagnostics['status'] == 'unconfigured_bev'
    assert diagnostics['timestamp_ns'] == 12_000_000_000
    assert node.bev_publisher.publish.call_args.args[0].header == msg.header
    assert not node.publisher.publish.call_args.args[0].detected
    assert all(topic.startswith('/perception/lane') for _,topic,_ in ros[0])
    assert {args[1] for args in ros[1]} == {'/front/color/image_raw','/front/color/camera_info'}
