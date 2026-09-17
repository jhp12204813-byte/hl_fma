"""Offline snapshot tests: mocked ROS transport, real cv_bridge and file codecs."""
from types import SimpleNamespace
from unittest.mock import MagicMock

import cv2
import numpy as np
import pytest
from sensor_msgs.msg import Image

from fma_perception import lane_debug_snapshot as snapshot


@pytest.fixture
def harness(monkeypatch, tmp_path):
    clock = [100.0]
    logger = MagicMock()
    subscriptions = {}
    monkeypatch.setattr(snapshot.time, 'monotonic', lambda: clock[0])
    monkeypatch.setattr(snapshot.Node, '__init__', lambda self, name: None)
    monkeypatch.setattr(snapshot.Node, 'declare_parameter',
                        lambda self, name, default: SimpleNamespace(
                            value=str(tmp_path) if name == 'output_dir' else default))
    monkeypatch.setattr(snapshot.Node, 'get_logger', lambda self: logger)
    monkeypatch.setattr(snapshot.Node, 'create_subscription',
                        lambda self, kind, topic, callback, qos:
                        subscriptions.setdefault(topic, (callback, qos)))
    destroyed = MagicMock()
    shutdown = MagicMock()
    monkeypatch.setattr(snapshot.Node, 'destroy_node', destroyed)
    monkeypatch.setattr(snapshot.rclpy, 'init', MagicMock())
    monkeypatch.setattr(snapshot.rclpy, 'ok', lambda: True)
    monkeypatch.setattr(snapshot.rclpy, 'shutdown', shutdown)
    return SimpleNamespace(clock=clock, logger=logger, subscriptions=subscriptions,
                           output=tmp_path, destroyed=destroyed, shutdown=shutdown)


@pytest.mark.parametrize('mask_encoding', ['mono8', 'bgr8'])
def test_three_frames_save_latest_and_exit(harness, monkeypatch, mask_encoding):
    topics = list(snapshot.TOPICS)
    mask = np.full((16, 16) if mask_encoding == 'mono8' else (16, 16, 3), 127, np.uint8)
    rgb = np.full((16, 16, 3), (240, 50, 20), np.uint8)
    bridge = snapshot.CvBridge()
    messages = [
        (topics[0], bridge.cv2_to_imgmsg(np.zeros_like(rgb), encoding='rgb8')),
        (topics[0], bridge.cv2_to_imgmsg(rgb, encoding='rgb8')),
        (topics[1], bridge.cv2_to_imgmsg(rgb, encoding='bgr8')),
        (topics[2], bridge.cv2_to_imgmsg(mask, encoding=mask_encoding)),
    ]

    def spin(node, timeout_sec):
        assert not list(harness.output.iterdir())
        topic, msg = messages.pop(0)
        callback, qos = harness.subscriptions[topic]
        assert qos.depth == 1
        assert qos.reliability == snapshot.ReliabilityPolicy.BEST_EFFORT
        callback(msg)

    monkeypatch.setattr(snapshot.rclpy, 'spin_once', spin)
    assert snapshot.main() == 0
    assert not messages
    assert set(p.name for p in harness.output.iterdir()) == set(snapshot.TOPICS.values())
    camera = cv2.imread(str(harness.output / 'camera_rgb.jpg'))
    debug = cv2.imread(str(harness.output / 'lane_debug.jpg'))
    assert np.allclose(camera, rgb[:, :, ::-1], atol=3)
    assert np.allclose(debug, rgb, atol=3)
    saved_mask = cv2.imread(str(harness.output / 'lane_mask.png'), cv2.IMREAD_UNCHANGED)
    np.testing.assert_array_equal(saved_mask, mask)
    harness.destroyed.assert_called_once()
    harness.shutdown.assert_called_once()


@pytest.mark.parametrize('received', [0, 1, 2])
def test_timeout_reports_missing_topics(harness, monkeypatch, received):
    def spin(node, timeout_sec):
        assert timeout_sec == pytest.approx(5.0)
        for topic in list(snapshot.TOPICS)[:received]:
            node.on_image(topic, node.bridge.cv2_to_imgmsg(
                np.zeros((8, 8, 3), np.uint8), encoding='bgr8'))
        harness.clock[0] += timeout_sec

    monkeypatch.setattr(snapshot.rclpy, 'spin_once', spin)
    assert snapshot.main() != 0
    error = harness.logger.error.call_args.args[0]
    for index, topic in enumerate(snapshot.TOPICS):
        assert (topic in error) == (index >= received)
    assert not list(harness.output.iterdir())
    harness.destroyed.assert_called_once()
    harness.shutdown.assert_called_once()


def test_invalid_image_conversion_exits(harness, monkeypatch):
    topic = next(iter(snapshot.TOPICS))
    monkeypatch.setattr(snapshot.rclpy, 'spin_once',
                        lambda node, timeout_sec: node.on_image(
                            topic, Image(encoding='invalid_encoding')))
    assert snapshot.main() != 0
    error = harness.logger.error.call_args.args[0]
    assert 'conversion failed' in error and topic in error
    assert not list(harness.output.iterdir())
    harness.destroyed.assert_called_once()
    harness.shutdown.assert_called_once()


def test_failed_write_exits(harness, monkeypatch):
    monkeypatch.setattr(snapshot.cv2, 'imwrite', lambda *args: False)

    def spin(node, timeout_sec):
        for topic in snapshot.TOPICS:
            node.on_image(topic, node.bridge.cv2_to_imgmsg(
                np.zeros((8, 8, 3), np.uint8), encoding='bgr8'))

    monkeypatch.setattr(snapshot.rclpy, 'spin_once', spin)
    assert snapshot.main() != 0
    assert 'save failed' in harness.logger.error.call_args.args[0]
