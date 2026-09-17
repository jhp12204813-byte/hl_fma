"""Real local codecs and cv_bridge; ROS transport is mocked."""
from types import SimpleNamespace
from unittest.mock import MagicMock
import hashlib

import cv2
import numpy as np
import pytest
from builtin_interfaces.msg import Time
from sensor_msgs.msg import Image

from fma_perception import lane_media_replay as replay


@pytest.fixture
def video(tmp_path):
    path = tmp_path / 'sample.avi'
    writer = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*'MJPG'), 12., (64, 48))
    assert writer.isOpened()
    for value in (30, 100, 220):
        writer.write(np.full((48, 64, 3), value, np.uint8))
    writer.release()
    return path


def test_video_loop_resolution_fps_and_source_unchanged(video):
    before = hashlib.sha256(video.read_bytes()).digest()
    source = replay.MediaSource(str(video))
    try:
        assert (source.width, source.height, source.fps) == (64, 48, 12.)
        frames = [source.read() for _ in range(7)]
        assert all(f.shape == (48, 64, 3) for f in frames)
        assert [round(float(f.mean())) for f in frames] == [30, 100, 220, 30, 100, 220, 30]
    finally:
        source.close()
        source.close()
    assert hashlib.sha256(video.read_bytes()).digest() == before


def test_no_loop_eof_and_override(video):
    source = replay.MediaSource(str(video), fps_override=5., loop=False)
    assert source.fps == 5.
    try:
        assert all(source.read() is not None for _ in range(3))
        assert source.read() is None
        assert source.read() is None
    finally:
        source.close()


@pytest.mark.parametrize('suffix', ['jpg', 'jpeg', 'png'])
def test_image_repeat_and_single(tmp_path, suffix):
    path = tmp_path / f'photo.{suffix}'
    assert cv2.imwrite(str(path), np.full((40, 60, 3), 120, np.uint8))
    source = replay.MediaSource(str(path), fps_override=7.)
    assert source.fps == 7. and source.source_fps == 0.
    np.testing.assert_array_equal(source.read(), source.read())
    single = replay.MediaSource(str(path), loop=False)
    assert single.fps == 30.
    assert single.read() is not None and single.read() is None


@pytest.mark.parametrize('name', ['missing.mp4', 'corrupt.mp4', 'corrupt.png', 'unsupported.txt'])
def test_invalid_path(tmp_path, name):
    path = tmp_path / name
    if not name.startswith('missing'):
        path.write_bytes(b'not a media file')
    with pytest.raises(ValueError):
        replay.MediaSource(str(path))


@pytest.mark.parametrize('fps', [-1., float('inf'), float('nan')])
def test_invalid_fps(video, fps):
    with pytest.raises(ValueError, match='fps_override'):
        replay.MediaSource(str(video), fps_override=fps)


@pytest.mark.parametrize('fps', [0., float('nan'), -1.])
def test_unknown_fps_fallback(video, monkeypatch, fps):
    capture = MagicMock()
    capture.isOpened.return_value = True
    capture.read.return_value = (True, np.zeros((48, 64, 3), np.uint8))
    capture.get.return_value = fps
    monkeypatch.setattr(replay.cv2, 'VideoCapture', lambda path: capture)
    source = replay.MediaSource(str(video))
    assert source.fps == 30.
    source.close()
    capture.release.assert_called_once()


def test_video_publish_contract_and_eof(video, monkeypatch):
    params = {'media_path': str(video), 'loop': False}
    monkeypatch.setattr(replay.Node, '__init__', lambda self, name: None)
    monkeypatch.setattr(replay.Node, 'declare_parameter',
                        lambda self, name, default, descriptor:
                        SimpleNamespace(value=params.get(name, default)))
    publisher, timer, clock = MagicMock(), MagicMock(), MagicMock()
    clock.now.return_value.to_msg.side_effect = [Time(sec=i) for i in (10, 11, 12)]
    create_pub = MagicMock(return_value=publisher)
    create_timer = MagicMock(return_value=timer)
    monkeypatch.setattr(replay.Node, 'create_publisher', create_pub)
    monkeypatch.setattr(replay.Node, 'create_timer', create_timer)
    monkeypatch.setattr(replay.Node, 'get_clock', lambda self: clock)
    monkeypatch.setattr(replay.Node, 'get_logger', lambda self: MagicMock())
    monkeypatch.setattr(replay.Node, 'destroy_node', MagicMock())
    node = replay.LaneMediaReplay()
    assert create_pub.call_args.args == (Image, '/front/color/image_raw', replay.qos_profile_sensor_data)
    assert create_timer.call_args.args[0] == pytest.approx(1 / 12)
    for _ in range(4):
        node.publish_frame()
    assert publisher.publish.call_count == 3
    for i, call in enumerate(publisher.publish.call_args_list):
        msg = call.args[0]
        assert (msg.width, msg.height, msg.encoding) == (64, 48, 'bgr8')
        assert msg.header.frame_id == 'front_camera'
        assert msg.header.stamp.sec == 10 + i
        assert float(node.bridge.imgmsg_to_cv2(msg).mean()) == pytest.approx((30, 100, 220)[i], abs=2)
    timer.cancel.assert_called_once()
    assert node.source.capture is None
    node.destroy_node()
