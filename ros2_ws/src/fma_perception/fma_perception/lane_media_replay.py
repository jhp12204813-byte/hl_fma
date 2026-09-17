"""Replay local media as camera images without changing source files."""
import math
from pathlib import Path
import sys

import cv2
from cv_bridge import CvBridge
import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class MediaSource:
    def __init__(self, path, fps_override=0.0, loop=True):
        self.capture = None
        self.image = None
        self.pending = None
        self.loop = loop
        self.finished = False
        if not math.isfinite(fps_override) or fps_override < 0:
            raise ValueError('fps_override must be finite and nonnegative')
        path = Path(path).expanduser()
        if not path.is_file():
            raise ValueError(f'Media file does not exist: {path}')
        try:
            if path.suffix.lower() in ('.jpg', '.jpeg', '.png'):
                self.image = cv2.imread(str(path), cv2.IMREAD_COLOR)
                if self.image is None:
                    raise ValueError(f'Cannot decode image: {path}')
                self.source_fps = 0.0
                frame = self.image
            elif path.suffix.lower() in ('.mp4', '.avi', '.mov', '.mkv'):
                self.capture = cv2.VideoCapture(str(path))
                ok, frame = self.capture.read()
                if not self.capture.isOpened() or not ok:
                    raise ValueError(f'Cannot decode video: {path}')
                self.source_fps = self.capture.get(cv2.CAP_PROP_FPS)
                self.pending = frame
            else:
                raise ValueError(f'Unsupported media extension: {path.suffix}')
            self.height, self.width = frame.shape[:2]
            source_valid = math.isfinite(self.source_fps) and self.source_fps > 0
            self.fps = fps_override or (self.source_fps if source_valid else 30.0)
        except Exception:
            self.close()
            raise

    def read(self):
        if self.finished:
            return None
        if self.image is not None:
            self.finished = not self.loop
            return self.image
        if self.pending is not None:
            frame, self.pending = self.pending, None
            return frame
        ok, frame = self.capture.read()
        if not ok and self.loop:
            self.capture.set(cv2.CAP_PROP_POS_FRAMES, 0)
            ok, frame = self.capture.read()
        if not ok:
            self.finished = True
            return None
        return frame

    def close(self):
        if self.capture is not None:
            self.capture.release()
            self.capture = None


class LaneMediaReplay(Node):
    def __init__(self):
        super().__init__('lane_media_replay')
        self.source = None
        try:
            def param(name, value):
                return self.declare_parameter(
                    name, value, ParameterDescriptor(read_only=True)).value
            path = param('media_path', '')
            loop = param('loop', True)
            fps = param('fps_override', 0.0)
            self.source = MediaSource(path, fps, loop)
            self.bridge = CvBridge()
            self.publisher = self.create_publisher(
                Image, '/front/color/image_raw', qos_profile_sensor_data)
            self.timer = self.create_timer(1.0 / self.source.fps, self.publish_frame)
            self.get_logger().info(
                f'{path}: resolution={self.source.width}x{self.source.height}, '
                f'source_fps={self.source.source_fps}, publish_fps={self.source.fps}, '
                f'loop={loop}; frame_id=front_camera, bgr8')
        except Exception:
            self.destroy_node()
            raise

    def publish_frame(self):
        frame = self.source.read()
        if frame is None:
            self.timer.cancel()
            self.source.close()
            self.get_logger().info('Replay ended (EOF or decode failure)')
            return
        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'front_camera'
        self.publisher.publish(msg)

    def destroy_node(self):
        if self.source is not None:
            self.source.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = LaneMediaReplay()
        rclpy.spin(node)
        return 0
    except (ValueError, cv2.error) as error:
        print(f'lane_media_replay: {error}', file=sys.stderr)
        return 1
    except (KeyboardInterrupt, ExternalShutdownException):
        return 0
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
