"""Shared C920 image source; no vehicle command output."""
import math
import time

import cv2
from cv_bridge import CvBridge, CvBridgeError
import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class C920CameraNode(Node):
    def __init__(self):
        super().__init__('c920_camera')
        self.capture = None
        try:
            defaults = dict(device='/dev/fma_c920', width=1280, height=720,
                            fps=30.0, topic='/camera/front/image_raw', frame_id='front_camera')
            self.config = {k: self.declare_parameter(
                k, v, ParameterDescriptor(read_only=True)).value for k, v in defaults.items()}
            c = self.config
            if c['width'] <= 0 or c['height'] <= 0 or not math.isfinite(c['fps']) or c['fps'] <= 0:
                raise ValueError('Camera dimensions and fps must be positive and finite')
            self.bridge = CvBridge()
            self.publisher = self.create_publisher(Image, c['topic'], qos_profile_sensor_data)
            self.capture = cv2.VideoCapture(c['device'], cv2.CAP_V4L2)
            if not self.capture.isOpened():
                raise RuntimeError(f"Cannot open C920 camera: {c['device']}")
            for prop, value in (
                    (cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG')),
                    (cv2.CAP_PROP_FRAME_WIDTH, c['width']),
                    (cv2.CAP_PROP_FRAME_HEIGHT, c['height']),
                    (cv2.CAP_PROP_FPS, c['fps'])):
                if not self.capture.set(prop, value):
                    self.get_logger().warning(f'Camera rejected property {prop}={value}')
            self.last_warning = -math.inf
            self.timer = self.create_timer(1. / c['fps'], self.publish_frame)
        except Exception:
            self.destroy_node()
            raise

    def publish_frame(self):
        try:
            ok, frame = self.capture.read()
            if not ok or frame is None:
                raise RuntimeError('C920 frame read failed')
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.config['frame_id']
            self.publisher.publish(msg)
        except (cv2.error, CvBridgeError, RuntimeError) as error:
            now = time.monotonic()
            if now - self.last_warning >= 5.:
                self.get_logger().warning(str(error))
                self.last_warning = now

    def destroy_node(self):
        try:
            if self.capture is not None:
                self.capture.release()
                self.capture = None
        finally:
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = C920CameraNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as error:
        rclpy.logging.get_logger('c920_camera').error(str(error))
        return 1
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0
