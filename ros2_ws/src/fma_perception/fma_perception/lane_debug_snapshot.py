"""Save the latest images from three lane debug streams, then exit."""
from functools import partial
import math
from pathlib import Path
import time

import cv2
from cv_bridge import CvBridge, CvBridgeError
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Image


TOPICS = {
    '/camera/camera/color/image_raw': 'camera_rgb.jpg',
    '/perception/lane/debug_image': 'lane_debug.jpg',
    '/perception/lane/debug_mask': 'lane_mask.png',
}


class LaneDebugSnapshot(Node):
    def __init__(self):
        super().__init__('lane_debug_snapshot')
        self.output_dir = Path(self.declare_parameter(
            'output_dir', '~/lane_debug/').value).expanduser()
        timeout = float(self.declare_parameter('timeout', 5.0).value)
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError('timeout must be finite and positive')
        self.deadline = time.monotonic() + timeout
        self.frames = {}
        self.exit_code = None
        self.bridge = CvBridge()
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)
        self.image_subscriptions = [
            self.create_subscription(Image, topic, partial(self.on_image, topic), qos)
            for topic in TOPICS
        ]

    def check_timeout(self):
        if self.exit_code is None and time.monotonic() >= self.deadline:
            missing = ', '.join(topic for topic in TOPICS if topic not in self.frames)
            self.get_logger().error(f'Snapshot timeout; missing topics: {missing}')
            self.exit_code = 1

    def on_image(self, topic, msg):
        self.check_timeout()
        if self.exit_code is not None:
            return
        # Keep mono masks as mono; preserve colors in the detector's BGR mask.
        encoding = 'mono8' if topic.endswith('/debug_mask') and msg.encoding == 'mono8' else 'bgr8'
        try:
            self.frames[topic] = self.bridge.imgmsg_to_cv2(msg, desired_encoding=encoding)
        except (CvBridgeError, cv2.error, ValueError, RuntimeError, TypeError) as error:
            self.get_logger().error(f'Image conversion failed for {topic}: {error}')
            self.exit_code = 1
            return
        if len(self.frames) == len(TOPICS):
            try:
                self.output_dir.mkdir(parents=True, exist_ok=True)
                for source, filename in TOPICS.items():
                    path = self.output_dir / filename
                    if not cv2.imwrite(str(path), self.frames[source]):
                        raise OSError(f'Could not write {path}')
            except (OSError, cv2.error) as error:
                self.get_logger().error(f'Snapshot save failed: {error}')
                self.exit_code = 1
                return
            self.get_logger().info(f'Saved lane snapshots to {self.output_dir}')
            self.exit_code = 0


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = LaneDebugSnapshot()
        while rclpy.ok() and node.exit_code is None:
            node.check_timeout()
            if node.exit_code is None:
                rclpy.spin_once(node, timeout_sec=max(0.0, node.deadline - time.monotonic()))
        return node.exit_code if node.exit_code is not None else 1
    except (KeyboardInterrupt, ExternalShutdownException):
        return 1
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
