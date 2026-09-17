"""D435i RGB + aligned depth obstacle detector."""

import math
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from fma_interfaces.msg import Obstacle, ObstacleArray
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from ultralytics import YOLO


class ObstacleDetectionNode(Node):
    def __init__(self):
        super().__init__('obstacle_detection_node')

        self.declare_parameter(
            'model_path',
            '/home/sohyun/fma_autonomous_vehicle/'
            'autonomous_vision/models/obstacle_detector_candidate.pt'
        )
        self.declare_parameter(
            'color_topic',
            '/camera/camera/color/image_raw'
        )
        self.declare_parameter(
            'depth_topic',
            '/camera/camera/aligned_depth_to_color/image_raw'
        )
        self.declare_parameter(
            'camera_info_topic',
            '/camera/camera/color/camera_info'
        )
        self.declare_parameter('confidence', 0.03)
        self.declare_parameter('imgsz', 640)
        self.declare_parameter('max_distance_m', 8.0)
        self.declare_parameter('path_half_width_m', 1.2)
        self.declare_parameter('max_inference_hz', 10.0)
        self.declare_parameter('show_debug', False)

        model_path = self.get_parameter('model_path').value
        self.confidence = float(self.get_parameter('confidence').value)
        self.imgsz = int(self.get_parameter('imgsz').value)
        self.max_distance_m = float(
            self.get_parameter('max_distance_m').value
        )
        self.path_half_width_m = float(
            self.get_parameter('path_half_width_m').value
        )
        self.max_inference_hz = float(
            self.get_parameter('max_inference_hz').value
        )
        self.show_debug = bool(
            self.get_parameter('show_debug').value
        )

        self.bridge = CvBridge()
        self.model = YOLO(model_path)

        expected = {0: 'child_dummy', 1: 'vehicle_obstacle'}
        if self.model.names != expected:
            raise RuntimeError(
                f'Unexpected obstacle classes: {self.model.names}'
            )

        self.depth = None
        self.depth_stamp_ns = None
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None
        self.last_inference = -math.inf

        self.publisher = self.create_publisher(
            ObstacleArray,
            '/perception/obstacles',
            10,
        )

        self.create_subscription(
            Image,
            self.get_parameter('depth_topic').value,
            self.on_depth,
            10,
        )
        self.create_subscription(
            CameraInfo,
            self.get_parameter('camera_info_topic').value,
            self.on_camera_info,
            10,
        )
        self.create_subscription(
            Image,
            self.get_parameter('color_topic').value,
            self.on_color,
            10,
        )

        self.get_logger().info(
            f'Obstacle model loaded: {model_path}'
        )

    def on_camera_info(self, msg):
        self.fx = float(msg.k[0])
        self.fy = float(msg.k[4])
        self.cx = float(msg.k[2])
        self.cy = float(msg.k[5])

    def on_depth(self, msg):
        try:
            depth = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding='passthrough'
            )
        except Exception as exc:
            self.get_logger().warning(f'Depth conversion failed: {exc}')
            return

        if msg.encoding in ('16UC1', 'mono16'):
            depth = depth.astype(np.float32) * 0.001
        else:
            depth = depth.astype(np.float32)

        self.depth = depth
        self.depth_stamp_ns = (
            int(msg.header.stamp.sec) * 1_000_000_000
            + int(msg.header.stamp.nanosec)
        )

    def obstacle_depth(self, box):
        if self.depth is None:
            return None

        h, w = self.depth.shape[:2]

        x1, y1, x2, y2 = box
        bw = max(1.0, x2 - x1)
        bh = max(1.0, y2 - y1)

        # Central ROI avoids most bbox background pixels.
        rx1 = int(max(0, x1 + 0.30 * bw))
        rx2 = int(min(w, x2 - 0.30 * bw))
        ry1 = int(max(0, y1 + 0.30 * bh))
        ry2 = int(min(h, y2 - 0.20 * bh))

        if rx2 <= rx1 or ry2 <= ry1:
            return None

        roi = self.depth[ry1:ry2, rx1:rx2]

        valid = roi[
            np.isfinite(roi)
            & (roi > 0.15)
            & (roi <= self.max_distance_m)
        ]

        if valid.size < 10:
            return None

        # Slightly near-biased percentile reduces background contamination.
        return float(np.percentile(valid, 35))

    def on_color(self, msg):
        now = time.monotonic()

        if self.max_inference_hz > 0:
            minimum_period = 1.0 / self.max_inference_hz
            if now - self.last_inference < minimum_period:
                return

        if self.depth is None or self.fx is None:
            return

        color_stamp_ns = (
            int(msg.header.stamp.sec) * 1_000_000_000
            + int(msg.header.stamp.nanosec)
        )

        if self.depth_stamp_ns is None:
            return

        # Reject badly unsynchronized RGB/depth frames.
        if abs(color_stamp_ns - self.depth_stamp_ns) > 150_000_000:
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding='bgr8'
            )
        except Exception as exc:
            self.get_logger().warning(f'Color conversion failed: {exc}')
            return

        self.last_inference = now
        debug_frame = frame.copy() if self.show_debug else None

        results = self.model.predict(
            frame,
            conf=self.confidence,
            iou=0.45,
            agnostic_nms=True,
            imgsz=self.imgsz,
            verbose=False,
        )

        output = ObstacleArray()
        output.header = msg.header
        output.header.frame_id = 'base_link'

        if not results:
            self.publisher.publish(output)
            return

        result = results[0]

        for box in result.boxes:
            confidence = float(box.conf[0])
            class_id = int(box.cls[0])
            class_name = self.model.names[class_id]
            xyxy = box.xyxy[0].cpu().numpy().astype(float)

            depth_m = self.obstacle_depth(xyxy)
            if depth_m is None:
                if debug_frame is not None:
                    x1, y1, x2, y2 = map(int, xyxy)
                    cv2.rectangle(
                        debug_frame, (x1, y1), (x2, y2),
                        (0, 0, 255), 2
                    )
                    cv2.putText(
                        debug_frame,
                        f'{class_name} {confidence:.2f} NO_DEPTH',
                        (x1, max(25, y1 - 8)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                        (0, 0, 255), 2
                    )
                continue

            x1, y1, x2, y2 = xyxy
            u = 0.5 * (x1 + x2)

            # Aligned depth is in color-camera optical coordinates.
            optical_x = (u - self.cx) * depth_m / self.fx

            # REP-103 body frame:
            # x forward = optical z
            # y left    = -optical x
            x_m = depth_m
            y_m = -optical_x

            obstacle = Obstacle()
            obstacle.x_m = float(x_m)
            obstacle.y_m = float(y_m)
            obstacle.distance_m = float(
                math.hypot(x_m, y_m)
            )
            obstacle.bearing_rad = float(
                math.atan2(y_m, x_m)
            )
            obstacle.in_path = bool(
                abs(y_m) <= self.path_half_width_m
            )
            obstacle.confidence = confidence

            output.obstacles.append(obstacle)

            if debug_frame is not None:
                bx1, by1, bx2, by2 = map(int, xyxy)

                if class_name == 'child_dummy':
                    color = (0, 255, 0)
                else:
                    color = (255, 180, 0)

                cv2.rectangle(
                    debug_frame,
                    (bx1, by1), (bx2, by2),
                    color, 2
                )

                cv2.putText(
                    debug_frame,
                    f'{class_name} {confidence:.2f} {obstacle.distance_m:.2f}m',
                    (bx1, max(25, by1 - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    color,
                    2
                )

                cx = int((bx1 + bx2) / 2)
                cy = int((by1 + by2) / 2)
                cv2.circle(debug_frame, (cx, cy), 5, color, -1)

        self.publisher.publish(output)

        if debug_frame is not None:
            cv2.imshow('FMA D435i Obstacle Detection', debug_frame)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord('q'), 27):
                self.show_debug = False
                cv2.destroyAllWindows()

        if output.obstacles:
            nearest = min(
                output.obstacles,
                key=lambda item: item.distance_m,
            )
            self.get_logger().info(
                f'OBSTACLES={len(output.obstacles)} '
                f'nearest={nearest.distance_m:.2f}m '
                f'y={nearest.y_m:.2f}m '
                f'in_path={nearest.in_path} '
                f'conf={nearest.confidence:.2f}'
            )


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetectionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
