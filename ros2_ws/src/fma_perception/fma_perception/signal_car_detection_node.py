"""D435i RGB signal-car perception -> LaneSignal."""

import sys
from pathlib import Path

import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from fma_interfaces.msg import LaneSignal


class SignalCarDetectionNode(Node):

    def __init__(self):
        super().__init__('signal_car_detection')

        self.declare_parameter(
            'repo_root',
            str(Path.home() / 'fma_autonomous_vehicle'),
        )
        self.declare_parameter(
            'color_topic',
            '/camera/camera/color/image_raw',
        )
        self.declare_parameter(
            'detector_weights',
            'autonomous_vision/models/detector_best.pt',
        )
        self.declare_parameter(
            'classifier_weights',
            'autonomous_vision/models/sign_classifier_best.pt',
        )

        self.declare_parameter('detector_confidence', 0.35)
        self.declare_parameter('detector_iou', 0.45)
        self.declare_parameter('detector_imgsz', 640)
        self.declare_parameter('classifier_confidence', 0.70)
        self.declare_parameter('classifier_imgsz', 224)
        self.declare_parameter('device', '')

        # Three signal-board slots.
        # SLOT 1 -> LEFT
        # SLOT 2 -> CENTER
        # SLOT 3 -> RIGHT
        self.declare_parameter('left_slot', 1)
        self.declare_parameter('center_slot', 2)
        self.declare_parameter('right_slot', 3)

        repo_root = Path(
            self.get_parameter('repo_root').value
        ).expanduser().resolve()

        if not repo_root.is_dir():
            raise RuntimeError(f'repo_root not found: {repo_root}')

        if str(repo_root) not in sys.path:
            sys.path.insert(0, str(repo_root))

        from autonomous_vision import config
        from autonomous_vision.bbox_tracker import SignPanelTracker
        from autonomous_vision.sign_classifier import (
            SignPanelClassifier,
            SlotStateManager,
        )
        from autonomous_vision.utils import clamp_bbox
        from autonomous_vision.yolo_detector import YOLODetector

        self.config = config
        self.clamp_bbox = clamp_bbox

        detector_path = repo_root / self.get_parameter(
            'detector_weights'
        ).value
        classifier_path = repo_root / self.get_parameter(
            'classifier_weights'
        ).value

        if not detector_path.is_file():
            raise RuntimeError(
                f'detector model not found: {detector_path}'
            )

        if not classifier_path.is_file():
            raise RuntimeError(
                f'classifier model not found: {classifier_path}'
            )

        device = self.get_parameter('device').value.strip() or None

        self.detector = YOLODetector(
            detector_path,
            confidence=self.get_parameter(
                'detector_confidence'
            ).value,
            iou=self.get_parameter('detector_iou').value,
            image_size=self.get_parameter(
                'detector_imgsz'
            ).value,
            device=device,
        )

        self.classifier = SignPanelClassifier(
            classifier_path,
            image_size=self.get_parameter(
                'classifier_imgsz'
            ).value,
            confidence_threshold=self.get_parameter(
                'classifier_confidence'
            ).value,
            device=device,
        )

        self.panel_tracker = SignPanelTracker()
        self.slot_manager = SlotStateManager()

        self.left_slot = int(
            self.get_parameter('left_slot').value
        )
        self.center_slot = int(
            self.get_parameter('center_slot').value
        )
        self.right_slot = int(
            self.get_parameter('right_slot').value
        )

        slots = (
            self.left_slot,
            self.center_slot,
            self.right_slot,
        )

        if any(slot not in (1, 2, 3) for slot in slots):
            raise ValueError(
                'left_slot, center_slot and right_slot '
                'must be 1, 2, or 3'
            )

        if len(set(slots)) != 3:
            raise ValueError(
                'left_slot, center_slot and right_slot '
                'must all be different'
            )

        self.bridge = CvBridge()

        self.publisher = self.create_publisher(
            LaneSignal,
            '/perception/lane_signal',
            10,
        )

        color_topic = self.get_parameter('color_topic').value

        self.subscription = self.create_subscription(
            Image,
            color_topic,
            self.image_callback,
            qos_profile_sensor_data,
        )

        self.last_snapshot = None

        self.get_logger().info(
            'Signal-car perception ready: '
            f'color={color_topic}, '
            f'LEFT<-SLOT{self.left_slot}, '
            f'CENTER<-SLOT{self.center_slot}, '
            f'RIGHT<-SLOT{self.right_slot}'
        )

    def extract_panel_crops(self, frame, tracked_panels):
        crops = {}

        for panel in tracked_panels:
            if not panel.observed or panel.bbox is None:
                continue

            bounds = self.clamp_bbox(
                panel.bbox,
                frame.shape[1],
                frame.shape[0],
            )

            if bounds is None:
                continue

            x1, y1, x2, y2 = bounds

            if (
                x2 - x1 < self.config.MIN_SIGN_PANEL_WIDTH
                or y2 - y1 < self.config.MIN_SIGN_PANEL_HEIGHT
            ):
                continue

            crops[panel.slot] = frame[y1:y2, x1:x2]

        return crops

    @staticmethod
    def to_lane_signal_state(stable_state):
        if stable_state == 'GREEN_ARROW':
            return LaneSignal.OPEN

        if stable_state in ('RED_X', 'LANE_CHANGE'):
            return LaneSignal.CLOSED

        return LaneSignal.UNKNOWN

    def slot_result(self, states, slot):
        for state in states:
            if state.slot == slot:
                signal_state = self.to_lane_signal_state(
                    state.stable_state
                )

                confidence = (
                    float(
                        self.slot_manager.stable_confidences[
                            slot - 1
                        ]
                    )
                    if signal_state != LaneSignal.UNKNOWN
                    else 0.0
                )

                return signal_state, confidence, state.stable_state

        return LaneSignal.UNKNOWN, 0.0, 'UNKNOWN'

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding='bgr8',
            )

            detections = self.detector.detect(frame)

            panel_detections = [
                item
                for item in detections
                if item.class_name
                == self.config.SIGN_PANEL_CLASS_NAME
            ]

            tracked_panels = self.panel_tracker.update(
                panel_detections
            )

            crops = self.extract_panel_crops(
                frame,
                tracked_panels,
            )

            predictions = {}

            if crops:
                ordered_slots = sorted(crops)

                batch = self.classifier.classify_batch(
                    [crops[slot] for slot in ordered_slots]
                )

                predictions = dict(
                    zip(ordered_slots, batch)
                )

            states = self.slot_manager.update(
                tracked_panels,
                predictions,
            )

            left_state, left_conf, left_raw = self.slot_result(
                states,
                self.left_slot,
            )

            center_state, center_conf, center_raw = self.slot_result(
                states,
                self.center_slot,
            )

            right_state, right_conf, right_raw = self.slot_result(
                states,
                self.right_slot,
            )

            output = LaneSignal()
            output.header = msg.header

            output.left_state = left_state
            output.center_state = center_state
            output.right_state = right_state

            output.left_confidence = left_conf
            output.center_confidence = center_conf
            output.right_confidence = right_conf

            self.publisher.publish(output)

            snapshot = (
                left_state,
                center_state,
                right_state,
                left_raw,
                center_raw,
                right_raw,
            )

            if snapshot != self.last_snapshot:
                self.last_snapshot = snapshot

                self.get_logger().info(
                    'SIGNAL '
                    f'LEFT={left_raw}({left_conf:.2f}) '
                    f'CENTER={center_raw}({center_conf:.2f}) '
                    f'RIGHT={right_raw}({right_conf:.2f})'
                )

        except Exception as error:
            self.get_logger().error(
                f'signal-car inference failed: {error}'
            )


def main(args=None):
    rclpy.init(args=args)

    node = SignalCarDetectionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
