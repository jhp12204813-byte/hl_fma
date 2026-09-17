"""ROS adapter for lane perception; publishes no motion commands."""
import cv2
import json
from cv_bridge import CvBridge, CvBridgeError
import rclpy
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from fma_interfaces.msg import Lane

from fma_perception.lane_detection import LaneConfig, detect_lane
from fma_perception.bev_lane import BEVConfig, BEVLaneDetector


class LaneDetectorNode(Node):
    def __init__(self):
        super().__init__('lane_detector')
        defaults = LaneConfig()
        self.parameter_names = tuple(vars(defaults))
        self.calibration_names = {'lane_width_m', 'meters_per_pixel_y', 'single_lane_width_px'}
        self.config = LaneConfig(**{
            k: self.declare_parameter(k, v, ParameterDescriptor(read_only=k in self.calibration_names)).value
            for k, v in vars(defaults).items()})
        self.pipeline = self.declare_parameter(
            'pipeline', 'hough', ParameterDescriptor(read_only=True)).value
        if self.pipeline not in ('hough', 'bev'):
            raise ValueError('pipeline must be hough or bev')
        self.camera_info = None
        self.bev_detector = None
        if self.pipeline == 'bev':
            values = {}
            for key, value in vars(BEVConfig()).items():
                default = [-1.0]*8 if key.endswith('_points') else value
                value = self.declare_parameter('bev_'+key, default,
                                               ParameterDescriptor(read_only=True)).value
                if key.endswith('_points'):
                    value = () if list(value) == [-1.0]*8 else tuple(value)
                values[key] = value
            self.bev_config = BEVConfig(**values)
            self.bev_detector = BEVLaneDetector(self.bev_config, self.config)
        self.add_on_set_parameters_callback(self.validate_parameters)
        self.bridge = CvBridge()
        self.publisher = self.create_publisher(Lane, '/perception/lane', 1)
        self.debug_publisher = self.create_publisher(
            Image, '/perception/lane/debug_image', qos_profile_sensor_data)
        self.mask_publisher = self.create_publisher(
            Image, '/perception/lane/debug_mask', qos_profile_sensor_data)
        if self.pipeline == 'bev':
            self.bev_publisher = self.create_publisher(
                Image, '/perception/lane/debug_bev', qos_profile_sensor_data)
            self.diagnostics_publisher = self.create_publisher(
                String, '/perception/lane/diagnostics', 1)
            self.info_subscription = self.create_subscription(
                CameraInfo, '/front/color/camera_info', self.on_camera_info, qos_profile_sensor_data)
        self.subscription = self.create_subscription(
            Image, '/front/color/image_raw', self.on_image, qos_profile_sensor_data)
        if not self.config.lane_width_m or not self.config.meters_per_pixel_y:
            self.get_logger().warning('Lane calibration absent: metric detection disabled')

    def read_config(self):
        return LaneConfig(**{k: self.get_parameter(k).value for k in self.parameter_names})

    def validate_parameters(self, parameters):
        # Validate only: ROS commits after all callbacks accept the transaction.
        # Read committed values at the next frame, so rejected batches cannot leak.
        values = vars(self.read_config()).copy()
        for parameter in parameters:
            if parameter.name in self.calibration_names or parameter.name == 'pipeline' or parameter.name.startswith('bev_'):
                return SetParametersResult(successful=False, reason='Calibration is startup-only')
            if parameter.name in values:
                values[parameter.name] = parameter.value
        try:
            LaneConfig(**values)
        except (ValueError, TypeError) as error:
            return SetParametersResult(successful=False, reason=str(error))
        return SetParametersResult(successful=True)

    def on_camera_info(self, msg):
        self.camera_info = dict(k=list(msg.k), d=list(msg.d), width=msg.width,
                                height=msg.height, distortion_model=msg.distortion_model,
                                frame_id=msg.header.frame_id)

    def publish_diagnostics(self, values, header):
        values = dict(values, frame_id=header.frame_id,
                      timestamp_ns=header.stamp.sec*1_000_000_000+header.stamp.nanosec)
        msg = String()
        msg.data = json.dumps(values, allow_nan=False)
        self.diagnostics_publisher.publish(msg)

    def on_image(self, msg):
        config = self.read_config()
        if self.pipeline == 'bev' and config != self.config:
            self.bev_detector = BEVLaneDetector(self.bev_config, config)
        self.config = config
        output = Lane()
        output.header = msg.header
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            if self.pipeline == 'bev':
                camera = self.camera_info
                if camera and (not msg.header.frame_id or camera['frame_id'] != msg.header.frame_id):
                    camera = None
                analysis = self.bev_detector.process(image,
                    msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec, camera)
                result = analysis.estimate
                bev_image = self.bridge.cv2_to_imgmsg(analysis.bev_debug, encoding='bgr8')
                bev_image.header = msg.header
                self.bev_publisher.publish(bev_image)
                self.publish_diagnostics(analysis.diagnostics, msg.header)
            else:
                result = detect_lane(image, self.config)
            for field in ('detected', 'lateral_error_m', 'heading_error_rad',
                          'curvature', 'confidence'):
                setattr(output, field, getattr(result, field))
            debug = self.bridge.cv2_to_imgmsg(result.debug, encoding='bgr8')
            debug.header = msg.header
            self.debug_publisher.publish(debug)
            mask = self.bridge.cv2_to_imgmsg(result.debug_mask, encoding='bgr8')
            mask.header = msg.header
            self.mask_publisher.publish(mask)
        except (ValueError, RuntimeError, cv2.error, CvBridgeError) as error:
            if self.bev_detector is not None:
                self.bev_detector.reset()
                self.publish_diagnostics(dict(status='invalid_image', error=str(error)), msg.header)
            self.get_logger().warning(f'Invalid camera image: {error}', throttle_duration_sec=5.)
        self.publisher.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = LaneDetectorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
