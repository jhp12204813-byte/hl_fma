"""ROS subscriptions and session lifecycle. Never publishes vehicle commands."""
import copy
import threading
import time
import xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
import cv2
from cv_bridge import CvBridge
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, Imu, CameraInfo
from realsense2_camera_msgs.msg import Metadata
from realsense2_camera_msgs.srv import DeviceInfo
from rcl_interfaces.srv import GetParameters
from rosidl_runtime_py.convert import message_to_ordereddict
from ament_index_python.packages import get_package_share_directory
from .writer import Writer
from .device import probe

PREFIX = '/camera/camera'
TOPICS = dict(rgb=PREFIX+'/color/image_raw', depth=PREFIX+'/aligned_depth_to_color/image_raw',
              imu=PREFIX+'/imu', device_metadata=PREFIX+'/color/metadata',
              color_info=PREFIX+'/color/camera_info', depth_info=PREFIX+'/aligned_depth_to_color/camera_info')
EXPECTED = {'rgb_camera.color_profile': '640,480,30', 'depth_module.depth_profile': '640,480,30',
            'enable_accel': True, 'enable_gyro': True, 'unite_imu_method': 2,
            'enable_sync': False, 'align_depth.enable': True}


def header(message):
    return dict(timestamp_ns=message.header.stamp.sec*1_000_000_000 + message.header.stamp.nanosec,
                frame_id=message.header.frame_id, receive_monotonic_ns=time.monotonic_ns(),
                receive_wall_ns=time.time_ns())


def imu_data(message):
    def vector(value, names):
        return {name: float(getattr(value, name)) for name in names}
    return dict(angular_velocity=vector(message.angular_velocity, ('x', 'y', 'z')),
                linear_acceleration=vector(message.linear_acceleration, ('x', 'y', 'z')),
                orientation=vector(message.orientation, ('x', 'y', 'z', 'w')),
                angular_velocity_covariance=list(message.angular_velocity_covariance),
                linear_acceleration_covariance=list(message.linear_acceleration_covariance),
                orientation_covariance=list(message.orientation_covariance))


class Recorder:
    def __init__(self, node, root=None):
        # Avoid nested OpenCV thread pools competing with ROS/Qt and depth workers.
        cv2.setNumThreads(1)
        self.node, self.root = node, Path(root).expanduser() if root is not None else Path.home() / 'd435i_recordings_ku'
        self.bridge = CvBridge()
        self.lock = threading.RLock()
        self.writer = None
        self.finishing = False
        self.result = None
        self.path = None
        self.preview = None
        self.camera_info = {}
        self.last_seen = {}
        self.last_received = {}
        self.receive_stats = {}
        self.stop_details = None
        self.errors = []
        self.device = None
        self.parameters = None
        self.subscriptions = []
        for kind, typ in (('rgb', Image), ('depth', Image), ('imu', Imu), ('device_metadata', Metadata)):
            # Absorb short executor scheduling bursts without a 5-sample DDS cache loss.
            qos = QoSProfile(depth=400 if kind == 'imu' else 30,
                             reliability=ReliabilityPolicy.RELIABLE if typ is Image else qos_profile_sensor_data.reliability,
                             durability=qos_profile_sensor_data.durability)
            self.subscriptions.append(node.create_subscription(typ, TOPICS[kind],
                lambda msg, key=kind: self.receive(key, msg), qos))
        for key, topic in (('color', 'color_info'), ('depth', 'depth_info')):
            self.subscriptions.append(node.create_subscription(CameraInfo, TOPICS[topic],
                lambda msg, name=key: self.info(name, msg), qos_profile_sensor_data))
        self.device_client = node.create_client(DeviceInfo, PREFIX+'/device_info')
        self.param_client = node.create_client(GetParameters, PREFIX+'/get_parameters')
        self.device_future = self.param_future = None
        self.timer = node.create_timer(.25, self.poll)

    def info(self, name, msg):
        with self.lock:
            self.camera_info[name] = message_to_ordereddict(msg)

    def poll(self):
        with self.lock:
            now = time.monotonic()
            if (self.device_future is None and now >= getattr(self, 'device_poll_due', 0)
                    and self.device_client.service_is_ready()):
                self.device_future = self.device_client.call_async(DeviceInfo.Request())
                self.device_poll_due = now + 5
            if (self.param_future is None and now >= getattr(self, 'param_poll_due', 0)
                    and self.param_client.service_is_ready()):
                self.param_future = self.param_client.call_async(GetParameters.Request(names=list(EXPECTED)))
                self.param_poll_due = now + 1
            if self.device_future and self.device_future.done():
                try:
                    self.device = message_to_ordereddict(self.device_future.result())
                except Exception:
                    self.device = None
                self.device_future = None
            if self.param_future and self.param_future.done():
                try:
                    values = self.param_future.result().values
                    self.parameters = {name: (v.bool_value if v.type == 1 else v.integer_value if v.type == 2 else v.string_value)
                                       for name, v in zip(EXPECTED, values)}
                except Exception:
                    self.parameters = None
                self.param_future = None
            if self.writer:
                if self.parameters != EXPECTED:
                    self.errors.append('Camera configuration changed or unavailable')
                now = time.monotonic()
                for key in ('rgb', 'depth', 'imu'):
                    if now - self.last_seen.get(key, self.started) >= 3:
                        self.errors.append(f'{key.upper()} timeout, last message {now - self.last_seen.get(key, self.started):.2f}s ago')
                for key in ('rgb', 'depth', 'imu'):
                    if self.node.count_publishers(TOPICS[key]) != 1:
                        self.errors.append(f'{key}: expected exactly one publisher')
                if self.errors or self.writer.errors:
                    self.stop(reason='; '.join(self.errors + list(self.writer.errors)), automatic=True)

    def ready(self):
        if not self.device or self.device.get('serial_number') != '142122070689':
            return 'Waiting for front D435i device_info (142122070689)'
        if not self.parameters or any(self.parameters.get(k) != v for k, v in EXPECTED.items()):
            return f'Camera parameters must match required settings: {self.parameters}'
        if set(self.camera_info) != {'color', 'depth'}:
            return 'Waiting for both CameraInfo streams'
        for key in ('rgb', 'depth', 'imu'):
            if time.monotonic() - self.last_seen.get(key, 0) > 3:
                return f'Waiting for {key}'
            if self.node.count_publishers(TOPICS[key]) != 1:
                return f'{key}: expected exactly one publisher'
        return None

    def start(self):
        with self.lock:
            if self.writer or self.finishing:
                raise RuntimeError('Recording/finalization already active')
            problem = self.ready()
            if problem:
                raise RuntimeError(problem)
            version = ET.parse(Path(get_package_share_directory('realsense2_camera'))/'package.xml').findtext('version')
            if version != '4.58.3':
                raise RuntimeError('ROS depth conversion verified only for wrapper 4.58.3')
            metadata = dict(camera_serial=self.device['serial_number'], firmware=self.device['firmware_version'],
                            device=self.device, driver_version=version, librealsense_version='2.58.3',
                            librealsense_version_source='installed driver startup verified 2026-09-09',
                            configured=dict(rgb=[640, 480, 30], depth=[640, 480, 30]),
                            topics=TOPICS, camera_info=copy.deepcopy(self.camera_info),
                            camera_parameters=self.parameters, depth_scale_m=.001,
                            depth_scale_source='realsense-ros 4.58.3 base_realsense_node.cpp fix_depth_scale: ROS Z16 in mm',
                            device_depth_scale_m=None,
                            device_depth_scale_source='not exposed by running ROS node; distinct from normalized ROS units',
                            warnings=['Device startup reported missing IMU calibration; default intrinsics/extrinsics'],
                            orientation_note='Raw IMU orientation retained; no attitude estimator implemented')
            metadata.update(probe())
            self.writer = Writer(self.root, metadata)
            self.path = self.writer.path
            self.started = time.monotonic()
            self.receive_stats = {}
            self.errors = []
            self.result = None
            self.stop_details = None

    def receive(self, kind, msg):
        meta = header(msg)
        try:
            if kind == 'rgb':
                data = self.bridge.imgmsg_to_cv2(msg, 'bgr8').copy()
                meta['encoding'] = msg.encoding
            elif kind == 'depth':
                if msg.encoding != '16UC1':
                    raise ValueError(f'Unexpected depth encoding {msg.encoding}')
                data = self.bridge.imgmsg_to_cv2(msg, 'passthrough').astype(np.uint16, copy=True)
                meta['encoding'] = msg.encoding
            elif kind == 'imu':
                data = imu_data(msg)
            else:
                data = dict(json_data=msg.json_data)
            with self.lock:
                self.last_seen[kind] = meta['receive_monotonic_ns'] / 1e9
                self.last_received[kind] = dict(meta)
                if kind == 'rgb':
                    self.preview = data
                if self.writer:
                    stats = self.receive_stats.setdefault(kind, dict(count=0,
                        first_receive_monotonic_ns=meta['receive_monotonic_ns'],
                        first_timestamp_ns=meta['timestamp_ns']))
                    stats['count'] += 1
                    stats['last_receive_monotonic_ns'] = meta['receive_monotonic_ns']
                    stats['last_timestamp_ns'] = meta['timestamp_ns']
                    span = (meta['receive_monotonic_ns'] - stats['first_receive_monotonic_ns']) / 1e9
                    stats['receive_hz'] = (stats['count'] - 1) / span if span > 0 else None
                    if not self.writer.submit(kind, data, meta):
                        self.node.get_logger().error(f'Writer rejected {kind}; recording will FAIL')
        except Exception as exc:
            with self.lock:
                if self.writer:
                    self.errors.append(f'{kind}: {exc}')
            self.node.get_logger().error(str(exc))

    def stop(self, reason='Manual stop', automatic=False):
        with self.lock:
            if not self.writer:
                return
            writer, self.writer = self.writer, None
            self.finishing = True
            now = time.monotonic()
            self.stop_details = dict(automatic=automatic, reason=reason,
                message=('AUTO STOP: ' if automatic else 'STOP: ') + reason,
                monotonic_ns=int(now * 1e9), wall_ns=time.time_ns(),
                elapsed_s=now - self.started,
                last_messages={key: dict(getattr(self, 'last_received', {}).get(key, {}),
                    age_s=now - self.last_seen.get(key, self.started)) for key in ('rgb', 'depth', 'imu')})
            self.node.get_logger().warning(self.stop_details['message'])
            extra = dict(camera_info=copy.deepcopy(self.camera_info), recording_errors=list(self.errors),
                         stop_details=copy.deepcopy(self.stop_details),
                         received_streams=copy.deepcopy(getattr(self, 'receive_stats', {})))
        def finish():
            try:
                self.result = writer.close(extra)
            except Exception as exc:
                self.result = dict(status='FAIL', failures=[f'Finalization: {exc}'])
            finally:
                self.finishing = False
        self.finalizer = threading.Thread(target=finish, name='d435i-verify')
        self.finalizer.start()

    def shutdown(self):
        self.stop(reason='Recorder shutdown')
        if hasattr(self, 'finalizer'):
            self.finalizer.join()
        self.node.destroy_timer(self.timer)
        for sub in self.subscriptions:
            self.node.destroy_subscription(sub)
        self.node.destroy_client(self.device_client)
        self.node.destroy_client(self.param_client)
