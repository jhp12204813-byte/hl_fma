"""C920 lane-follow preview. No control publisher exists unless enable_drive=true."""
from dataclasses import dataclass
import importlib.util
import math
from pathlib import Path
import threading
import time
import warnings

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

from fma_interfaces.msg import DriveCommand
from fma_perception.c920_bev import C920BEV
from fma_perception.paper_lane_tracker import PaperLaneConfig, PaperLaneTracker
from fma_vehicle.vehicle_controller_node import ConversionConfig, steering_angle_to_adc


@dataclass(frozen=True)
class FollowConfig:
    lookahead_m: float = 2.0
    lookahead_margin_m: float = 0.10
    min_observed_span_m: float = 0.30
    lateral_gain: float = 0.30  # rad/m
    heading_gain: float = 0.60  # rad/rad


def calculate_command(lane, bev, config=FollowConfig(), *, field_stop_test=False,
                      allow_single_side_test=False, single_side_control_max_m=3.30,
                      pair_control_max_m=3.0):
    """Left-positive errors and steering; BEV x right, y forward, v backward."""
    invalid = {'valid': False, 'reason': 'invalid_pair', 'lateral_error_m': None,
               'heading_error_deg': None, 'steering_cmd_rad': 0.0, 'expected_adc': None,
               'desired_lookahead': config.lookahead_m, 'used_lookahead': None,
               'observed_min': None, 'observed_max': None, 'control_source': 'NONE'}
    center = None
    source = 'NONE'
    if lane.get('pair_state') == 'PAIR_VALID' and (lane.get('pair_quality') or {}).get('valid'):
        center = lane.get('center')
        source = 'PAIR'
    elif (field_stop_test and allow_single_side_test
          and lane.get('pair_state') not in ('PAIR_VALID', 'PAIR_WEAK')
          and lane.get('state') in ('LEFT_ONLY', 'RIGHT_ONLY')):
        side = 'left' if lane['state'] == 'LEFT_ONLY' else 'right'
        if lane.get(side) is None:
            return {**invalid, 'reason': 'no_visible_boundary'}
        # Only the tracker's measured-pair temporary center is trusted. No
        # controller-generated center, assumed lane width, or side-fit fallback.
        if lane.get('center_source') == lane['state'] + '_OFFSET':
            center = lane.get('temporary_center')
            source = 'TEMPORARY_CENTER'
    else:
        return invalid
    if center is None:
        return {**invalid, 'reason': 'no_center'}
    invalid['control_source'] = source
    y_min, y_max = float(center['y_min']), float(center['y_max'])
    if not math.isfinite(y_min) or not math.isfinite(y_max) or y_max < y_min:
        return {**invalid, 'reason': 'invalid_observed_range'}
    # Intersect observed rows with the BEV; never extrapolate the center fit.
    y_min, y_max = max(0.0, y_min), min(bev.height - 1.0, y_max)
    if y_max < y_min:
        return {**invalid, 'reason': 'invalid_observed_range'}
    ego_u, _ = bev.ground_to_bev_pixel(0, 0)
    observed_min, observed_max = sorted(
        bev.bev_pixel_to_ground(ego_u, y)[1] for y in (y_min, y_max))
    invalid.update(observed_min=observed_min, observed_max=observed_max)
    span = observed_max - observed_min
    if span < config.min_observed_span_m or span <= 2 * config.lookahead_margin_m:
        return {**invalid, 'reason': 'observed_center_too_short'}
    allowed_min = max(1.8, observed_min + config.lookahead_margin_m)
    control_max = 3.0
    if field_stop_test and source == 'PAIR':
        if not math.isfinite(pair_control_max_m) or not 1.8 <= pair_control_max_m < 4.0:
            return {**invalid, 'reason': 'invalid_pair_control_max'}
        control_max = pair_control_max_m
    if field_stop_test and allow_single_side_test and source == 'TEMPORARY_CENTER':
        if not math.isfinite(single_side_control_max_m) or not 1.8 <= single_side_control_max_m <= 5.20:
            return {**invalid, 'reason': 'invalid_single_side_control_max'}
        control_max = single_side_control_max_m
    allowed_max = min(control_max, observed_max - config.lookahead_margin_m)
    if allowed_min > allowed_max:
        return {**invalid, 'reason': 'outside_control_window'}
    used = min(allowed_max, max(allowed_min, config.lookahead_m))
    _, v = bev.ground_to_bev_pixel(0, used)
    coeff = np.asarray(center['coefficients'], dtype=float)
    if coeff.shape != (3,) or not np.all(np.isfinite(coeff)):
        return {**invalid, 'reason': 'nonfinite_center'}
    u = float(np.polyval(coeff, v))
    slope = float(np.polyval(np.polyder(coeff), v))
    if not math.isfinite(u) or not math.isfinite(slope) or not 0 <= u < bev.width:
        return {**invalid, 'reason': 'target_outside_bev'}
    x, _ = bev.bev_pixel_to_ground(u, v)
    lateral = -x
    heading = math.atan(-slope)
    calibration = ConversionConfig()
    steering = max(calibration.steering_right_angle_rad,
                   min(calibration.steering_left_angle_rad,
                       config.lateral_gain * lateral + config.heading_gain * heading))
    return {**invalid, 'control_source': source, 'used_lookahead': used, 'valid': True, 'reason': source.lower() + '_valid', 'lateral_error_m': lateral,
            'heading_error_deg': math.degrees(heading), 'steering_cmd_rad': steering,
            'expected_adc': steering_angle_to_adc(steering), 'target_px': (u, v)}


def process_lane(tracker, mask):
    # Limit warning handling to this call; no fitting/numerical changes.
    rank_warning = np.exceptions.RankWarning if hasattr(np, 'exceptions') else np.RankWarning
    with warnings.catch_warnings(record=True) as caught:
        warnings.simplefilter('always', rank_warning)
        result = tracker.process(mask)
    count = 0
    for item in caught:
        if issubclass(item.category, rank_warning):
            count += 1
        else:
            warnings.warn_explicit(item.message, item.category, item.filename, item.lineno)
    return result, count


class LaneFollowNode(Node):
    def __init__(self, field_stop_test=False, competition_tracking=False):
        super().__init__('lane_follow')
        share = Path(get_package_share_directory('fma_control'))
        defaults = {'enable_drive': False, 'device': '/dev/video0',
                    'config': str(share / 'config/c920_bev_calibration.yaml'),
                    'lane_min_width_m': 3.50,
                    'pair_control_max_m': 3.0,
                    'max_frame_age_sec': 0.25, **vars(FollowConfig())}
        if field_stop_test:
            from fma_perception.stop_line_detector import StopLineConfig
            defaults['pair_control_max_m'] = 3.5
            defaults['drive_pwm'] = 40
            defaults['stop_approach_pwm'] = 40
            defaults['stop_approach_timeout_sec'] = 3.0
            defaults['stop_trigger_distance_m'] = 0.60
            defaults['allow_single_side_test'] = False
            defaults['single_side_control_max_m'] = 3.30
            defaults['stop_min_thickness_m'] = StopLineConfig().min_thickness_m
        if competition_tracking:
            from fma_perception.competition_lane_tracker import MetricLaneConfig
            defaults.update(drive_pwm=120, single_drive_pwm=64, degraded_pwm=40,
                            pair_return_blend_sec=.3, obstacle_stop_topic='/safety/obstacle_stop',
                            c920_lock_exposure=True, c920_manual_exposure=156, c920_gain=0)
            defaults.update(vars(MetricLaneConfig()))
        self.options = {name: self.declare_parameter(name, value,
                        ParameterDescriptor(read_only=True)).value for name, value in defaults.items()}
        for name in ('lookahead_m', 'lateral_gain', 'heading_gain', 'max_frame_age_sec', 'lane_min_width_m',
                     'lookahead_margin_m', 'min_observed_span_m'):
            value = self.options[name]
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f'{name} must be finite and positive')
        if self.options['enable_drive'] and self.options['lane_min_width_m'] != 3.50 and not field_stop_test:
            raise ValueError('lane_min_width_m override is DRY-RUN only; DRIVE requires 3.50 m')
        pair_max = self.options['pair_control_max_m']
        if not math.isfinite(pair_max) or not 1.8 <= pair_max < 4.0:
            raise ValueError('pair_control_max_m must be within [1.8, 4.0) m')
        if not field_stop_test and pair_max != 3.0:
            raise ValueError('pair_control_max_m override is field-test only; production requires 3.0 m')
        if field_stop_test:
            pwm = self.options['drive_pwm']
            if type(pwm) is not int or not 0 <= pwm <= 799:
                raise ValueError('drive_pwm must be an integer within 0..799')
            approach_pwm = self.options['stop_approach_pwm']
            if type(approach_pwm) is not int or not 0 <= approach_pwm <= 799:
                raise ValueError('stop_approach_pwm must be an integer within 0..799')
            timeout = self.options['stop_approach_timeout_sec']
            if not math.isfinite(timeout) or timeout <= 0:
                raise ValueError('stop_approach_timeout_sec must be finite and positive')
            distance = self.options['stop_trigger_distance_m']
            if not math.isfinite(distance) or distance <= 0:
                raise ValueError('stop_trigger_distance_m must be finite and positive')
            control_max = self.options['single_side_control_max_m']
            if not math.isfinite(control_max) or not 1.8 <= control_max <= 5.20:
                raise ValueError('single_side_control_max_m must be within 1.8..5.20 m')
            thickness = self.options['stop_min_thickness_m']
            if not math.isfinite(thickness) or not 0 < thickness <= StopLineConfig().max_thickness_m:
                raise ValueError('stop_min_thickness_m must be positive and within detector maximum')
        self.follow_config = FollowConfig(**{name: self.options[name] for name in vars(FollowConfig())})
        if competition_tracking:
            from fma_perception.competition_lane_tracker import CompetitionLaneTracker, MetricLaneConfig
            metric_config = MetricLaneConfig(**{name: self.options[name] for name in vars(MetricLaneConfig())})
            for name in ('single_drive_pwm', 'degraded_pwm'):
                value = self.options[name]
                if type(value) is not int or not 0 <= value <= 799:
                    raise ValueError(f'{name} must be an integer in 0..799')
            if self.options['degraded_pwm'] > self.options['single_drive_pwm']:
                raise ValueError('degraded_pwm must not exceed single_drive_pwm')
            if type(self.options['c920_lock_exposure']) is not bool:
                raise ValueError('c920_lock_exposure must be boolean')
            exposure = self.options['c920_manual_exposure']
            gain = self.options['c920_gain']
            if type(exposure) is not int or not 3 <= exposure <= 2047:
                raise ValueError('c920_manual_exposure must be an integer in 3..2047')
            if type(gain) is not int or not 0 <= gain <= 255:
                raise ValueError('c920_gain must be an integer in 0..255')
            from fma_control.competition_lane_control import PairReturnBlend
            self.pair_return_blend = PairReturnBlend(self.options['pair_return_blend_sec'])
        self.bev = C920BEV(self.options['config'])
        if (self.bev.image_width, self.bev.image_height) != (640, 480):
            raise ValueError('C920 capture requires matching 640x480 calibration')
        # Package the existing replay source unchanged; reuse its exact HSV/mask code.
        spec = importlib.util.spec_from_file_location('c920_lane_replay', share / 'tools/replay_c920_lane.py')
        replay = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(replay)
        self.make_mask = replay.make_mask
        self.tracker = PaperLaneTracker(PaperLaneConfig(
            min_pair_width_px=self.options['lane_min_width_m'] / self.bev.resolution_m),
            diagnostics=field_stop_test)
        if competition_tracking:
            self.competition_tracker = CompetitionLaneTracker(self.bev, metric_config)
        self.publisher = (self.create_publisher(DriveCommand, '/cmd/lane', 1)
                          if self.options['enable_drive'] else None)
        self.lock = threading.Lock()
        self.latest = None
        self.failure = None
        self.rank_warnings = 0
        self.last_log = -math.inf
        self.stopping = threading.Event()
        self.timer = self.create_timer(.05, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.worker = threading.Thread(target=self.capture, daemon=True)
        self.worker.start()

    def process_frame(self, frame, frame_stamp=None):
        _, mask, _ = self.make_mask(frame, self.bev)
        lane, count = process_lane(self.tracker, mask)
        return calculate_command(lane, self.bev, self.follow_config), count

    def diagnostic(self, result, fps):
        return ''

    def capture(self):
        cap = None
        try:
            cap = cv2.VideoCapture(self.options['device'], cv2.CAP_V4L2)
            if not cap.isOpened():
                raise RuntimeError('C920 open failed')
            for prop, value in ((cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG')),
                                (cv2.CAP_PROP_FRAME_WIDTH, 640), (cv2.CAP_PROP_FRAME_HEIGHT, 480),
                                (cv2.CAP_PROP_FPS, 30), (cv2.CAP_PROP_BUFFERSIZE, 1)):
                cap.set(prop, value)

            if self.options.get('c920_lock_exposure', False):
                from fma_control.c920_camera_controls import apply_c920_manual_controls
                controls = apply_c920_manual_controls(
                    self.options['device'],
                    self.options['c920_manual_exposure'],
                    self.options['c920_gain'],
                )
                self.get_logger().info(
                    'C920 competition controls locked: ' + controls.replace('\n', ', ')
                )

            previous = time.monotonic()
            while not self.stopping.is_set():
                # Timestamp BEFORE read so a blocked read/slow fit cannot create fresh motion.
                started = time.monotonic()
                ok, frame = cap.read()
                if not ok or frame.shape[:2] != (480, 640):
                    raise RuntimeError('C920 read failed or frame size mismatch')
                result, count = self.process_frame(frame, frame_stamp=started)
                now = time.monotonic()
                with self.lock:
                    self.latest = (started, result, 1 / max(now - previous, 1e-9))
                    self.rank_warnings += count
                previous = now
        except Exception as error:
            with self.lock:
                self.failure = str(error)
                self.latest = None
        finally:
            if cap is not None:
                cap.release()

    def requested_drive_pwm(self):
        return 40

    def publish_result(self, result):
        self.published_pwm = 0
        if self.publisher is None:
            return
        msg = DriveCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pwm_control = True
        msg.drive_pwm = self.requested_drive_pwm() if result['valid'] else 0
        msg.speed_mps = .2 if result['valid'] else 0.0  # direction only, NOT m/s control
        msg.steering_angle_rad = result['steering_cmd_rad'] if result['valid'] else 0.0
        msg.emergency_stop = not result['valid']
        self.publisher.publish(msg)
        self.published_pwm = msg.drive_pwm

    def tick(self):
        now = time.monotonic()
        with self.lock:
            latest, failure, count = self.latest, self.failure, self.rank_warnings
        result = {'valid': False, 'reason': failure or 'no_fresh_frame',
                  'lateral_error_m': None, 'heading_error_deg': None,
                  'steering_cmd_rad': 0.0, 'expected_adc': None}
        fps = 0.0
        if latest is not None:
            started, calculated, fps = latest
            if 0 <= now - started < self.options['max_frame_age_sec'] and not failure:
                result = calculated
        self.publish_result(result)
        if now - self.last_log >= 1.0:
            mode = (f'DRIVE ENABLED ({self.requested_drive_pwm() / 8:g}% duty)'
                    if self.publisher is not None else 'DRY-RUN')
            if getattr(self, 'competition_tracking', False):
                mode = (f'drive_enabled={self.publisher is not None}'
                        f" configured_pair_pwm={self.options['drive_pwm']}"
                        f" configured_single_pwm={self.options['single_drive_pwm']}"
                        f" published_pwm={getattr(self, 'published_pwm', 0)}")
            def metric(value):
                return 'N/A' if value is None else f'{value:.2f}m'
            diagnostic = (
                f" desired_lookahead={metric(self.follow_config.lookahead_m)}"
                f" used_lookahead={metric(result.get('used_lookahead'))}"
                f" observed_min={metric(result.get('observed_min'))}"
                f" observed_max={metric(result.get('observed_max'))}")
            self.get_logger().info(
                f"{mode} lane_min_width_m={self.options['lane_min_width_m']:.2f} LANE VALID={result['valid']} reason={result['reason']} "
                f"lateral_error_m={result['lateral_error_m']} "
                f"heading_error_deg={result['heading_error_deg']} "
                f"steering_cmd_rad={result['steering_cmd_rad']:.4f} "
                f"expected_adc={result['expected_adc']} FPS={fps:.1f} RankWarnings_total={count}{diagnostic}{self.diagnostic(result, fps)}")
            self.last_log = now

    def destroy_node(self):
        self.timer.cancel()
        self.stopping.set()
        for _ in range(3):
            try:
                self.publish_result({'valid': False})
                if self.publisher is not None:
                    time.sleep(.05)
            except Exception as error:
                self.get_logger().error(f'Cleanup STOP failed: {error}')
        self.worker.join(timeout=1.0)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = None
    try:
        node = LaneFollowNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
