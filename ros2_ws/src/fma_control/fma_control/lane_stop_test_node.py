"""Explicit low-speed field test: existing lane control plus confirmed-stop latch."""
import threading
import time
import math

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.signals import SignalHandlerOptions

from fma_control.lane_follow_node import LaneFollowNode, calculate_command, process_lane
from fma_perception.stop_line_detector import StopLineConfig, StopLineDetector
from fma_perception.stop_line_tracker import StopLineTracker
from fma_interfaces.msg import DriveCommand


def pair_perception_diagnostic(lane, bev, raw_mask):
    """Read final selected fits and pre-tracker BEV mask without modifying either."""
    if lane.get('pair_state') not in ('PAIR_VALID', 'PAIR_WEAK'):
        return None
    _, near_row = bev.ground_to_bev_pixel(0, 1.)
    _, far_row = bev.ground_to_bev_pixel(0, 3.)
    row_lo, row_hi = sorted((near_row, far_row))
    detail = {'pair_state': lane['pair_state']}
    ranges = {}
    for side in ('left', 'right'):
        fit = lane.get(side)
        observed_min = observed_max = None
        if fit is not None:
            lo, hi = float(fit['y_min']), float(fit['y_max'])
            if math.isfinite(lo) and math.isfinite(hi) and lo <= hi:
                lo, hi = max(0., lo), min(bev.height - 1., hi)
                if lo <= hi:
                    observed_min, observed_max = sorted(
                        bev.bev_pixel_to_ground(bev.width / 2., row)[1] for row in (lo, hi))
        ranges[side] = (observed_min, observed_max)
        # These are the actual final RANSAC/joint-refit selections, not pixels
        # near the fitted curve or a count inferred from the fit's row range.
        inliers = fit.get('_inlier_y') if fit is not None else None
        inlier_count = None
        if inliers is not None:
            rows = np.asarray(inliers)
            inlier_count = int(np.count_nonzero((rows >= row_lo) & (rows <= row_hi)))
        candidates = fit.get('_candidate_y') if fit is not None else None
        candidate_count = None
        if candidates is not None:
            rows = np.asarray(candidates)
            candidate_count = int(np.count_nonzero((rows >= row_lo) & (rows <= row_hi)))
        for key, value in {
            'observed_min_m': observed_min, 'observed_max_m': observed_max,
            'y_max_row': fit.get('y_max') if fit is not None else None,
            'points': fit.get('points') if fit is not None else None,
            'residual_px': fit.get('residual_px') if fit is not None else None,
            'inliers_1_3m': inlier_count,
            'selected_inliers_1_3m': inlier_count,
            'candidate_points_1_3m': candidate_count,
        }.items():
            detail[f'{side.upper()}_{key}'] = value
    common_min = common_max = None
    limiting = 'NONE'
    if all(value is not None for bounds in ranges.values() for value in bounds):
        left_min, left_max = ranges['left']
        right_min, right_max = ranges['right']
        common_min, common_max = max(left_min, right_min), min(left_max, right_max)
        limiting = 'BOTH' if left_min == right_min else 'LEFT' if left_min > right_min else 'RIGHT'
        if common_min > common_max:
            common_min = common_max = None
    raw_count = None
    left_raw = right_raw = None
    if isinstance(raw_mask, np.ndarray) and raw_mask.ndim == 2:
        start, end = max(0, math.ceil(row_lo)), min(raw_mask.shape[0], math.floor(row_hi) + 1)
        raw_count = int(np.count_nonzero(raw_mask[start:end])) if start < end else 0
        origin_u, _ = bev.ground_to_bev_pixel(0, 0)
        split = max(0, min(raw_mask.shape[1], math.ceil(origin_u)))
        left_raw = int(np.count_nonzero(raw_mask[start:end, :split])) if start < end else 0
        right_raw = int(np.count_nonzero(raw_mask[start:end, split:])) if start < end else 0
    counts = [detail[f'{side}_inliers_1_3m'] for side in ('LEFT', 'RIGHT')]
    detail.update(pair_common_observed_min_m=common_min,
                  pair_common_observed_max_m=common_max,
                  pair_near_limit_side=limiting, raw_mask_pixels_1_3m=raw_count,
                  LEFT_raw_mask_pixels_1_3m=left_raw, RIGHT_raw_mask_pixels_1_3m=right_raw,
                  selected_inliers_1_3m=sum(counts) if all(c is not None for c in counts) else None)
    return detail


class LaneStopTestNode(LaneFollowNode):
    def __init__(self, competition_tracking=False):
        # Initialize before base starts its camera worker. Events cannot be lost
        # if the worker replaces a snapshot before the publishing timer sees it.
        self.stop_latched = threading.Event()
        self.fault_latched = threading.Event()
        self.started = threading.Event()
        self.stop_detector = None
        self.stop_tracker = StopLineTracker()
        self.stop_lock = threading.Lock()
        self.stop_pending = None
        self.stop_snapshot = None
        self.frame_valid_streak = 0
        self.last_lane_stamp = -math.inf
        self.approach_started = None
        self.approach_failed = False
        self.approach_distance = None
        self.held_steering = 0.0
        self.test_state = 'ACQUIRING'
        self.emergency_latched = threading.Event()
        self.obstacle_stopped = threading.Event()
        self.competition_tracking = competition_tracking
        self.active_lane_pwm = 0
        super().__init__(field_stop_test=True, competition_tracking=competition_tracking)
        self.emergency_subscription = self.create_subscription(
            DriveCommand, '/cmd/emergency', self.on_emergency, 1)
        if competition_tracking:
            from std_msgs.msg import Bool
            self.obstacle_subscription = self.create_subscription(
                Bool, self.options['obstacle_stop_topic'], self.on_obstacle, 1)
        self.stop_worker = threading.Thread(target=self.stop_loop, daemon=True)
        self.stop_worker.start()

    def process_frame(self, frame, frame_stamp=None):
        stamp = time.monotonic() if frame_stamp is None else frame_stamp
        _, mask, white_bev = self.make_mask(frame, self.bev)
        if self.competition_tracking:
            from fma_control.competition_lane_control import calculate_competition_command
            tracking = self.competition_tracker.process(mask, timestamp=stamp)
            result, lane = calculate_competition_command(tracking, self.competition_tracker,
                                                        self.follow_config, self.options)
            result = self.pair_return_blend.apply(result, stamp)
            self.get_logger().info('COMPETITION_LANE ' + ' '.join(
                f'{key}={value}' for key, value in {**result['lane_scoring'],
                    'tracking_source': tracking['source'], 'confidence': tracking['confidence'],
                    'grade': tracking['confidence_grade'],
                    **{key: result[key] for key in ('motion_control_source', 'motion_observed_min',
                       'motion_used_lookahead', 'motion_valid', 'requested_pwm')}}.items()))
            with self.stop_lock:
                self.observe_motion_frame(result, stamp, time.monotonic())
                self.stop_pending = (stamp, white_bev, lane)
            return result, 0
        lane, count = process_lane(self.tracker, mask)
        diagnostic = pair_perception_diagnostic(lane, self.bev, mask)
        if diagnostic is not None:
            def display(value):
                if value is None:
                    return 'N/A'
                return f'{value:.3f}' if isinstance(value, float) else str(value)
            # Every processed pair frame, even if control is INVALID; no 1s throttle.
            self.get_logger().info(
                f'PAIR_PERCEPTION frame_stamp={stamp:.6f} ' + ' '.join(
                    f'{key}={display(value)}' for key, value in diagnostic.items()))
        result = calculate_command(
            lane, self.bev, self.follow_config, field_stop_test=True,
            allow_single_side_test=self.options['allow_single_side_test'],
            pair_control_max_m=self.options['pair_control_max_m'],
            single_side_control_max_m=self.options['single_side_control_max_m'])
        with self.stop_lock:
            self.observe_motion_frame(result, stamp, time.monotonic())
            # One latest-frame mailbox: no detector backlog or second camera.
            self.stop_pending = (stamp, white_bev, lane)
        return result, count

    def stop_loop(self):
        try:
            while not self.stopping.is_set():
                start = time.monotonic()
                with self.stop_lock:
                    sample, self.stop_pending = self.stop_pending, None
                if sample is not None and 0 <= start-sample[0] < .5:
                    self.process_stop(*sample)
                self.stopping.wait(max(0, .2-(time.monotonic()-start)))  # at most 5 Hz
        except Exception as error:
            self.fault_latched.set()
            self.get_logger().error(f'Stop worker failed: {error}')

    def process_stop(self, stamp, white_bev, lane):
        if self.stop_detector is None:
            self.stop_detector = StopLineDetector(resolution_m=self.bev.resolution_m,
                                                 far_m=self.bev.far_m, width_px=self.bev.width,
                                                 config=StopLineConfig(min_thickness_m=self.options['stop_min_thickness_m']))
        stop = self.stop_detector.detect(white_bev, lane)
        # Never confirm or cache a result that finished too late.
        if not 0 <= time.monotonic()-stamp < .5:
            self.stop_tracker.reset()
            return
        if self.stop_snapshot is None or stamp-self.stop_snapshot[0] >= .5:
            self.stop_tracker.reset()
        tracked = self.stop_tracker.update(stop)
        center = stop['stop_line_distance_m']
        selected = next((c for c in stop['candidates'] if c['distance_m'] == center), None)
        thickness = None if selected is None else selected['height'] * self.bev.resolution_m
        result = dict(stop_detected=bool(stop['stop_line_detected']),
                      stop_confirmed=bool(tracked['stop_tracked']), stop_center_m=center,
                      stop_near_edge_m=None if center is None or thickness is None else center-thickness/2,
                      stop_confidence=stop['stop_line_confidence'])
        near = result['stop_near_edge_m']
        if self.approach_started is not None:
            if (not result['stop_confirmed'] or near is None or not math.isfinite(near)
                    or near-self.approach_distance > self.stop_tracker.cfg.max_distance_increase_m):
                self.approach_failed = True
        if (result['stop_confirmed'] and result['stop_detected']
                and near is not None and math.isfinite(near)
                and near <= self.options['stop_trigger_distance_m']):
            self.stop_latched.set()
        with self.stop_lock:
            self.stop_snapshot = (stamp, result)

    def motion_safe(self, result, stamp, now):
        used = result.get('used_lookahead')
        control_max = self.options['pair_control_max_m']
        if not math.isfinite(control_max) or not 1.8 <= control_max < 4.0:
            return False
        source = result.get('control_source')
        if self.competition_tracking:
            source = {'PAIR_TRACK': 'PAIR', 'SINGLE_LEFT_TRACK': 'TEMPORARY_CENTER',
                      'SINGLE_RIGHT_TRACK': 'TEMPORARY_CENTER'}.get(source, source)
        if source == 'TEMPORARY_CENTER' and (self.options['allow_single_side_test'] or self.competition_tracking):
            control_max = min(3.30, self.options['single_side_control_max_m'])
        elif source != 'PAIR':
            return False
        if source == 'PAIR':
            observed_min = result.get('observed_min')
            if observed_min is None or not math.isfinite(observed_min) or observed_min > control_max:
                return False
        snapshot = self.stop_snapshot
        return (result['valid'] and used is not None and 1.8 <= used <= control_max
                and 0 <= now-stamp < self.options['max_frame_age_sec']
                and snapshot is not None and 0 <= now-snapshot[0] < .5
                and not self.stop_latched.is_set() and not self.fault_latched.is_set()
                and not self.failure and not self.emergency_latched.is_set()
                and not self.obstacle_stopped.is_set()
                and not self.stopping.is_set())

    def on_obstacle(self, msg):
        if msg.data:
            self.obstacle_stopped.set()
        else:
            self.obstacle_stopped.clear()

    def on_emergency(self, msg):
        # Same explicit SET/CLEAR policy as the arbiter; silence never releases it.
        if msg.emergency_stop:
            self.emergency_latched.set()
        else:
            self.emergency_latched.clear()

    def observe_motion_frame(self, result, stamp, now):
        """Called once per completed camera frame, under stop_lock."""
        safe_now = self.motion_safe(result, stamp, now)
        if not safe_now:
            self.frame_valid_streak = 0
        elif stamp > self.last_lane_stamp:
            if stamp-self.last_lane_stamp >= self.options['max_frame_age_sec']:
                self.frame_valid_streak = 0
            self.frame_valid_streak = min(5, self.frame_valid_streak + 1)
        self.last_lane_stamp = max(self.last_lane_stamp, stamp)
        result['acquired'] = self.frame_valid_streak >= 5
        result['frame_stamp'] = stamp

    def capture(self):
        try:
            super().capture()
        finally:
            if self.failure:
                self.fault_latched.set()

    def requested_drive_pwm(self):
        if self.test_state == 'STOP_APPROACH':
            return min(self.options['drive_pwm'], self.options['stop_approach_pwm'])
        if self.competition_tracking:
            return self.active_lane_pwm
        return self.options['drive_pwm']

    def approach_check(self, result, now, snapshot):
        if self.approach_started is None:
            return False
        stop = snapshot[1] if snapshot is not None else {}
        near = stop.get('stop_near_edge_m')
        healthy = (not self.stopping.is_set() and not self.failure
                   and not self.fault_latched.is_set()
                   and not self.emergency_latched.is_set()
                   and not self.obstacle_stopped.is_set()
                   and 0 <= now-result.get('frame_stamp', -math.inf) < self.options['max_frame_age_sec']
                   and snapshot is not None and 0 <= now-snapshot[0] < .5
                   and stop.get('stop_confirmed') and near is not None and math.isfinite(near)
                   and now-self.approach_started < self.options['stop_approach_timeout_sec'])
        if healthy and self.approach_distance is not None:
            healthy = near-self.approach_distance <= self.stop_tracker.cfg.max_distance_increase_m
        if not healthy:
            self.approach_failed = True
            return False
        self.approach_distance = near
        if near <= self.options['stop_trigger_distance_m']:
            self.stop_latched.set()
            return False
        return not self.approach_failed

    def publish_result(self, result):
        if self.failure:
            self.fault_latched.set()
        with self.stop_lock:
            now = time.monotonic()
            stamp = result.get('frame_stamp', -math.inf)
            safe_now = self.motion_safe(result, stamp, now)
            if not safe_now:
                self.frame_valid_streak = 0
            acquired = safe_now and self.frame_valid_streak >= 5 and result.get('acquired', False)
            snapshot = self.stop_snapshot
            if safe_now:
                self.held_steering = max(-.10, min(.10, result['steering_cmd_rad']))
            if self.approach_started is None and self.test_state == 'DRIVING' and safe_now:
                stop = snapshot[1]
                near = stop.get('stop_near_edge_m')
                confidence = stop.get('stop_confidence', 0.)
                if (stop.get('stop_detected') and stop.get('stop_confirmed')
                        and near is not None and math.isfinite(near) and near <= 2.0
                        and math.isfinite(confidence) and confidence >= .80):
                    self.approach_started = now
                    self.approach_distance = near
            approaching = self.approach_check(result, now, snapshot)
            if self.stop_latched.is_set():
                self.test_state = 'STOP_LINE'
            elif self.fault_latched.is_set() or self.emergency_latched.is_set() or self.obstacle_stopped.is_set() or self.approach_failed:
                self.test_state = 'INVALID'
            elif approaching:
                self.test_state = 'STOP_APPROACH'
            elif acquired:
                self.test_state = 'DRIVING'
                self.started.set()
                if self.competition_tracking:
                    self.active_lane_pwm = min(self.options['drive_pwm'], result.get('requested_lane_pwm', 0))
            else:
                self.test_state = ('ACQUIRING' if safe_now
                                   or self.latest is None and not self.failure else 'INVALID')
            safe = {**result, 'valid': self.test_state in ('DRIVING', 'STOP_APPROACH')}
            if self.test_state == 'STOP_APPROACH':
                safe['steering_cmd_rad'] = self.held_steering
            super().publish_result(safe)

    def destroy_node(self):
        try:
            return super().destroy_node()
        finally:
            self.stop_worker.join(timeout=1.0)

    def diagnostic(self, result, fps):
        with self.stop_lock:
            snapshot = self.stop_snapshot
        stop_fresh = snapshot is not None and 0 <= time.monotonic()-snapshot[0] < .5
        result = {**result, **(snapshot[1] if stop_fresh else {})}
        return (f" stop_approach_active={self.test_state == 'STOP_APPROACH'}"
                f" stop_approach_pwm={self.options['stop_approach_pwm']}"
                f" held_steering_rad={self.held_steering:.4f}"
                f" stop_age_sec={None if snapshot is None else time.monotonic()-snapshot[0]}"
                f" approach_elapsed_sec={0 if self.approach_started is None else time.monotonic()-self.approach_started}"
                f" safe_streak={self.frame_valid_streak}"
                f" system_fault_latched={self.fault_latched.is_set()} stop_latched={self.stop_latched.is_set()}"
                f" control_source={result.get('control_source', 'NONE')}"
                f" tracking_source={result.get('tracking_source', 'LEGACY')}"
                f" tracking_confidence={result.get('tracking_confidence')}"
                f" motion_control_source={result.get('motion_control_source', 'NONE')}"
                f" motion_observed_min={result.get('observed_min')}"
                f" motion_used_lookahead={result.get('used_lookahead')}"
                f" motion_valid={result['valid']}"
                f" requested_pwm={result.get('requested_lane_pwm', 0)}"
                f" published_pwm={getattr(self, 'published_pwm', 0)}"
                f" pair_control_max_m={self.options['pair_control_max_m']:.2f}"
                f" single_side_control_max_m={self.options['single_side_control_max_m']:.2f}"
                f" allow_single_side_test={self.options['allow_single_side_test']}"
                f" stop_min_thickness_m={self.options['stop_min_thickness_m']:.2f}"
                f" STATE={self.test_state} stop_fresh={stop_fresh} lane_valid={result['valid']}"
                f" stop_detected={result.get('stop_detected', False)}"
                f" stop_confirmed={result.get('stop_confirmed', False)}"
                f" stop_trigger_distance_m={self.options['stop_trigger_distance_m']:.2f}"
                f" stop_triggered={self.stop_latched.is_set()}"
                f" stop_center_m={result.get('stop_center_m')}"
                f" stop_near_edge_m={result.get('stop_near_edge_m')}"
                f" stop_confidence={result.get('stop_confidence', 0.0)}"
                f" drive_pwm={self.requested_drive_pwm() if self.test_state in ('DRIVING', 'STOP_APPROACH') and self.publisher is not None else 0}")


def main(args=None, *, competition_tracking=False):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = None
    try:
        node = LaneStopTestNode(competition_tracking=True) if competition_tracking else LaneStopTestNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def competition_main(args=None):
    main(args, competition_tracking=True)
