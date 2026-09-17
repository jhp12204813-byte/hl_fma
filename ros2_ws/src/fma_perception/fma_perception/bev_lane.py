"""Opt-in calibrated BEV lane pipeline. No ROS or control side effects.

Image x points right and y down. Metric X points left and Y forward.
All geometry/metric calibration is supplied by the caller, never inferred.
"""
from dataclasses import dataclass, field
import math

import cv2
import numpy as np

from .lane_detection import LaneConfig, LaneEstimate, roi_bounds


@dataclass(frozen=True)
class BEVConfig:
    source_points: tuple = ()  # normalized TL, TR, BR, BL in UNDISTORTED image
    destination_points: tuple = ()  # normalized points in BEV
    width: int = 640
    height: int = 480
    image_width: int = 640
    image_height: int = 480
    undistort: bool = True
    lane_colors: str = 'yellow'
    windows: int = 12
    multi_height_seeds: bool = False
    directional_tracking: bool = False
    partial_support_ratio: float = .15
    margin_ratio: float = .08
    max_marking_width_ratio: float = .055
    min_marking_width_ratio: float = .006
    min_support_ratio: float = .6
    max_gap_ratio: float = .18
    residual_ratio: float = .012
    min_inlier_ratio: float = .7
    lane_width_min_ratio: float = .2
    lane_width_max_ratio: float = .85
    lane_width_variation_ratio: float = .2
    max_tangent_difference: float = .35
    temporal_alpha: float = .4
    max_temporal_shift_ratio: float = .08
    max_frame_gap_s: float = .3
    min_confirm_frames: int = 3
    geometry_calibrated: bool = False
    meters_per_pixel_x: float = 0.
    meters_per_pixel_y: float = 0.
    vehicle_x_px: float = -1.
    vehicle_y_px: float = -1.
    lookahead_ratio: float = .65

    def __post_init__(self):
        for name in ('width', 'height', 'image_width', 'image_height', 'windows', 'min_confirm_frames'):
            value = getattr(self, name)
            if type(value) is not int or value < (32 if 'width' in name or 'height' in name else 1):
                raise ValueError(f'Invalid {name}')
        if self.windows > self.height // 2 or max(self.width, self.height) > 4096:
            raise ValueError('Invalid BEV dimensions/windows')
        if self.lane_colors not in ('yellow', 'white', 'white_yellow'):
            raise ValueError('Invalid lane_colors')
        for name in ('undistort', 'geometry_calibrated', 'multi_height_seeds', 'directional_tracking'):
            if type(getattr(self, name)) is not bool:
                raise ValueError(f'{name} must be boolean')
        for name in ('partial_support_ratio', 'margin_ratio', 'max_marking_width_ratio', 'min_marking_width_ratio', 'min_support_ratio',
                     'max_gap_ratio', 'residual_ratio', 'min_inlier_ratio',
                     'lane_width_min_ratio', 'lane_width_max_ratio',
                     'lane_width_variation_ratio', 'temporal_alpha',
                     'max_temporal_shift_ratio', 'lookahead_ratio'):
            value = getattr(self, name)
            if not math.isfinite(value) or not 0 < value < 1:
                raise ValueError(f'{name} must be in (0, 1)')
        if self.lane_width_min_ratio >= self.lane_width_max_ratio:
            raise ValueError('Lane width bounds are reversed')
        if self.min_marking_width_ratio >= self.max_marking_width_ratio:
            raise ValueError('Marking width bounds are reversed')
        for name in ('max_frame_gap_s', 'max_tangent_difference'):
            if not math.isfinite(getattr(self, name)) or getattr(self, name) <= 0:
                raise ValueError(f'Invalid {name}')
        for name in ('meters_per_pixel_x', 'meters_per_pixel_y'):
            if not math.isfinite(getattr(self, name)) or getattr(self, name) < 0:
                raise ValueError(f'Invalid {name}')
        for name in ('vehicle_x_px', 'vehicle_y_px'):
            if not math.isfinite(getattr(self, name)) or getattr(self, name) < -1:
                raise ValueError(f'Invalid {name}')
        for points in (self.source_points, self.destination_points):
            if len(points):
                q = np.asarray(points, dtype=np.float32)
                if q.size != 8 or not np.isfinite(q).all() or np.any((q < 0) | (q > 1)):
                    raise ValueError('BEV points must be eight normalized coordinates')
                q = q.reshape(4, 2)
                if not cv2.isContourConvex(q) or cv2.contourArea(q, oriented=True) <= .001:
                    raise ValueError('BEV points must form a convex TL/TR/BR/BL polygon')
        if bool(len(self.source_points)) != bool(len(self.destination_points)):
            raise ValueError('Both BEV point sets are required')

    @property
    def metric_ready(self):
        return (self.geometry_calibrated and bool(len(self.source_points))
                and self.meters_per_pixel_x > 0 and self.meters_per_pixel_y > 0
                and 0 <= self.vehicle_x_px < self.width
                and self.lookahead_ratio * (self.height-1) < self.vehicle_y_px < self.height)


@dataclass
class BEVResult:
    estimate: LaneEstimate
    bev_debug: np.ndarray
    diagnostics: dict = field(default_factory=dict)


def narrow_runs(mask, max_width):
    """Remove wide paint at individual rows, never veto an entire component.

At a lane/stop-line intersection only the wide rows are omitted; lane pixels
above and below remain available to the window search and robust fit.
"""
    clean = np.zeros_like(mask)
    rows = []
    for y, row in enumerate(mask):
        edges = np.diff(np.r_[0, row > 0, 0].astype(np.int8))
        starts, ends = np.flatnonzero(edges == 1), np.flatnonzero(edges == -1)
        for a, b in zip(starts, ends):
            if 2 <= b-a <= max_width:
                clean[y, a:b] = 255
                rows.append((y, (a+b-1)/2, b-a))
    return clean, np.asarray(rows, dtype=float).reshape(-1, 3)


def robust_curve(ys, xs, config, partial=False, diagnostic=None):
    """RANSAC + trimmed quadratic fit, with final residual/coverage checks."""
    def reject(reason):
        if diagnostic is not None: diagnostic['reason'] = reason
        return None
    required = config.partial_support_ratio if partial else config.min_support_ratio
    if diagnostic is not None:
        diagnostic.update(pixels=len(ys), span_px=float(np.ptp(ys)) if len(ys) else 0,
                          required_span_px=config.height*required)
    if len(ys) < 12: return reject('pixel_insufficient')
    if np.ptp(ys) < config.height * required: return reject('vertical_span_insufficient')
    t = ys / (config.height-1)
    threshold = config.width * config.residual_ratio
    rng = np.random.default_rng(0)
    best = np.zeros(len(xs), bool)
    for _ in range(40):
        indices = rng.choice(len(xs), 3, replace=False)
        if np.ptp(t[indices]) < min(.25, required*.5) or len(np.unique(t[indices])) < 3:
            continue
        fit = np.polyfit(t[indices], xs[indices], 2)
        keep = np.abs(xs-np.polyval(fit, t)) <= threshold
        if keep.sum() > best.sum():
            best = keep
    if best.mean() < config.min_inlier_ratio:
        return reject('polynomial_fit_failed')
    for _ in range(2):
        fit = np.polyfit(t[best], xs[best], 2)
        best = np.abs(xs-np.polyval(fit, t)) <= threshold
        if best.sum() < 12:
            return reject('pixel_insufficient')
    fit = np.polyfit(t[best], xs[best], 2)
    residual = float(np.sqrt(np.mean((xs[best]-np.polyval(fit, t[best]))**2)))
    support_rows = np.unique(ys[best]).astype(int)
    coverage = len(support_rows)/config.height
    max_gap = np.diff(support_rows).max() if len(support_rows) > 1 else config.height
    if diagnostic is not None:
        diagnostic.update(rms_px=residual, coverage=coverage, max_gap_px=int(max_gap))
    if best.mean() < config.min_inlier_ratio: return reject('polynomial_fit_failed')
    if residual > threshold: return reject('residual_exceeded')
    if coverage < required: return reject('vertical_span_insufficient')
    if max_gap > config.height*config.max_gap_ratio: return reject('continuity_failed')
    if diagnostic is not None: diagnostic['reason'] = 'accepted'
    return dict(coefficients=fit.tolist(), coverage=coverage, rms_px=residual,
                inlier_ratio=float(best.mean()), y_min=int(support_rows[0]),
                y_max=int(support_rows[-1]), pixels=int(len(xs)))


def same_observed_curve(a, b, config):
    low = max(a['y_min'], b['y_min'])
    high = min(a['y_max'], b['y_max'])
    if high-low < config.height*config.partial_support_ratio*.5:
        return False
    t = np.linspace(low, high, 10)/(config.height-1)
    return np.max(np.abs(np.polyval(a['coefficients'], t)
                         - np.polyval(b['coefficients'], t))) < config.width*config.residual_ratio*2


def sliding_curves(mask, config, diagnostics=None):
    clean, runs = narrow_runs(mask, max(2, int(config.width*config.max_marking_width_ratio)))
    if not len(runs):
        if diagnostics is not None: diagnostics.append(dict(reason="no_seed", stage="run_filter"))
        return clean, [], []
    seeds = []
    margin = max(4, int(config.width*config.margin_ratio))
    starts = range(config.windows) if config.multi_height_seeds else [0]
    for start in starts:
        high = config.height*(config.windows-start)//config.windows
        low = max(0, high-config.height//config.windows) if config.multi_height_seeds else int(config.height*.7)
        histogram = np.sum(clean[low:high] > 0, axis=0).astype(float)
        threshold = max(3, (high-low)*.12) if config.multi_height_seeds else config.height*.04
        for _ in range(4 if config.multi_height_seeds else 8):
            seed = int(np.argmax(histogram))
            if histogram[seed] < threshold:
                break
            seeds.append((seed, start))
            histogram[max(0, seed-margin):seed+margin+1] = 0
    if not seeds and diagnostics is not None: diagnostics.append(dict(reason='no_seed',stage='histogram'))
    curves, windows = [], []
    for seed, start in seeds:
        diagnostic=dict(seed_x=int(seed),start_band=int(start))
        if diagnostics is not None: diagnostics.append(diagnostic)
        current, velocity = float(seed), 0.
        collected = []
        for index in range(start, config.windows):
            low = config.height*(config.windows-index-1)//config.windows
            high = config.height*(config.windows-index)//config.windows
            predicted = np.clip(current+velocity, 0, config.width-1)
            selected = runs[(runs[:, 0] >= low) & (runs[:, 0] < high)
                            & (np.abs(runs[:, 1]-predicted) <= margin)]
            windows.append([max(0, int(predicted-margin)), low,
                            min(config.width-1, int(predicted+margin)), high-1])
            # One closest run per row; do not join distinct nearby markings.
            chosen = []
            direction = None
            if config.directional_tracking and len(collected) >= 12:
                recent = np.asarray(collected)[-2*(config.height//config.windows):]
                if np.ptp(recent[:, 0]) >= 8:
                    direction = np.polyfit(recent[:, 0], recent[:, 1], 1)
                    # A local tangent guides search; it never becomes an output curve.
                    selected = runs[(runs[:, 0] >= low) & (runs[:, 0] < high)
                                    & (np.abs(runs[:, 1]-np.polyval(direction,runs[:, 0])) <= margin*.4)]
            for y in np.unique(selected[:, 0]):
                candidates = selected[selected[:, 0] == y]
                target = np.polyval(direction,y) if direction is not None else predicted
                chosen.append(candidates[np.argmin(np.abs(candidates[:, 1]-target))])
            if chosen:
                chosen = np.asarray(chosen)
                next_center = float(np.median(chosen[:, 1]))
                velocity = float(np.clip(next_center-current, -margin*.7, margin*.7))
                current = next_center
                collected.extend(chosen)
            else:
                current = float(predicted)
        if not collected:
            diagnostic['reason']='pixel_insufficient'
            continue
        ys, xs, widths = np.asarray(collected).T
        if np.median(widths) < config.width*config.min_marking_width_ratio:
            diagnostic['reason']='marking_width_failed'
            continue
        curve = robust_curve(ys, xs, config, partial=config.multi_height_seeds, diagnostic=diagnostic)
        if curve is not None:
            curve['partial'] = curve['coverage'] < config.min_support_ratio
            curve['marking_width_median_px'] = float(np.median(widths))
        if curve is not None:
            duplicates = [c for c in curves if same_observed_curve(curve, c, config)]
            if not duplicates:
                curves.append(curve)
            elif curve['coverage'] > max(c['coverage'] for c in duplicates):
                curves = [c for c in curves if c not in duplicates] + [curve]
    return clean, curves, windows


def choose_pair(curves, config, diagnostics=None):
    accepted = []
    if len(curves)<2 and diagnostics is not None: diagnostics.append(dict(reason="fewer_than_two_candidates",count=len(curves)))
    t = np.linspace(0, 1, 25)
    anchor = config.vehicle_x_px if config.vehicle_x_px >= 0 else (config.width-1)/2
    for a in curves:
        for b in curves:
            if a is b: continue
            check=dict(left=curves.index(a),right=curves.index(b))
            if diagnostics is not None: diagnostics.append(check)
            if a.get('partial', False) or b.get('partial', False):
                check['reason']='partial_candidate'
                continue
            left, right = np.asarray(a['coefficients']), np.asarray(b['coefficients'])
            widths = np.polyval(right-left, t)
            if not (np.all(widths >= config.width*config.lane_width_min_ratio)
                    and np.all(widths <= config.width*config.lane_width_max_ratio)):
                check['reason']='lane_width_failed'
                continue
            if np.ptp(widths)/np.median(widths) > config.lane_width_variation_ratio:
                check['reason']='lane_width_variation_failed'
                continue
            if np.max(np.abs(np.polyval(np.polyder(right-left), t)))/(config.height-1) > config.max_tangent_difference:
                check['reason']='heading_failed'
                continue
            # Classify by ordered curve relation at the vehicle end, not segment midpoints.
            if not np.polyval(left, 1) < anchor < np.polyval(right, 1):
                check['reason']='left_right_relation_failed'
                continue
            y = config.lookahead_ratio*(config.height-1)
            if not max(a['y_min'], b['y_min']) <= y <= min(a['y_max'], b['y_max']):
                check['reason']='lookahead_outside_observation'
                continue
            score = min(a['coverage']*a['inlier_ratio'], b['coverage']*b['inlier_ratio'])
            score *= max(0., 1-max(a['rms_px'], b['rms_px'])/(config.width*config.residual_ratio))
            check['reason']='eligible'
            accepted.append((score, left, right))
    if not accepted:
        return None
    accepted.sort(key=lambda item: item[0], reverse=True)
    # Multiple similarly supported boundaries are ambiguous (e.g. road paint).
    if len(accepted) > 1 and accepted[1][0] >= accepted[0][0]*.9:
        return None
    return accepted[0]


def metric_errors(center, config):
    t = config.lookahead_ratio
    y = t*(config.height-1)
    dx_dy = np.polyval(np.polyder(center), t)/(config.height-1)
    d2x_dy2 = 2*center[0]/(config.height-1)**2
    slope = dx_dy*config.meters_per_pixel_x/config.meters_per_pixel_y
    lateral = (config.vehicle_x_px-np.polyval(center, t))*config.meters_per_pixel_x
    heading = math.atan(slope)
    curvature = (-d2x_dy2*config.meters_per_pixel_x/config.meters_per_pixel_y**2
                 / (1+slope*slope)**1.5)
    return float(lateral), heading, float(curvature), (config.vehicle_y_px-y)*config.meters_per_pixel_y


class BEVLaneDetector:
    def __init__(self, config=BEVConfig(), image_config=LaneConfig()):
        self.config = config
        self.image_config = image_config
        self._camera_key = None
        self._maps = None
        self.reset()

    def reset(self):
        self.previous = None
        self.last_timestamp = None
        self.confirmations = 0

    def _rectify(self, image, camera):
        if not self.config.undistort:
            return image
        if camera is None:
            raise ValueError('missing_camera_info')
        h, w = image.shape[:2]
        k, d = np.asarray(camera['k'], float), np.asarray(camera['d'], float)
        if (camera['width'] != w or camera['height'] != h or k.size != 9
                or d.size not in (4, 5, 8, 12, 14) or not np.isfinite(k).all()
                or not np.isfinite(d).all() or camera['distortion_model'] not in ('plumb_bob', 'rational_polynomial')):
            raise ValueError('invalid_camera_info')
        k = k.reshape(3, 3)
        if k[0, 0] <= 0 or k[1, 1] <= 0 or not np.allclose(k[2], [0, 0, 1]):
            raise ValueError('invalid_camera_matrix')
        key = (w, h, tuple(k.ravel()), tuple(d))
        if key != self._camera_key:
            self.reset()
            self._maps = cv2.initUndistortRectifyMap(k, d, None, k, (w, h), cv2.CV_32FC1)
            self._camera_key = key
        return cv2.remap(image, *self._maps, cv2.INTER_LINEAR)

    def process(self, bgr, timestamp_ns, camera=None):
        if bgr is None or bgr.dtype != np.uint8 or bgr.ndim != 3 or bgr.shape[2] != 3:
            self.reset()
            raise ValueError('Expected uint8 BGR image')
        cfg = self.config
        empty = np.zeros_like(bgr)
        bev = np.zeros((cfg.height, cfg.width, 3), np.uint8)
        diag = dict(pipeline='bev', status='unconfigured_bev', visual_confidence=0.,
                    pixel_error=None, metric_ready=cfg.metric_ready, curves=[],
                    timestamp_ns=int(timestamp_ns), center_coefficients=None)
        estimate = LaneEstimate(False, 0., 0., 0., 0., bgr.copy(), False, empty)
        result = BEVResult(estimate, bev, diag)
        if bgr.shape[:2] != (cfg.image_height, cfg.image_width):
            self.reset()
            diag['status'] = 'calibration_image_size_mismatch'
            return self._finish(result)
        try:
            image = self._rectify(bgr, camera)
        except (ValueError, KeyError, TypeError, cv2.error) as error:
            self.reset()
            diag['status'] = str(error)
            return self._finish(result)
        estimate.debug = image.copy()
        if not len(cfg.source_points):
            self.reset()
            return self._finish(result)
        if self.last_timestamp is not None:
            dt = (timestamp_ns-self.last_timestamp)/1e9
            if dt <= 0 or dt > cfg.max_frame_gap_s:
                self.reset()
                diag['temporal_reset'] = 'timestamp_discontinuity'
        self.last_timestamp = timestamp_ns
        h, w = image.shape[:2]
        src = np.float32(cfg.source_points).reshape(4, 2)*[w-1, h-1]
        dst = np.float32(cfg.destination_points).reshape(4, 2)*[cfg.width-1, cfg.height-1]
        transform = cv2.getPerspectiveTransform(src.astype(np.float32), dst.astype(np.float32))
        cv2.polylines(estimate.debug, [np.rint(src).astype(np.int32)], True, (255, 150, 0), 2)
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        masks = {}
        for color in ('white', 'yellow'):
            masks[color] = cv2.inRange(hsv,
                tuple(getattr(self.image_config, f'{color}_{c}_min') for c in 'hsv'),
                tuple(getattr(self.image_config, f'{color}_{c}_max') for c in 'hsv'))
        mask = np.zeros((h, w), np.uint8)
        for color in cfg.lane_colors.split('_'):
            mask |= masks[color]
        x0, y0, x1, y1 = roi_bounds(image.shape, self.image_config)
        roi = np.zeros_like(mask)
        cv2.fillConvexPoly(roi, np.rint(src).astype(np.int32), 255)
        roi[:y0] = roi[y1+1:] = 0
        roi[:, :x0] = roi[:, x1+1:] = 0
        mask &= roi
        kernel = self.image_config.morph_kernel_size
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((kernel, kernel), np.uint8))
        estimate.debug_mask[masks['white'] & mask > 0] = (255, 255, 255)
        estimate.debug_mask[masks['yellow'] & mask > 0] = (0, 255, 255)
        warped = cv2.warpPerspective(mask, transform, (cfg.width, cfg.height), flags=cv2.INTER_NEAREST)
        clean, curves, windows = sliding_curves(warped, cfg)
        histogram = np.sum(clean[int(cfg.height*.7):] > 0, axis=0)
        diag.update(hsv_roi_pixels=int(np.count_nonzero(mask)),
                    bev_mask_pixels=int(np.count_nonzero(warped)),
                    narrow_mask_pixels=int(np.count_nonzero(clean)),
                    supported_row_ratio=float(np.mean(np.any(clean > 0, axis=1))),
                    bottom_histogram_peak=int(histogram.max()),
                    seed_threshold_rows=cfg.height*.04,
                    sliding_window_count=len(windows))
        result.bev_debug = cv2.cvtColor(clean, cv2.COLOR_GRAY2BGR)
        for x0, y0, x1, y1 in windows:
            cv2.rectangle(result.bev_debug, (x0, y0), (x1, y1), (80, 40, 0), 1)
        diag.update(curves=curves, wide_pixels_removed=int(np.count_nonzero(warped)-np.count_nonzero(clean)))
        pair = choose_pair(curves, cfg)
        for curve in curves:
            self._draw_curve(result.bev_debug, curve['coefficients'], (0, 180, 255), curve['y_min'], curve['y_max'])
            ys = np.linspace(curve['y_min'], curve['y_max'], 60)
            pts = np.c_[np.polyval(curve['coefficients'], ys/(cfg.height-1)), ys].astype(np.float32)
            projected = cv2.perspectiveTransform(pts.reshape(-1, 1, 2), np.linalg.inv(transform))
            if np.isfinite(projected).all() and np.abs(projected).max() < 1e6:
                cv2.polylines(estimate.debug, [np.rint(projected).astype(np.int32)], False, (0, 180, 255), 2)
        if pair is None:
            self.previous = None
            self.confirmations = 0
            diag['status'] = 'single_or_inconsistent_lanes' if curves else 'lane_lost'
            return self._finish(result)
        score, left, right = pair
        sample_t = np.linspace(0, 1, 25)
        if self.previous is not None:
            change = max(np.max(np.abs(np.polyval(now-old, sample_t)))
                         for now, old in zip((left, right), self.previous))
            if change > cfg.width*cfg.max_temporal_shift_ratio:
                self.previous = None
                self.confirmations = 0
                diag['status'] = 'temporal_jump_rejected'
                return self._finish(result)
            left, right = [cfg.temporal_alpha*new+(1-cfg.temporal_alpha)*old
                           for new, old in zip((left, right), self.previous)]
        self.previous = (left, right)
        self.confirmations += 1
        center = (left+right)/2
        estimate.visual_detected = True
        diag.update(visual_confidence=float(score), confirmation_frames=self.confirmations,
                    pixel_error=float((cfg.vehicle_x_px if cfg.vehicle_x_px >= 0 else (cfg.width-1)/2)
                                      - np.polyval(center, cfg.lookahead_ratio)),
                    center_coefficients=center.tolist(), status='visual_only_uncalibrated')
        self._draw_curve(result.bev_debug, center, (0, 255, 0))
        for curve, color in ((left, (0, 255, 255)), (right, (255, 0, 255)), (center, (0, 255, 0))):
            ts = np.linspace(0, 1, 100)
            pts = np.c_[np.polyval(curve, ts), ts*(cfg.height-1)].astype(np.float32)
            projected = cv2.perspectiveTransform(pts.reshape(-1, 1, 2), np.linalg.inv(transform))
            if np.isfinite(projected).all() and np.abs(projected).max() < 1e6:
                cv2.polylines(estimate.debug, [np.rint(projected).astype(np.int32)], False, color, 2)
        if cfg.metric_ready and self.confirmations >= cfg.min_confirm_frames and score > 0:
            lateral, heading, curvature, distance = metric_errors(center, cfg)
            if all(math.isfinite(v) for v in (lateral, heading, curvature)):
                estimate.detected = True
                estimate.lateral_error_m, estimate.heading_error_rad = lateral, heading
                estimate.curvature, estimate.confidence = curvature, float(score)
                diag.update(status='detected', lookahead_forward_m=distance)
        elif cfg.metric_ready:
            diag['status'] = 'confirming'
        return self._finish(result)

    def _draw_curve(self, image, curve, color, y0=0, y1=None):
        y1 = self.config.height-1 if y1 is None else y1
        ys = np.linspace(y0, y1, 100)
        xs = np.polyval(curve, ys/(self.config.height-1))
        pts = np.c_[np.clip(xs, -self.config.width, 2*self.config.width), ys].astype(np.int32)
        cv2.polylines(image, [pts], False, color, 2)

    @staticmethod
    def _finish(result):
        cv2.putText(result.estimate.debug, 'BEV: '+result.diagnostics['status'],
                    (10, 22), cv2.FONT_HERSHEY_SIMPLEX, .5, (0, 200, 255), 1)
        pixel = result.diagnostics.get('pixel_error')
        text = 'pixel_error=N/A' if pixel is None else f'pixel_error={pixel:+.1f}px (+left)'
        cv2.putText(result.estimate.debug, text, (10, 44), 0, .5, (0, 200, 255), 1)
        if result.estimate.detected:
            estimate = result.estimate
            text = (f'ey={estimate.lateral_error_m:+.3f}m heading={estimate.heading_error_rad:+.3f}rad '
                    f'k={estimate.curvature:+.3f}/m conf={estimate.confidence:.2f}')
        else:
            text = 'Metric output disabled; visual confidence=' + f'{result.diagnostics["visual_confidence"]:.2f}'
        cv2.putText(result.estimate.debug, text, (10, 66), 0, .45, (0, 200, 255), 1)
        return result
