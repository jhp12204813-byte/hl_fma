"""Image-space lane estimates; physical outputs require explicit scale calibration."""
from dataclasses import dataclass
import math

import cv2
import numpy as np


@dataclass(frozen=True)
class LaneConfig:
    roi_top_ratio: float = 0.6
    roi_bottom_ratio: float = 1.0
    roi_left_ratio: float = 0.0
    roi_right_ratio: float = 1.0
    lookahead_ratio: float = 0.65
    lane_width_m: float = 0.0
    single_lane_width_px: float = 0.0
    meters_per_pixel_y: float = 0.0

    white_h_min: int = 0
    white_h_max: int = 179
    white_s_min: int = 0
    white_s_max: int = 65
    white_v_min: int = 170
    white_v_max: int = 255
    yellow_h_min: int = 15
    yellow_h_max: int = 40
    yellow_s_min: int = 75
    yellow_s_max: int = 255
    yellow_v_min: int = 90
    yellow_v_max: int = 255
    morph_kernel_size: int = 3
    min_candidate_area_px: int = 0
    min_line_length_px: int = 0
    max_line_gap_px: int = 0
    min_abs_slope: float = 1.0

    def __post_init__(self):
        for color in ('white', 'yellow'):
            for channel, limit in (('h', 179), ('s', 255), ('v', 255)):
                low = getattr(self, f'{color}_{channel}_min')
                high = getattr(self, f'{color}_{channel}_max')
                if (type(low) is not int or type(high) is not int
                        or not 0 <= low <= high <= limit):
                    raise ValueError(f'{color} {channel}: integer 0 <= min <= max <= {limit}')
        if (type(self.morph_kernel_size) is not int or
                not 1 <= self.morph_kernel_size <= 31 or self.morph_kernel_size % 2 == 0):
            raise ValueError('morph_kernel_size must be odd in 1..31')
        for name in ('min_candidate_area_px', 'min_line_length_px', 'max_line_gap_px'):
            value = getattr(self, name)
            if type(value) is not int or value < 0:
                raise ValueError(f'{name} must be a nonnegative integer')
        if self.min_abs_slope < 0:
            raise ValueError('min_abs_slope must be nonnegative')
        values = vars(self)
        if not all(math.isfinite(v) for v in values.values()):
            raise ValueError('Lane parameters must be finite')
        if not 0 <= self.roi_top_ratio < self.roi_bottom_ratio <= 1:
            raise ValueError('ROI must satisfy 0 <= top < bottom <= 1')
        if not 0 <= self.roi_left_ratio < self.roi_right_ratio <= 1:
            raise ValueError('ROI must satisfy 0 <= left < right <= 1')
        if not self.roi_top_ratio <= self.lookahead_ratio < self.roi_bottom_ratio:
            raise ValueError('Lookahead must lie inside ROI and before bottom')
        if 0 < self.lane_width_m < .05:
            raise ValueError('Measured lane_width_m must be at least 0.05 m')
        if min(self.lane_width_m, self.meters_per_pixel_y, self.single_lane_width_px) < 0:
            raise ValueError('Calibration scales must be nonnegative')


@dataclass
class LaneEstimate:
    detected: bool
    lateral_error_m: float
    heading_error_rad: float
    curvature: float
    confidence: float
    debug: np.ndarray
    visual_detected: bool
    debug_mask: np.ndarray


def roi_bounds(shape, config):
    """Normalized rectangular ROI; kept independent from extraction and ROS."""
    h, w = shape[:2]
    bounds = (int(w * config.roi_left_ratio), int(h * config.roi_top_ratio),
              min(w - 1, int(w * config.roi_right_ratio) - 1),
              min(h - 1, int(h * config.roi_bottom_ratio) - 1))
    if bounds[2] - bounds[0] < 8 or bounds[3] - bounds[1] < 8:
        raise ValueError('ROI too small')
    return bounds


def detect_lane(bgr, config=LaneConfig(), *, yellow_only=False):
    """Positive lateral/heading means left. Zero curvature means unavailable."""
    if bgr is None or bgr.dtype != np.uint8 or bgr.ndim != 3 or bgr.shape[2] != 3:
        raise ValueError('Expected uint8 BGR image')
    h, w = bgr.shape[:2]
    if min(h, w) < 16:
        raise ValueError('Image too small')
    x_min, top, x_max, bottom = roi_bounds(bgr.shape, config)
    lookahead = int(h * config.lookahead_ratio)
    if lookahead >= bottom:
        raise ValueError('Lookahead must precede bottom row')
    debug = bgr.copy()
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    def color_mask(color):
        return cv2.inRange(
            hsv, tuple(getattr(config, f'{color}_{c}_min') for c in 'hsv'),
            tuple(getattr(config, f'{color}_{c}_max') for c in 'hsv'))
    white = np.zeros((h, w), dtype=np.uint8) if yellow_only else color_mask('white')
    yellow = color_mask('yellow')
    mask = cv2.bitwise_or(white, yellow)
    mask[:top] = 0
    mask[bottom + 1:] = 0
    mask[:, :x_min] = 0
    mask[:, x_max + 1:] = 0
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((config.morph_kernel_size, config.morph_kernel_size), np.uint8))
    if config.min_candidate_area_px > 0:
        count, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
        keep = np.zeros(count, dtype=np.uint8)
        keep[1:] = (stats[1:, cv2.CC_STAT_AREA] >= config.min_candidate_area_px) * 255
        mask = keep[labels]
    debug_mask = np.zeros_like(bgr)
    debug_mask[(white > 0) & (mask > 0)] = (255, 255, 255)
    debug_mask[(yellow > 0) & (mask > 0)] = (0, 255, 255)
    lines = cv2.HoughLinesP(
        cv2.Canny(mask, 50, 150), 1, np.pi / 180, threshold=max(12, h // 30),
        minLineLength=config.min_line_length_px or max(10, (bottom - top) // 5),
        maxLineGap=config.max_line_gap_px or max(5, h // 40))
    points = [[], []]
    if lines is not None:
        for x1, y1, x2, y2 in lines[:, 0]:
            if abs(int(y2) - int(y1)) < max(5, config.min_abs_slope * abs(int(x2) - int(x1))):
                continue
            side = int((x1 + x2) / 2 >= w / 2)
            points[side].extend([(y1, x1), (y2, x2)])
    fits = []
    for group in points:
        if len(group) < 4:
            fits.append(None)
            continue
        ys, xs = np.asarray(group, dtype=float).T
        if np.ptp(ys) < (bottom - top) * .4:
            fits.append(None)
            continue
        fit = np.polyfit(ys, xs, 1)
        residual = np.abs(xs - np.polyval(fit, ys))
        keep = residual <= max(4., w * .03)
        if keep.sum() < 4 or np.ptp(ys[keep]) < (bottom - top) * .4:
            fits.append(None)
        else:
            fits.append(np.polyfit(ys[keep], xs[keep], 1))
    # Show candidates even if only one side is available.
    debug[mask > 0] = (0, 150, 150)
    for fit, color in zip(fits, [(0, 255, 255), (255, 0, 255)]):
        if fit is not None:
            cv2.line(debug, (int(np.polyval(fit, top)), top),
                     (int(np.polyval(fit, bottom)), bottom), color, 3)
    visual = False
    lateral = heading = confidence = 0.
    center = None
    observed_width = 0.
    support = 0.
    calibrated = config.lane_width_m > 0 and config.meters_per_pixel_y > 0
    if all(f is not None for f in fits):
        left, right = fits
        widths = np.polyval(right - left, [top, bottom])
        visual = bool(np.all(widths > w * .1) and np.all(widths < w * .95))
        if visual:
            center = (left + right) / 2
            observed_width = float(np.polyval(right - left, lookahead))
            support = .5 + .5 * min(
                np.ptp(np.asarray(group)[:, 0]) / max(1, bottom - top)
                for group in points)
    elif calibrated and w * .1 < config.single_lane_width_px < w * .95:
        # Optional measured apparent width, never a fabricated opposite lane.
        for side, fit in enumerate(fits):
            if fit is not None:
                center = fit.copy()
                center[1] += (1 if side == 0 else -1) * config.single_lane_width_px / 2
                observed_width = config.single_lane_width_px
                support = .35
                visual = bool(x_min <= np.polyval(center, lookahead) <= x_max)
    if visual and center is not None:
        cv2.line(debug, (int(np.polyval(center, top)), top),
                 (int(np.polyval(center, bottom)), bottom), (0, 255, 0), 3)
        cv2.circle(debug, (int(np.polyval(center, lookahead)), lookahead),
                   6, (0, 0, 255), -1)
        if calibrated:
            scale_x = config.lane_width_m / observed_width
            center_near = float(np.polyval(center, bottom))
            center_ahead = float(np.polyval(center, lookahead))
            lateral = (w / 2 - center_ahead) * scale_x
            heading = math.atan2((center_near - center_ahead) * scale_x,
                                 (bottom - lookahead) * config.meters_per_pixel_y)
            confidence = support
    if not all(math.isfinite(v) for v in (lateral, heading, confidence)):
        lateral = heading = confidence = 0.
    detected = bool(visual and confidence > 0)
    cv2.rectangle(debug, (x_min, top), (x_max, bottom), (255, 150, 0), 2)
    cv2.line(debug, (w // 2, top), (w // 2, bottom), (255, 255, 0), 1)
    cv2.putText(debug, f'detected={detected} confidence={confidence:.2f} visual={visual}',
                (10, 25), cv2.FONT_HERSHEY_SIMPLEX, .55, (0, 255, 0), 1)
    cv2.putText(debug, f'lateral={lateral:+.3f}m heading={heading:+.3f}rad',
                (10, 75), cv2.FONT_HERSHEY_SIMPLEX, .55, (0, 255, 0), 1)
    # Debug-only image coordinates at the same lookahead as the metric output.
    # Keep unavailable centers explicit; never infer a width without calibration.
    image_center_px = w / 2
    lane_center_px = float(np.polyval(center, lookahead)) if visual and center is not None else None
    center_text = 'N/A' if lane_center_px is None else f'{lane_center_px:.1f}'
    error_text = 'N/A' if lane_center_px is None else f'{image_center_px - lane_center_px:+.1f}'
    cv2.putText(debug, f'y={lookahead} lane_center_px={center_text} image_center_px={image_center_px:.1f}',
                (10, 100), cv2.FONT_HERSHEY_SIMPLEX, .5, (0, 255, 0), 1)
    cv2.putText(debug, f'pixel_error={error_text}px (+left) L=yellow R=magenta',
                (10, 125), cv2.FONT_HERSHEY_SIMPLEX, .5, (0, 255, 0), 1)
    if not (config.lane_width_m > 0 and config.meters_per_pixel_y > 0):
        cv2.putText(debug, 'UNCALIBRATED: metric output disabled', (10, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, .55, (0, 200, 255), 1)
    return LaneEstimate(detected, float(lateral), float(heading), 0.,
                        float(np.clip(confidence, 0, 1)), debug, visual, debug_mask)
