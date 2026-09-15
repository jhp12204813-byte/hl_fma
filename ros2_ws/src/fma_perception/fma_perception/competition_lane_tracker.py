"""Metric lane selection and persistent single-boundary tracking. No hardware/ROS.

All geometry uses vehicle x-right/y-forward meters. Learned normal offsets have
no clock expiry; current-frame quality and identity determine usability.
"""
from dataclasses import dataclass, asdict
import math
import cv2
import numpy as np


@dataclass(frozen=True)
class MetricLaneConfig:
    nominal_lane_width_m: float = 3.5
    min_lane_width_m: float = 2.0
    max_lane_width_m: float = 6.0
    ema_alpha: float = .15
    min_component_length_m: float = .25
    max_thickness_m: float = .30
    min_aspect_ratio: float = 3.0
    min_length_m: float = .9
    max_gap_m: float = 1.2
    merge_lateral_m: float = .22
    residual_limit_m: float = .06
    min_points: int = 50
    max_heading_rad: float = .75
    max_curvature: float = .5
    lateral_jump_m: float = .65
    heading_jump_rad: float = .35
    curvature_jump: float = .2
    min_pair_overlap_m: float = .8
    valid_confidence: float = .55
    high_confidence: float = .80
    pair_score_gap: float = .06
    single_score_gap: float = .05
    max_candidates: int = 24

    def __post_init__(self):
        for name, value in asdict(self).items():
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f'{name} must be finite and positive')
        if not self.min_lane_width_m <= self.nominal_lane_width_m <= self.max_lane_width_m:
            raise ValueError('nominal width must lie inside global plausibility bounds')
        if not 0 < self.ema_alpha <= 1 or not 0 < self.valid_confidence < self.high_confidence <= 1:
            raise ValueError('invalid EMA/confidence parameters')
        if type(self.min_points) is not int or type(self.max_candidates) is not int:
            raise ValueError('point/candidate counts must be integers')


def heading_curvature(coeff, y):
    slope = np.polyval(np.polyder(coeff), y)
    return np.arctan(slope), 2 * coeff[0] / (1 + slope*slope)**1.5


def normal_offset_curve(coeff, y_min, y_max, signed_offset_m):
    """Positive offset is boundary's right-hand local normal, not a fixed x shift."""
    ys = np.linspace(y_min, y_max, 100)
    xs = np.polyval(coeff, ys)
    slope = np.polyval(np.polyder(coeff), ys)
    scale = np.sqrt(1 + slope*slope)
    center_x = xs + signed_offset_m / scale
    center_y = ys - signed_offset_m * slope / scale
    # Parallel curves can fold when offset*curvature reaches one. Never fit through a fold.
    if not np.all(np.isfinite(center_x)) or not np.all(np.diff(center_y) > 0):
        return None
    coefficients = np.polyfit(center_y, center_x, 2)
    residual = float(np.max(np.abs(np.polyval(coefficients, center_y) - center_x)))
    return {'coefficients': coefficients, 'y_min_m': float(center_y[0]),
            'y_max_m': float(center_y[-1]), 'points_m': np.c_[center_x, center_y],
            'boundary_y_m': ys, 'approximation_error_m': residual}


class RelativeHeadingConsistency:
    """Optional quality-checked F9P/path heading in x-right/y-forward radians.

    Caller converts path minus vehicle heading into the BEV convention. Missing
    or stale optional measurements never become GPS control or a mandatory input.
    """
    def __init__(self, max_age_sec=.5, max_error_rad=.4):
        if not all(math.isfinite(v) and v > 0 for v in (max_age_sec, max_error_rad)):
            raise ValueError('heading consistency limits must be positive')
        self.max_age_sec, self.max_error_rad = max_age_sec, max_error_rad
        self.sample = None

    def update(self, relative_heading_rad, timestamp, valid=True):
        self.sample = (relative_heading_rad, timestamp) if valid and all(
            math.isfinite(v) for v in (relative_heading_rad, timestamp)) else None

    def __call__(self, result):
        current, heading = result.get('frame_timestamp'), result.get('boundary_heading')
        if self.sample is None or current is None or heading is None:
            return {'valid': True, 'available': False}
        expected, stamp = self.sample
        if not 0 <= current-stamp < self.max_age_sec:
            return {'valid': True, 'available': False}
        difference = math.atan2(math.sin(heading-expected), math.cos(heading-expected))
        return {'valid': abs(difference) <= self.max_error_rad, 'available': True,
                'reason': 'secondary_heading_mismatch'}


class CompetitionLaneTracker:
    def __init__(self, bev, cfg=None, consistency_check=None):
        self.bev = bev
        self.cfg = cfg or MetricLaneConfig()
        # Optional caller-supplied freshness/quality-aware GPS/path validator.
        # No GNSS topics are subscribed here, and absence is not an error.
        self.consistency_check = consistency_check
        self.reset()

    def reset(self):
        self.expected_lane_width_m = None
        self.left_offset_m = self.right_offset_m = None
        self.previous = {}
        self.last_timestamp = None
        self.source = 'INVALID'

    def _fit(self, points):
        x, y = points[:, 0], points[:, 1]
        rows = np.round(y / self.bev.resolution_m).astype(int)
        unique = np.unique(rows)
        yy = unique * self.bev.resolution_m
        xx = np.array([np.median(x[rows == row]) for row in unique])
        if len(unique) < 3:
            return None
        # Seeded metric RANSAC on row centers prevents thick paint dominating a fit.
        rng = np.random.default_rng(7)
        best = np.zeros(len(xx), bool)
        for _ in range(40):
            ids = rng.choice(len(xx), 3, replace=False)
            if np.ptp(yy[ids]) < .2:
                continue
            coeff = np.polyfit(yy[ids], xx[ids], 2)
            keep = np.abs(xx - np.polyval(coeff, yy)) <= self.cfg.residual_limit_m
            if keep.sum() > best.sum():
                best = keep
        if best.sum() < 3:
            return None
        coeff = np.polyfit(yy[best], xx[best], 2)
        residual = float(np.sqrt(np.mean((xx[best]-np.polyval(coeff, yy[best]))**2)))
        return coeff, yy[best], residual, float(best.mean())

    def _candidates(self, mask):
        count, labels, stats, _ = cv2.connectedComponentsWithStats((mask > 0).astype(np.uint8), 8)
        components, rejected = [], []
        for label in range(1, count):
            yy, xx = np.nonzero(labels == label)
            if len(xx) < 3:
                continue
            points = np.c_[(xx-self.bev.width/2)*self.bev.resolution_m,
                           self.bev.far_m-yy*self.bev.resolution_m]
            span = np.ptp(points[:, 1]) + self.bev.resolution_m
            row_counts = np.unique(yy, return_counts=True)[1]
            thickness = float(np.median(row_counts) * self.bev.resolution_m)
            # Reject short, thick paint BEFORE grouping disconnected fragments.
            if (span < self.cfg.min_component_length_m or thickness > self.cfg.max_thickness_m
                    or span/max(thickness, self.bev.resolution_m) < self.cfg.min_aspect_ratio):
                rejected.append({'id': f'C{label}', 'points_m': points,
                                 'length_m': float(span), 'thickness_m': thickness,
                                 'reject_reason': 'chunky_component', 'score': 0.})
                continue
            fitted = self._fit(points)
            if fitted is not None:
                coeff, ys, residual, coverage = fitted
                components.append({'ids': [label], 'points_m': points, 'coefficients': coeff,
                                   'y_min_m': float(ys.min()), 'y_max_m': float(ys.max()),
                                   'thickness_m': thickness})
        # Join thin aligned dashed components; chunky curb blocks never enter here.
        groups = []
        for component in sorted(components, key=lambda c: c['y_min_m']):
            matches = []
            for index, group in enumerate(groups):
                gap = max(0., component['y_min_m']-group['y_max_m'])
                y = (component['y_min_m'] + min(component['y_max_m'], group['y_max_m'])) / 2
                lateral = abs(np.polyval(component['coefficients'], y)-np.polyval(group['coefficients'], y))
                heading = abs(float(heading_curvature(component['coefficients'], y)[0]
                                    - heading_curvature(group['coefficients'], y)[0]))
                if gap <= self.cfg.max_gap_m and lateral <= self.cfg.merge_lateral_m and heading <= .25:
                    matches.append((lateral, index))
            if matches:
                group = groups[min(matches)[1]]
                group['points_m'] = np.vstack([group['points_m'], component['points_m']])
                group['ids'] += component['ids']
                fitted = self._fit(group['points_m'])
                if fitted:
                    group['coefficients'] = fitted[0]
                group['y_max_m'] = max(group['y_max_m'], component['y_max_m'])
            else:
                groups.append(component.copy())
        candidates = []
        for group in groups:
            fitted = self._fit(group['points_m'])
            if fitted is None:
                continue
            coeff, ys, residual, inlier_ratio = fitted
            y0, y1 = float(ys.min()), float(ys.max())
            sample_y = np.linspace(y0, y1, 40)
            heading, curvature = heading_curvature(coeff, sample_y)
            length = float(np.sum(np.hypot(np.diff(np.polyval(coeff, sample_y)), np.diff(sample_y))))
            gaps = np.diff(np.sort(ys))
            max_gap = float(gaps.max()) if len(gaps) else 0.
            continuity = min(1., len(ys)*self.bev.resolution_m/max(y1-y0, self.bev.resolution_m))
            reference_y = np.clip(2., y0, y1)
            x = float(np.polyval(coeff, reference_y))
            side = 'left' if x < 0 else 'right'
            h, k = heading_curvature(coeff, reference_y)
            thickness = group['thickness_m'] * math.cos(float(h))
            score = (.25*min(1., length/2.) + .15*min(1., .15/max(thickness, .01))
                     + .15*max(0., 1-residual/self.cfg.residual_limit_m)
                     + .15*min(1., len(group['points_m'])/100)
                     + .15*continuity + .15*max(0., 1-abs(float(h))/self.cfg.max_heading_rad))
            score -= .15 * (1-inlier_ratio)
            reason = None
            if length < self.cfg.min_length_m or len(group['points_m']) < self.cfg.min_points:
                reason = 'short_boundary'
            elif max_gap > self.cfg.max_gap_m or continuity < .25:
                reason = 'discontinuous_boundary'
            elif np.max(np.abs(heading)) > self.cfg.max_heading_rad or np.max(np.abs(curvature)) > self.cfg.max_curvature:
                reason = 'heading_curvature_implausible'
            previous = self.previous.get(side)
            lateral_delta = heading_delta = curvature_delta = 0.
            if previous is not None:
                lo, hi = max(y0, previous['y_min_m']), min(y1, previous['y_max_m'])
                if lo > hi:
                    reason = 'no_continuity_overlap'
                else:
                    shared = np.linspace(lo, hi, 20)
                    lateral_delta = float(np.max(np.abs(np.polyval(coeff, shared)-np.polyval(previous['coefficients'], shared))))
                    ph, pk = heading_curvature(previous['coefficients'], shared)
                    ch, ck = heading_curvature(coeff, shared)
                    heading_delta = float(np.max(np.abs(ph-ch)))
                    curvature_delta = float(np.max(np.abs(pk-ck)))
                    if (lateral_delta > self.cfg.lateral_jump_m or heading_delta > self.cfg.heading_jump_rad
                            or curvature_delta > self.cfg.curvature_jump):
                        reason = 'boundary_identity_jump'
                    score -= .15*min(1., lateral_delta/self.cfg.lateral_jump_m)
                    score -= .10*min(1., heading_delta/self.cfg.heading_jump_rad)
                    score -= .10*min(1., curvature_delta/self.cfg.curvature_jump)
            if reason is None and score < self.cfg.valid_confidence:
                reason = 'low_boundary_confidence'
            candidates.append({**group, 'id': side[0].upper()+':'+','.join(map(str, group['ids'])),
                'side': side, 'coefficients': coeff, 'y_min_m': y0, 'y_max_m': y1,
                'length_m': length, 'thickness_m': thickness, 'heading': float(h),
                'curvature': float(k), 'residual_m': residual, 'point_count': len(group['points_m']),
                'continuity': continuity, 'max_gap_m': max_gap, 'inlier_ratio': inlier_ratio,
                'lateral_delta_m': lateral_delta, 'score': float(np.clip(score, 0, 1)), 'reject_reason': reason})
        return candidates, rejected

    def _pairs(self, candidates):
        pairs = []
        for left in candidates:
            if left['side'] != 'left' or left['reject_reason']:
                continue
            for right in candidates:
                if right['side'] != 'right' or right['reject_reason']:
                    continue
                lo, hi = max(left['y_min_m'], right['y_min_m']), min(left['y_max_m'], right['y_max_m'])
                if hi-lo < self.cfg.min_pair_overlap_m:
                    continue
                y = np.linspace(lo, hi, 40)
                lx, rx = np.polyval(left['coefficients'], y), np.polyval(right['coefficients'], y)
                lh, lk = heading_curvature(left['coefficients'], y)
                rh, rk = heading_curvature(right['coefficients'], y)
                # Normal projected separation avoids widening the learned offset on turns.
                normal_widths = (rx-lx)*np.cos((lh+rh)/2)
                width = float(np.median(normal_widths))
                if (np.any(rx <= lx) or not self.cfg.min_lane_width_m <= width <= self.cfg.max_lane_width_m):
                    continue
                width_consistency = float(np.std(normal_widths))
                heading_diff = float(np.max(np.abs(lh-rh)))
                curvature_diff = float(np.max(np.abs(lk-rk)))
                delta = 0. if self.expected_lane_width_m is None else width-self.expected_lane_width_m
                score = ((left['score']+right['score'])/2
                         - .15*min(1., width_consistency/.4)
                         - .15*min(1., heading_diff/.35)
                         - .10*min(1., curvature_diff/.15)
                         - .20*min(1., abs(delta)/1.0))
                if heading_diff > .5 or curvature_diff > .3 or width_consistency > .6:
                    continue
                pairs.append({'left': left, 'right': right, 'score': float(score), 'width': width,
                              'left_offset': float(np.median((rx-lx)*np.cos(lh)/2)),
                              'right_offset': float(np.median((rx-lx)*np.cos(rh)/2)),
                              'delta': delta, 'heading_diff': heading_diff, 'curvature_diff': curvature_diff,
                              'width_consistency_m': width_consistency, 'overlap': hi-lo,
                              'y_min_m': lo, 'y_max_m': hi})
        return sorted(pairs, key=lambda p: p['score'], reverse=True)

    def process(self, mask, timestamp=None):
        if mask.shape != (self.bev.height, self.bev.width):
            raise ValueError('mask shape must match calibrated BEV')
        if timestamp is not None and (not math.isfinite(timestamp) or self.last_timestamp is not None and timestamp <= self.last_timestamp):
            return self._invalid('non_new_frame')
        if timestamp is not None:
            self.last_timestamp = timestamp
        candidates, rejected = self._candidates(mask)
        result = self._invalid('no_reliable_boundary')
        result['frame_timestamp'] = timestamp
        result.update(candidates=candidates+rejected, candidate_count=len(candidates)+len(rejected))
        if len(candidates) > self.cfg.max_candidates:
            result['reject_reason'] = 'candidate_overload'
            return result
        pairs = self._pairs(candidates)
        if pairs:
            top = pairs[0]
            result.update(selected_pair_score=top['score'], second_pair_score=pairs[1]['score'] if len(pairs)>1 else None,
                          selected_lane_width_m=top['width'], width_delta_m=top['delta'],
                          heading_diff_deg=math.degrees(top['heading_diff']), pair_overlap_m=top['overlap'])
            if len(pairs)>1 and top['score']-pairs[1]['score'] < self.cfg.pair_score_gap:
                result['reject_reason'] = 'ambiguous_pair'
                return result
            if top['score'] >= self.cfg.valid_confidence:
                left, right = top['left'], top['right']
                coeff = (left['coefficients']+right['coefficients'])/2
                center = {'coefficients': coeff, 'y_min_m': top['y_min_m'], 'y_max_m': top['y_max_m']}
                result.update(valid=True, source='PAIR_TRACK', virtual_center=center,
                              confidence=min(1., top['score']), confidence_grade='HIGH' if top['score'] >= self.cfg.high_confidence else 'DEGRADED',
                              left=left, right=right, reject_reason=None)
                if not self._external_valid(result):
                    return result
                # Only a HIGH-confidence, unambiguous current pair teaches geometry.
                if top['score'] >= self.cfg.high_confidence:
                    width = top['width']
                    a = self.cfg.ema_alpha
                    self.expected_lane_width_m = width if self.expected_lane_width_m is None else (1-a)*self.expected_lane_width_m+a*width
                    self.left_offset_m = top['left_offset'] if self.left_offset_m is None else (1-a)*self.left_offset_m+a*top['left_offset']
                    self.right_offset_m = top['right_offset'] if self.right_offset_m is None else (1-a)*self.right_offset_m+a*top['right_offset']
                self.previous.update(left=left, right=right)
                self.source = result['source']
                result['expected_lane_width_m'] = self.expected_lane_width_m
                result.update(left_to_center_offset_m=self.left_offset_m, right_to_center_offset_m=self.right_offset_m)
                return result
        usable = sorted((c for c in candidates if not c['reject_reason']), key=lambda c: c['score'], reverse=True)
        if not usable:
            return result
        # Two reliable sides without a plausible pair must not silently become single tracking.
        if len({c['side'] for c in usable}) > 1:
            result['reject_reason'] = 'inconsistent_pair'
            return result
        if len(usable)>1 and usable[0]['score']-usable[1]['score'] < self.cfg.single_score_gap:
            result['reject_reason'] = 'ambiguous_boundary'
            return result
        boundary = usable[0]
        side = boundary['side']
        learned = self.left_offset_m if side == 'left' else self.right_offset_m
        offset = learned if learned is not None else self.cfg.nominal_lane_width_m/2
        center = normal_offset_curve(boundary['coefficients'], boundary['y_min_m'], boundary['y_max_m'],
                                     offset if side == 'left' else -offset)
        if center is None or center['approximation_error_m'] > self.cfg.residual_limit_m:
            result['reject_reason'] = 'invalid_normal_offset_curve'
            return result
        result.update(valid=True, source=f'SINGLE_{side.upper()}_TRACK', virtual_center=center,
                      boundary_heading=boundary['heading'], boundary_curvature=boundary['curvature'],
                      lateral_offset_target=offset, confidence=boundary['score'],
                      confidence_grade='HIGH' if boundary['score'] >= self.cfg.high_confidence else 'DEGRADED',
                      offset_source='LEARNED' if learned is not None else 'NOMINAL',
                      reject_reason=None, **{side: boundary})
        if not self._external_valid(result):
            return result
        previous = self.previous.get(side)
        other = 'right' if side == 'left' else 'left'
        if previous is not None and other in self.previous:
            # Move the unseen boundary's identity reference with the observed
            # vehicle-relative curve motion. This does NOT learn width/offsets.
            self.previous[other] = {**self.previous[other], 'coefficients':
                self.previous[other]['coefficients'] + boundary['coefficients'] - previous['coefficients']}
        self.previous[side] = boundary
        self.source = result['source']
        return result

    def single_control_candidates(self, tracking):
        """Read-only control fallback from the accepted pair's current boundaries.

        No re-learning, identity change, observation extrapolation or age refresh.
        """
        if not tracking['valid'] or tracking['source'] != 'PAIR_TRACK':
            return []
        results = []
        for side in ('left', 'right'):
            boundary = tracking.get(side)
            if not boundary or boundary['reject_reason'] or boundary['score'] < self.cfg.valid_confidence:
                continue
            learned = self.left_offset_m if side == 'left' else self.right_offset_m
            offset = learned if learned is not None else self.cfg.nominal_lane_width_m / 2
            center = normal_offset_curve(boundary['coefficients'], boundary['y_min_m'], boundary['y_max_m'],
                                         offset if side == 'left' else -offset)
            if center is None or center['approximation_error_m'] > self.cfg.residual_limit_m:
                continue
            candidate = {**self._invalid(None), 'frame_timestamp': tracking.get('frame_timestamp'),
                         'valid': True, 'source': f'SINGLE_{side.upper()}_TRACK',
                         side: boundary, 'virtual_center': center, 'confidence': boundary['score'],
                         'confidence_grade': 'HIGH' if boundary['score'] >= self.cfg.high_confidence else 'DEGRADED',
                         'boundary_heading': boundary['heading'], 'boundary_curvature': boundary['curvature'],
                         'lateral_offset_target': offset, 'offset_source': 'LEARNED' if learned is not None else 'NOMINAL'}
            if self._external_valid(candidate):
                results.append(candidate)
        return results

    def _external_valid(self, result):
        if self.consistency_check is None:
            return True
        verdict = self.consistency_check(result)
        if not verdict.get('valid', False):
            result.update(valid=False, source='INVALID', reject_reason=verdict.get('reason', 'secondary_consistency_failed'))
            return False
        return True

    def _invalid(self, reason):
        return {'valid': False, 'source': 'INVALID', 'virtual_center': None,
                'boundary_heading': None, 'boundary_curvature': None, 'lateral_offset_target': None,
                'confidence': 0., 'confidence_grade': 'INVALID', 'offset_source': None,
                'candidate_count': 0, 'candidates': [], 'selected_pair_score': None,
                'second_pair_score': None, 'selected_lane_width_m': None,
                'expected_lane_width_m': self.expected_lane_width_m, 'width_delta_m': None,
                'left_to_center_offset_m': self.left_offset_m, 'right_to_center_offset_m': self.right_offset_m,
                'heading_diff_deg': None, 'pair_overlap_m': None, 'reject_reason': reason}

    def to_bev_fit(self, metric_fit):
        """Convert x(y) meters to u(v) pixels without changing BEV calibration."""
        if metric_fit is None:
            return None
        r, far = self.bev.resolution_m, self.bev.far_m
        a, b, c = metric_fit['coefficients']
        return {'coefficients': np.array([a*r, -(2*a*far+b), (a*far*far+b*far+c)/r+self.bev.width/2]),
                'y_min': (far-metric_fit['y_max_m'])/r, 'y_max': (far-metric_fit['y_min_m'])/r}

    def to_lane_result(self, tracking):
        """Legacy detector input: actual boundaries only, never a fabricated second lane."""
        lane = {side: self.to_bev_fit(tracking.get(side)) for side in ('left', 'right')}
        lane.update(state='NONE', center=None, temporary_center=None, center_source='NONE')
        center = self.to_bev_fit(tracking['virtual_center'])
        if tracking['valid'] and tracking['source'] == 'PAIR_TRACK':
            lane.update(state='BOTH_VISUAL', pair_state='PAIR_VALID', pair_quality={'valid': True},
                        center=center, center_source='PAIR_VALID')
        elif tracking['valid'] and tracking['source'].startswith('SINGLE_'):
            side = 'LEFT' if tracking['source'] == 'SINGLE_LEFT_TRACK' else 'RIGHT'
            lane.update(state=side+'_ONLY', center_source=side+'_ONLY_OFFSET', temporary_center=center)
        return lane
