"""Competition perception adapter; reuse existing controller and distance windows."""
import math
from fma_control.lane_follow_node import calculate_command
from fma_vehicle.vehicle_controller_node import steering_angle_to_adc


def calculate_competition_command(tracking, tracker, config, options):
    lane = tracker.to_lane_result(tracking)  # Actual boundaries for STOP detection.

    def evaluate(selected):
        command = calculate_command(tracker.to_lane_result(selected), tracker.bev, config,
            field_stop_test=True, allow_single_side_test=True,
            pair_control_max_m=options['pair_control_max_m'],
            single_side_control_max_m=options['single_side_control_max_m'])
        if not selected['valid'] or selected['confidence'] < tracker.cfg.valid_confidence:
            command.update(valid=False, reason=selected.get('reject_reason') or 'low_confidence')
        return command

    selected = tracking
    result = evaluate(selected)
    # Only a valid, unambiguous selected pair may supply fallback boundaries.
    # Geometry/fusion rejection must never be bypassed by selecting another side.
    if tracking['valid'] and tracking['source'] == 'PAIR_TRACK' and result['reason'] == 'outside_control_window':
        usable = []
        for candidate in tracker.single_control_candidates(tracking):
            command = evaluate(candidate)
            if command['valid']:
                usable.append((candidate, command))
        usable.sort(key=lambda item: item[0]['confidence'], reverse=True)
        if usable:
            selected, result = usable[0]
    source = selected['source'] if selected['valid'] else 'NONE'
    if selected['confidence_grade'] == 'HIGH':
        pwm = options['drive_pwm'] if source == 'PAIR_TRACK' else options['single_drive_pwm']
    else:
        pwm = options['degraded_pwm']
    pwm = min(options['drive_pwm'], pwm) if result['valid'] else 0

    # At high speed the near part of a single boundary may temporarily
    # disappear. Keep steering from the reliable far boundary, but slow down
    # until near-field observation returns.
    if (result['valid']
            and source in ('SINGLE_LEFT_TRACK', 'SINGLE_RIGHT_TRACK')
            and result.get('observed_min') is not None
            and result['observed_min'] > 3.60):
        pwm = min(pwm, 160)
    result.update(control_source=source, motion_control_source=source,
                  motion_observed_min=result['observed_min'], motion_used_lookahead=result['used_lookahead'],
                  motion_valid=result['valid'], requested_pwm=pwm,
                  tracking_source=tracking['source'], tracking_confidence=tracking['confidence'],
                  confidence_grade=selected['confidence_grade'], requested_lane_pwm=pwm,
                  virtual_center=selected['virtual_center'], boundary_heading=selected['boundary_heading'],
                  boundary_curvature=selected['boundary_curvature'], lateral_offset_target=selected['lateral_offset_target'],
                  lane_scoring={k: tracking.get(k) for k in ('candidate_count', 'selected_pair_score', 'second_pair_score',
                      'selected_lane_width_m', 'expected_lane_width_m', 'width_delta_m', 'heading_diff_deg',
                      'pair_overlap_m', 'reject_reason')})
    return result, lane


class PairReturnBlend:
    """Short transition only on single→pair; never grants validity or changes PWM."""
    def __init__(self, duration_sec=.3):
        if not math.isfinite(duration_sec) or duration_sec <= 0:
            raise ValueError('pair_return_blend_sec must be positive')
        self.duration = duration_sec
        self.previous_source = None
        self.previous_steering = 0.
        self.started = None
        self.start_steering = 0.

    def apply(self, result, timestamp):
        if not result['valid']:
            self.previous_source = None
            self.started = None
            return result
        source = result.get('motion_control_source', result['tracking_source'])
        if source == 'PAIR_TRACK' and (self.previous_source or '').startswith('SINGLE_'):
            self.started = timestamp
            self.start_steering = self.previous_steering
        if source != 'PAIR_TRACK':
            self.started = None
        if self.started is not None:
            alpha = min(1., max(0., (timestamp-self.started)/self.duration))
            result = {**result, 'steering_cmd_rad': (1-alpha)*self.start_steering+alpha*result['steering_cmd_rad']}
            result['expected_adc'] = steering_angle_to_adc(result['steering_cmd_rad'])
            if alpha >= 1.:
                self.started = None
        self.previous_source = source
        self.previous_steering = result['steering_cmd_rad']
        return result
