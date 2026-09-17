"""Output occurrence rates are NOT ground-truth recall or false-positive rates."""
from collections import Counter
import numpy as np


def summarize(rows):
    summary = {}
    for session in sorted({r['session'] for r in rows}):
        group = [r for r in rows if r['session']==session]
        valid = [r for r in group if 'lanes' in r]
        count = len(valid)
        rate = lambda n: n/count if count else None
        lanes = [c for r in valid for c in r['lanes']]
        left = sum(any(c['side']=='left' for c in r['lanes']) for r in valid)
        right = sum(any(c['side']=='right' for c in r['lanes']) for r in valid)
        both = sum({'left','right'} <= {c['side'] for c in r['lanes']} for r in valid)
        single = sum(len(r['lanes'])==1 for r in valid)
        pairs = sum(r.get('pair_failure_category')=='accepted' for r in valid)
        centers = sum(r.get('center') is not None for r in valid)
        survived = opportunities = 0
        previous = None
        for row in group:
            if 'lanes' not in row:
                previous = None
                continue
            if previous and row.get('block_start')==previous.get('block_start') and row['index']==previous['index']+1 and not row.get('temporal_reset'):
                old={c['track_id'] for c in previous['lanes']}
                new={c['track_id'] for c in row['lanes'] if c['survived']}
                opportunities+=len(old);survived+=len(old&new)
            previous=row
        summary[session]=dict(requested_frames=len(group),evaluated_frames=count,
            output_occurrence_rates=dict(left=rate(left),right=rate(right),single_lane=rate(single),
                both_sides=rate(both),pair_acceptance=rate(pairs),center_generation=rate(centers)),
            counts=dict(left=left,right=right,single_lane=single,both_sides=both,pair_acceptance=pairs,center_generation=centers),
            ground_truth_metrics=dict(left_lane_detection_rate=None,right_lane_detection_rate=None,false_lane_rate=None,
                reason='No reviewed per-frame ground-truth annotations; output occurrence is not accuracy.'),
            average_track_length_m=float(np.mean([c['track_length_m'] for c in lanes])) if lanes else None,
            average_fit_residual_m=float(np.mean([c['residual_m'] for c in lanes])) if lanes else None,
            frame_to_frame_track_survival=survived/opportunities if opportunities else None,
            survival_counts=dict(survived=survived,opportunities=opportunities),
            candidate_rejections=dict(Counter(d['reason'] for r in valid for d in r['candidate_rejections'])),
            pair_rejections=dict(Counter(d['reason'] for r in valid for d in r['pair_rejections'])),
            pair_failure_categories=dict(Counter(r['pair_failure_category'] for r in valid)),
            baseline_curve_counts=dict(Counter(r['baseline']['curve_count'] for r in valid)),
            baseline_candidate_rejections=dict(Counter(d['reason'] for r in valid for d in r['baseline']['candidate_rejections'])),
            baseline_pair_rejections=dict(Counter(d['reason'] for r in valid for d in r['baseline']['pair_rejections'])))
    return summary
