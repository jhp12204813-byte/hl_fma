"""Visibility-aware ground candidates. Outputs are unlabelled visual hypotheses."""
import warnings
import numpy as np
from .bev_lane import narrow_runs


def geometry(coeff, t, height, resolution):
    slope = np.polyval(np.polyder(coeff), t)/(height-1)
    return np.arctan(slope), -2*coeff[0]/((height-1)**2*resolution)/(1+slope*slope)**1.5


def fit_candidate(points, visible, cfg, log):
    h, w = visible.shape
    if len(points) < 12:
        log['reason'] = 'pixel_insufficient'
        return None
    y, x, widths = np.asarray(points).T
    log.update(pixels=len(y), span_m=float(np.ptp(y)*cfg.resolution_m))
    if not cfg.min_marking_m <= np.median(widths)*cfg.resolution_m <= cfg.max_marking_m:
        log['reason'] = 'marking_width_failed'
        return None
    t = y/(h-1)
    threshold = cfg.fit_residual_m/cfg.resolution_m
    best = np.zeros(len(y), bool)
    rng = np.random.default_rng(7)
    for _ in range(60):
        indices = rng.choice(len(y), 3, replace=False)
        if np.ptp(y[indices]) < max(6, np.ptp(y)*.3):
            continue
        with warnings.catch_warnings():
            warnings.simplefilter('error', np.RankWarning)
            try:
                coeff = np.polyfit(t[indices], x[indices], 2)
            except (np.RankWarning, np.linalg.LinAlgError):
                continue
        keep = np.abs(x-np.polyval(coeff, t)) <= threshold
        if keep.sum() > best.sum():
            best = keep
    if best.sum() < 12 or best.mean() < .7:
        log['reason'] = 'polynomial_fit_failed'
        return None
    for _ in range(3):
        coeff = np.polyfit(t[best], x[best], 2)
        best = np.abs(x-np.polyval(coeff, t)) <= threshold
        if best.sum() < 12:
            log['reason'] = 'pixel_insufficient'
            return None
    if best.mean() < .7:
        log['reason'] = 'polynomial_fit_failed'
        return None
    residual = float(np.sqrt(np.mean((x[best]-np.polyval(coeff, t[best]))**2))*cfg.resolution_m)
    log['residual_m'] = residual
    if residual > cfg.fit_residual_m:
        log['reason'] = 'residual_exceeded'
        return None
    rows = np.unique(y[best]).astype(int)
    grid = np.arange(h)
    # Visibility along the fitted path, not the entire rectangular BEV height.
    projected = np.rint(np.polyval(coeff, grid/(h-1))).astype(int)
    valid = (projected >= 0)&(projected < w)
    valid &= visible[grid, np.clip(projected, 0, w-1)]
    # Never use a remote re-entry into FOV to inflate/deflate support.
    components = np.split(np.flatnonzero(valid), np.flatnonzero(np.diff(np.flatnonzero(valid)) > 1)+1)
    intersecting = [c for c in components if len(c) and np.intersect1d(c, rows).size]
    available = max(intersecting, key=lambda c: np.intersect1d(c, rows).size) if intersecting else np.array([], int)
    support = len(np.intersect1d(rows, available))/max(1, len(available))
    log.update(visible_rows=len(available), support_ratio=float(support))
    min_support = cfg.support_ratio
    if log.get('source') == 'previous_polynomial':
        min_support = min(min_support, 0.50)

    log['required_support_ratio'] = float(min_support)

    if (
        len(available) < 12
        or support < min_support
        or np.ptp(rows) < min_support * len(available)
    ):
        log['reason'] = 'vertical_span_insufficient'
        return None
    window = max(1, h//11)
    occupied = np.unique(rows//window)
    consecutive = max((len(c) for c in np.split(occupied, np.flatnonzero(np.diff(occupied)>1)+1)), default=0)
    needed = min(3, len(np.unique(available//window)))
    log.update(consecutive_windows=consecutive, required_windows=needed)
    if consecutive < max(2, needed):
        log['reason'] = 'continuous_windows_insufficient'
        return None
    max_row_gap = int(np.max(np.diff(rows))) if len(rows) > 1 else 0
    gap_ratio = 0.25 if log.get('source') == 'previous_polynomial' else 0.15
    allowed_gap = max(3, int(len(available) * gap_ratio))

    log['max_row_gap'] = max_row_gap
    log['allowed_row_gap'] = allowed_gap
    log['continuity_gap_ratio'] = gap_ratio

    if max_row_gap > allowed_gap:
        log['reason'] = 'continuity_failed'
        return None
    heading, curvature = geometry(coeff, np.linspace(rows[0], rows[-1], 20)/(h-1), h, cfg.resolution_m)
    if np.max(np.abs(heading)) > cfg.max_lane_heading_rad:
        log['reason'] = 'heading_failed'
        return None
    if np.max(np.abs(curvature)) > cfg.max_lane_curvature_inv_m:
        log['reason'] = 'curvature_failed'
        return None
    log['reason'] = 'spatial_accepted'
    return dict(coefficients=coeff.tolist(), y_min=int(rows[0]), y_max=int(rows[-1]),
                coverage=float(len(rows)/h), visible_support=float(support),
                rms_px=residual/cfg.resolution_m, residual_m=residual,
                inlier_ratio=float(best.mean()), track_length_m=float(np.ptp(rows)*cfg.resolution_m),
                heading_rad=float(np.median(heading)), curvature_inv_m=float(np.median(curvature)),
                confidence=float(support*best.mean()*max(0, 1-residual/cfg.fit_residual_m)))


def extract(mask, visible, cfg, previous):
    """Multi-band seeds, bidirectional side/middle entry and previous-fit search."""
    h, w = mask.shape
    clean, runs = narrow_runs(mask, max(2, int(cfg.max_marking_m/cfg.resolution_m)))
    logs, seeds, curves = [], [], []
    if not len(runs):
        return curves, [dict(reason='no_seed', stage='run_filter')]
    margin = cfg.window_margin_m/cfg.resolution_m
    for band in range(11):
        low, high = h*band//11, h*(band+1)//11
        histogram = np.sum(clean[low:high]>0, axis=0)
        for _ in range(4):
            x = int(np.argmax(histogram))
            if histogram[x] < max(3, (high-low)*.12):
                break
            seeds.append((x, (low+high)//2, None, 'multi_band'))
            histogram[max(0, int(x-margin)):min(w, int(x+margin)+1)] = 0
    for old in previous:
        y = (old['y_min']+old['y_max'])//2
        seeds.insert(0, (float(np.polyval(old['coefficients'], y/(h-1))), y,
                         old['coefficients'], 'previous_polynomial'))
    if not seeds:
        logs.append(dict(reason='no_seed', stage='histogram'))
    by_row = {y: runs[runs[:,0]==y] for y in range(h)}
    for seed_x, seed_y, prior, source in seeds:
        log = dict(candidate_id=len(logs), seed_x=seed_x, seed_y=seed_y, source=source)
        logs.append(log)
        selected = {}
        for direction in (-1, 1):
            recent = []
            current = seed_x
            for y in range(seed_y, h if direction>0 else -1, direction):
                if prior is not None:
                    target = np.polyval(prior, y/(h-1))
                elif len(recent) >= 8:
                    tail = np.asarray(recent[-30:])
                    target = np.polyval(np.polyfit(tail[:,0], tail[:,1], 1), y)
                else:
                    target = current
                row = by_row[y]
                choices = row[np.abs(row[:,1]-target)<=margin*(.5 if len(recent)>=8 else 1)]
                if len(choices):
                    point = choices[np.argmin(np.abs(choices[:,1]-target))]
                    selected[y] = point
                    recent.append(point)
                    current = point[1]
        curve = fit_candidate([selected[k] for k in sorted(selected)], visible, cfg, log)
        if curve is None:
            continue
        duplicate = False
        for other in curves:
            low, high = max(curve['y_min'], other['y_min']), min(curve['y_max'], other['y_max'])
            if high-low >= 6:
                t = np.linspace(low, high, 20)/(h-1)
                if np.max(np.abs(np.polyval(curve['coefficients'], t)-np.polyval(other['coefficients'], t)))*cfg.resolution_m < cfg.fit_residual_m*2:
                    duplicate = True
                    log['reason'] = 'duplicate_candidate'
                    break
        if not duplicate:
            curve['candidate_id'] = log['candidate_id']
            curves.append(curve)
    return curves, logs


class CandidateTracks:
    def __init__(self):
        self.previous = []
        self.stamp = None
        self.next_id = 1

    def reset(self):
        self.previous = []
        self.stamp = None

    def process(self, mask, visible, cfg, stamp):
        h, w = mask.shape
        reset = self.stamp is not None and (stamp <= self.stamp or stamp - self.stamp > 300_000_000)
        if reset:
            self.reset()

        old_tracks = list(self.previous)
        curves, logs = extract(mask, visible, cfg, old_tracks)
        used = set()
        max_missed = int(getattr(cfg, 'max_missed_frames', 2))

        for curve in curves:
            log = logs[curve['candidate_id']]
            matches = []

            for old in old_tracks:
                if old['track_id'] in used:
                    continue

                low = max(old['y_min'], curve['y_min'])
                high = min(old['y_max'], curve['y_max'])
                if high - low < 6:
                    continue

                t = np.linspace(low, high, 20) / (h - 1)
                distance = float(
                    np.max(
                        np.abs(
                            np.polyval(curve['coefficients'], t)
                            - np.polyval(old['coefficients'], t)
                        )
                    ) * cfg.resolution_m
                )

                if distance <= cfg.max_center_jump_m:
                    ah, ac = geometry(curve['coefficients'], t, h, cfg.resolution_m)
                    bh, bc = geometry(old['coefficients'], t, h, cfg.resolution_m)
                    matches.append((
                        distance,
                        float(np.max(np.abs(ah - bh))),
                        float(np.max(np.abs(ac - bc))),
                        old,
                    ))

            if matches:
                distance, dh, dc, old = min(matches, key=lambda x: x[0])
                log.update(
                    track_distance_m=distance,
                    heading_jump_rad=dh,
                    curvature_jump_inv_m=dc,
                    recovered_after_missed=int(old.get('missed', 0)),
                )
                used.add(old['track_id'])

                if dh > cfg.max_heading_jump_rad or dc > cfg.max_curvature_jump_inv_m:
                    log['reason'] = 'temporal_failed'
                    curve['rejected'] = True
                    continue

                curve.update(
                    track_id=old['track_id'],
                    age=old['age'] + 1,
                    missed=0,
                    survived=True,
                    observed_this_frame=True,
                )
            else:
                curve.update(
                    track_id=self.next_id,
                    age=1,
                    missed=0,
                    survived=False,
                    observed_this_frame=True,
                )
                self.next_id += 1

            t = np.linspace(curve['y_min'], curve['y_max'], 20) / (h - 1)
            x = np.polyval(curve['coefficients'], t) - w / 2

            side = 'left' if np.all(x < 0) else 'right' if np.all(x > 0) else 'unknown'

            if matches and side != old.get('side', side):
                log['reason'] = 'temporal_side_ambiguous'
                side = 'unknown'

            curve['side'] = side
            curve['side_basis'] = (
                'observed_curve_and_heading; temporal_id; camera_forward_axis'
            )
            curve['confirmed'] = curve['age'] >= cfg.min_track_frames
            curve['confidence'] *= min(1, curve['age'] / cfg.min_track_frames)

            if log['reason'] == 'spatial_accepted':
                log['reason'] = 'accepted' if curve['confirmed'] else 'temporal_pending'

        current = [c for c in curves if not c.get('rejected')]

        retained = []
        for old in old_tracks:
            if old['track_id'] in used:
                continue

            missed = int(old.get('missed', 0)) + 1
            if missed <= max_missed:
                stale = dict(old)
                stale['missed'] = missed
                stale['observed_this_frame'] = False
                retained.append(stale)

        # Retained tracks are used only for matching/seeding the next frame.
        # They are NOT returned as current detections, so stale lanes cannot form a pair.
        self.previous = current + retained
        self.stamp = stamp

        return current, logs, reset


def pair_tracks(tracks, shape, cfg):
    h, w = shape
    eligible = [t for t in tracks if t['confirmed']]
    logs, accepted = [], []
    if len(eligible)<2:
        return None, [dict(reason='fewer_than_two_confirmed', count=len(eligible))]
    for i, a in enumerate(eligible):
        for b in eligible[i+1:]:
            log=dict(track_ids=[a['track_id'], b['track_id']])
            logs.append(log)
            low, high=max(a['y_min'], b['y_min']),min(a['y_max'], b['y_max'])
            if high-low<12:
                log['reason']='pair_overlap_insufficient';continue
            t=np.linspace(low,high,25)/(h-1)
            left,right=np.asarray(a['coefficients']),np.asarray(b['coefficients'])
            if np.median(np.polyval(right-left,t))<0:
                left,right=right,left
            lx,rx=np.polyval(left,t),np.polyval(right,t)
            if not np.all((lx<w/2)&(rx>w/2)):
                log['reason']='left_right_relation_failed';continue
            widths=(rx-lx)*cfg.resolution_m
            log.update(width_min_m=float(widths.min()),width_max_m=float(widths.max()))
            if widths.min()<cfg.pair_width_min_m or widths.max()>cfg.pair_width_max_m:
                log['reason']='lane_width_failed';continue
            if np.ptp(widths)/np.median(widths)>.2:
                log['reason']='lane_width_variation_failed';continue
            ah,ac=geometry(left,t,h,cfg.resolution_m);bh,bc=geometry(right,t,h,cfg.resolution_m)
            if np.max(np.abs(ah-bh))>cfg.max_pair_heading_rad:
                log['reason']='heading_failed';continue
            if np.max(np.abs(ac-bc))>cfg.max_pair_curvature_inv_m:
                log['reason']='curvature_failed';continue
            log['reason']='eligible'
            accepted.append((min(a['confidence'],b['confidence']),left,right,low,high))
    accepted.sort(key=lambda x:x[0],reverse=True)
    if len(accepted)>1 and accepted[1][0]>=accepted[0][0]*.9:
        logs.append(dict(reason='pair_ambiguous'));return None,logs
    if not accepted:return None,logs
    logs.append(dict(reason='pair_accepted'))
    return accepted[0],logs
