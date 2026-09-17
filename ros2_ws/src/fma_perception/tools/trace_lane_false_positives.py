"""Offline forensic trace. Imports unchanged detector; never starts ROS nodes.

Segment veto experiments reuse baseline Hough lines, then reproduce its endpoint
fit exactly. They do not claim equivalence to masking pixels and rerunning Hough.
"""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import sys

import cv2
import numpy as np

from fma_perception.lane_detection import LaneConfig, detect_lane


CONFIG = LaneConfig()


def capture_pipeline(image):
    captured = {}
    previous = sys.getprofile()

    def profile(frame, event, arg):
        if frame.f_code is detect_lane.__code__ and event == 'return':
            captured.update(frame.f_locals)

    sys.setprofile(profile)
    try:
        result = detect_lane(image, CONFIG)
    finally:
        sys.setprofile(previous)
    return result, captured


def components(mask):
    count, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
    records = []
    for label in range(1, count):
        x, y, w, h, area = map(int, stats[label])
        region = labels[y:y + h, x:x + w] == label
        present = region.any(axis=1)
        first = region.argmax(axis=1)
        last = w - 1 - region[:, ::-1].argmax(axis=1)
        spans = (last - first + 1)[present]
        records.append(dict(
            id=label, x=x, y=y, width=w, height=h, area=area,
            bbox_width_over_height=w / h, fill=area / (w * h),
            area_over_height_squared=area / h ** 2,
            row_width_p50=float(np.percentile(spans, 50)),
            row_width_p90=float(np.percentile(spans, 90)),
            row_width_p90_over_height=float(np.percentile(spans, 90)) / h,
            occupied_rows=int(present.sum()),
            # An 8-connected component necessarily occupies every intermediate row.
            row_occupancy_within_bbox=float(present.mean()),
        ))
    return labels, records


def associate(line, labels):
    """Nearest foreground in radius 2 at each rasterized segment sample.

No semantic labels: dominant component and fraction are only geometric evidence.
A 2px tolerance accounts for Canny edges just outside binary components.
"""
    x1, y1, x2, y2 = map(int, line)
    n = max(abs(x2 - x1), abs(y2 - y1)) + 1
    xs = np.rint(np.linspace(x1, x2, n)).astype(int)
    ys = np.rint(np.linspace(y1, y2, n)).astype(int)
    found = np.zeros(n, np.int32)
    offsets = sorted(((dy, dx) for dy in range(-2, 3) for dx in range(-2, 3)),
                     key=lambda p: p[0] ** 2 + p[1] ** 2)
    for dy, dx in offsets:
        values = labels[np.clip(ys + dy, 0, labels.shape[0] - 1),
                        np.clip(xs + dx, 0, labels.shape[1] - 1)]
        take = (found == 0) & (values > 0)
        found[take] = values[take]
    ids, counts = np.unique(found[found > 0], return_counts=True)
    votes = {int(i): int(c) for i, c in zip(ids, counts)}
    dominant = max(votes, key=votes.get) if votes else 0
    return dominant, votes.get(dominant, 0) / n, votes


def segment_records(lines, labels, width):
    records = []
    for index, line in enumerate(lines):
        x1, y1, x2, y2 = map(int, line)
        dx, dy = x2 - x1, y2 - y1
        accepted = abs(dy) >= max(5, CONFIG.min_abs_slope * abs(dx))
        dominant, fraction, votes = associate(line, labels)
        records.append(dict(
            id=index, x1=x1, y1=y1, x2=x2, y2=y2,
            angle_deg=((math.degrees(math.atan2(dy, dx)) + 90) % 180) - 90,
            slope_dy_dx=None if dx == 0 else dy / dx,
            length=math.hypot(dx, dy), y_span=abs(dy),
            midpoint_x=(x1 + x2) / 2,
            side='right' if (x1 + x2) / 2 >= width / 2 else 'left',
            slope_accepted=accepted, component=dominant,
            component_fraction=fraction, component_votes=votes,
        ))
    return records


def fit_segments(segments, shape, top, bottom):
    """Exact default endpoint aggregation and two-pass regression, with evidence."""
    output = {}
    width = shape[1]
    for side in ('left', 'right'):
        selected = [s for s in segments if s['slope_accepted'] and s['side'] == side]
        endpoints = [(s, end, s[f'y{end}'], s[f'x{end}'])
                     for s in selected for end in (1, 2)]
        data = dict(fit=None, initial_fit=None, input_segment_ids=[s['id'] for s in selected],
                    final_segment_ids=[], endpoints=[], initial_y_span=0., final_y_span=0.)
        if endpoints:
            ys = np.array([p[2] for p in endpoints], dtype=float)
            xs = np.array([p[3] for p in endpoints], dtype=float)
            data['initial_y_span'] = float(np.ptp(ys))
            if len(endpoints) >= 4 and np.ptp(ys) >= (bottom - top) * .4:
                initial = np.polyfit(ys, xs, 1)
                residual = np.abs(xs - np.polyval(initial, ys))
                keep = residual <= max(4., width * .03)
                data['initial_fit'] = initial.tolist()
                data['final_y_span'] = float(np.ptp(ys[keep])) if keep.any() else 0.
                if keep.sum() >= 4 and data['final_y_span'] >= (bottom - top) * .4:
                    final = np.polyfit(ys[keep], xs[keep], 1)
                    data['fit'] = final.tolist()
                    data['final_segment_ids'] = sorted({endpoints[i][0]['id']
                                                        for i in np.flatnonzero(keep)})
                for i, (seg, end, y, x) in enumerate(endpoints):
                    data['endpoints'].append(dict(
                        segment=seg['id'], endpoint=end, y=y, x=x,
                        residual_initial=float(residual[i]), residual_gate_pass=bool(keep[i]),
                        used_final=bool(keep[i] and data['fit'] is not None),
                        residual_final=None if data['fit'] is None else
                        float(abs(x - np.polyval(data['fit'], y))),
                    ))
        output[side] = data
    left, right = output['left']['fit'], output['right']['fit']
    output['visual'] = False
    output['width_top_bottom'] = None
    output['perspective_pass'] = None
    if left is not None and right is not None:
        widths = np.polyval(np.array(right) - left, [top, bottom])
        output['width_top_bottom'] = widths.tolist()
        output['visual'] = bool(np.all(widths > width * .1) and np.all(widths < width * .95))
        output['perspective_pass'] = bool(left[0] <= 0 <= right[0] and widths[1] >= widths[0])
    return output


def experiments(segments, records, shape, top, bottom):
    by_id = {c['id']: c for c in records}
    roi_h = bottom - top + 1
    # Diagnostic thresholds only: none are ROS parameters or detector defaults.
    predicates = {
        'baseline': lambda c: True,
        'linked_only': lambda c: True,
        'bbox': lambda c: c['bbox_width_over_height'] <= .8,
        'area': lambda c: c['area_over_height_squared'] <= .30,
        'row_width_strict': lambda c: c['row_width_p90_over_height'] <= .30,
        'row_width': lambda c: c['row_width_p90_over_height'] <= .50,
        'y_support': lambda c: c['height'] / roi_h >= .60,
        'row_width_y': lambda c: (c['row_width_p90_over_height'] <= .50
                                 and c['height'] / roi_h >= .60),
    }
    result = {}
    for name, predicate in predicates.items():
        chosen = segments if name == 'baseline' else [
            s for s in segments if s['component'] in by_id
            and s['component_fraction'] >= .70 and predicate(by_id[s['component']])]
        fit = fit_segments(chosen, shape, top, bottom)
        fit['eligible_segment_ids'] = [s['id'] for s in chosen if s['slope_accepted']]
        result[name] = fit
    return result


def save_csv(path, records):
    if not records:
        return
    with path.open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(records[0]))
        writer.writeheader()
        for record in records:
            writer.writerow({k: json.dumps(v) if isinstance(v, (dict, list)) else v
                             for k, v in record.items()})


def analyze(image, output=None):
    result, local = capture_pipeline(image)
    labels, comps = components(local['mask'])
    lines = [] if local['lines'] is None else local['lines'][:, 0]
    segments = segment_records(lines, labels, image.shape[1])
    trials = experiments(segments, comps, image.shape, local['top'], local['bottom'])
    baseline = trials['baseline']
    for side, original in zip(('left', 'right'), local['fits']):
        fit = baseline[side]['fit']
        assert (fit is None) == (original is None)
        if fit is not None:
            np.testing.assert_allclose(fit, original, atol=1e-9)
    assert baseline['visual'] == result.visual_detected
    data = dict(shape=list(image.shape), roi=[local['top'], local['bottom']],
                components=comps, segments=segments, trials=trials)
    if output is not None:
        output.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(output / 'binary_mask.png'), local['mask'])
        cv2.imwrite(str(output / 'baseline.jpg'), result.debug)
        save_csv(output / 'components.csv', comps)
        save_csv(output / 'segments.csv', segments)
        save_csv(output / 'endpoints.csv', baseline['left']['endpoints'] + baseline['right']['endpoints'])
        overlay = image.copy()
        relevant = {s['component'] for s in segments if s['slope_accepted']}
        for c in comps:
            if c['id'] in relevant and c['area'] >= 1000:
                x, y, w, h = (c[k] for k in ('x', 'y', 'width', 'height'))
                cv2.rectangle(overlay, (x, y), (x + w, y + h), (0, 0, 255), 2)
                cv2.putText(overlay, f'C{c["id"]}', (x, y + 22), 0, .65, (0, 0, 255), 2)
        cv2.imwrite(str(output / 'components.jpg'), overlay)
        overlay = image.copy()
        for s in segments:
            if s['slope_accepted']:
                used = s['id'] in baseline[s['side']]['final_segment_ids']
                color = (0, 255, 0) if used else (0, 0, 255)
                cv2.line(overlay, (s['x1'], s['y1']), (s['x2'], s['y2']), color, 2)
                cv2.putText(overlay, str(s['id']), (s['x1'], s['y1']), 0, .4, color, 1)
        cv2.imwrite(str(output / 'segments.jpg'), overlay)
        candidate = image.copy()
        for side, color in (('left', (0, 255, 255)), ('right', (255, 0, 255))):
            fit = trials['row_width'][side]['fit']
            if fit is not None:
                top, bottom = local['top'], local['bottom']
                cv2.line(candidate, (int(np.polyval(fit, top)), top),
                         (int(np.polyval(fit, bottom)), bottom), color, 3)
        cv2.putText(candidate, 'OFFLINE component-linked row-width <= 0.50',
                    (10, 30), 0, .7, (0, 255, 0), 2)
        cv2.imwrite(str(output / 'row_width_candidate.jpg'), candidate)
        (output / 'trace.json').write_text(json.dumps(data, indent=2))
    return data


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--media', default='/home/idp2/차선영상2.mp4')
    parser.add_argument('--frames', type=int, nargs='+', default=[603, 643, 884])
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    cap = cv2.VideoCapture(args.media)
    if not cap.isOpened():
        raise ValueError(f'Cannot open {args.media}')
    fps = cap.get(cv2.CAP_PROP_FPS)
    summary = []
    try:
        for index in args.frames:
            cap.set(cv2.CAP_PROP_POS_FRAMES, index)
            ok, image = cap.read()
            if not ok:
                raise ValueError(f'Cannot decode frame {index}')
            data = analyze(image, args.output / f'f{index}')
            item = dict(frame=index, time=index / fps, component_count=len(data['components']),
                        hough_count=len(data['segments']),
                        slope_accepted=sum(s['slope_accepted'] for s in data['segments']),
                        trials=data['trials'])
            summary.append(item)
            print(index, item['hough_count'], item['slope_accepted'],
                  {k: [bool(v[s]['fit']) for s in ('left', 'right')] + [v['visual']]
                   for k, v in data['trials'].items()}, flush=True)
    finally:
        cap.release()
    import fma_perception.lane_detection as detector
    (args.output / 'summary.json').write_text(json.dumps(dict(
        media=args.media, config=vars(CONFIG),
        detector_sha256=hashlib.sha256(Path(detector.__file__).read_bytes()).hexdigest(),
        opencv_version=cv2.__version__, frames=summary), indent=2))


if __name__ == '__main__':
    main()
