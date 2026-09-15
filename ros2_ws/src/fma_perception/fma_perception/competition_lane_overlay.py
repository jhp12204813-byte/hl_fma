"""Read-only competition candidate overlay; never fed into perception/control."""
import cv2
import numpy as np


def draw_competition_overlay(image, bev, result):
    out = image.copy()
    selected = {result[s]['id'] for s in ('left', 'right') if result.get(s) is not None}
    label_rows = {'left': 0, 'right': 0}
    for candidate in result['candidates']:
        points = candidate['points_m']
        pixels = np.array([bev.ground_to_bev_pixel(x, y) for x, y in points], np.int32)
        if not len(pixels):
            continue
        rejected = candidate['reject_reason'] is not None
        color = (0, 0, 255) if rejected else (0, 220, 220)
        valid = (pixels[:, 0] >= 0) & (pixels[:, 0] < bev.width) & (pixels[:, 1] >= 0) & (pixels[:, 1] < bev.height)
        out[pixels[valid, 1], pixels[valid, 0]] = color
        if 'coefficients' in candidate:
            ys = np.linspace(candidate['y_min_m'], candidate['y_max_m'], 60)
            curve = np.array([bev.ground_to_bev_pixel(np.polyval(candidate['coefficients'], y), y) for y in ys], np.int32)
            cv2.polylines(out, [curve], False, (0, 255, 0) if candidate['id'] in selected else color,
                          4 if candidate['id'] in selected else 1)
        side = candidate.get('side', 'left' if np.median(points[:, 0]) < 0 else 'right')
        x0 = 5 if side == 'left' else bev.width//2 + 5
        y0 = 15 + 34*label_rows[side]
        label_rows[side] += 1
        heading = '-' if 'heading' not in candidate else f"{np.degrees(candidate['heading']):.1f}"
        residual = '-' if 'residual_m' not in candidate else f"{candidate['residual_m']:.3f}"
        labels = [f"{candidate['id']} len={candidate['length_m']:.2f} t={candidate['thickness_m']:.2f}m",
                  f"h={heading} r={residual} score={candidate['score']:.2f}"]
        for row, label in enumerate(labels):
            cv2.putText(out, label, (x0, y0+14*row), cv2.FONT_HERSHEY_SIMPLEX, .33, color, 1, cv2.LINE_AA)
        if rejected:
            cv2.rectangle(out, tuple(pixels.min(axis=0)), tuple(pixels.max(axis=0)), color, 1)
    center = result.get('virtual_center')
    if center is not None:
        ys = np.linspace(center['y_min_m'], center['y_max_m'], 60)
        curve = np.array([bev.ground_to_bev_pixel(np.polyval(center['coefficients'], y), y) for y in ys], np.int32)
        cv2.polylines(out, [curve], False, (255, 0, 255), 3)
    footer = np.zeros((90, bev.width, 3), np.uint8)
    def metric(value):
        return 'N/A' if value is None else f'{value:.3f}'
    for i, line in enumerate([
        f"{result['source']} confidence={result['confidence']:.2f} {result['confidence_grade']}",
        f"width={metric(result['selected_lane_width_m'])}m EMA={metric(result['expected_lane_width_m'])}m",
        f"best={metric(result['selected_pair_score'])} second={metric(result['second_pair_score'])} reject={result['reject_reason']}"]):
        cv2.putText(footer, line, (5, 20+i*25), cv2.FONT_HERSHEY_SIMPLEX, .38, (255,255,255), 1, cv2.LINE_AA)
    return np.vstack([out, footer])
