"""Fixed-square evaluation; optional review artifacts never change source labels."""
import argparse
import json
from pathlib import Path
import sys

import numpy as np
from ultralytics import YOLO

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from autonomous_vision.detection_metrics import match_detections
from autonomous_vision.obstacle_classes import CLASS_NAMES


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--model', type=Path, default=ROOT/'autonomous_vision/runs/obstacle_50ep/weights/best.pt')
    p.add_argument('--data', type=Path, default=ROOT/'autonomous_vision/dataset_obstacles')
    p.add_argument('--imgsz', type=int, default=640)
    p.add_argument('--device', default='0')
    p.add_argument('--batch', type=int, default=4, help='Maximum images passed to predict at once')
    p.add_argument('--output', type=Path, help='New directory for JSON and GT/prediction review sheets')
    a = p.parse_args()
    if a.batch < 1:
        p.error('batch must be positive')
    if a.output:
        a.output.mkdir(parents=True, exist_ok=False)
    report = {'model': str(a.model), 'imgsz': a.imgsz, 'rect': False, 'batch': a.batch,
              'nms_iou': .7, 'matching_iou': .5, 'splits': {}}
    model = YOLO(a.model)
    if model.names != dict(enumerate(CLASS_NAMES)):
        raise ValueError('Expected obstacle classes 0 child_dummy / 1 vehicle_obstacle')
    print('Model:', a.model, model.names)
    for split in ('train', 'val'):
        base = a.data/'images'/split
        paths = sorted(x for x in base.rglob('*') if x.suffix.lower() in ('.jpg', '.jpeg', '.png'))
        if not paths:
            raise ValueError(f'No images: {base}')
        totals = {(t, c): [0, 0, 0] for t in (.05, .25, .4) for c in model.names}
        records, tiles = [], []
        # Explicit recursive paths preserve the recording-session subdirectories.
        # A Python image list is treated as one batch by Ultralytics. Explicitly
        # chunk it; stream=True alone does not bound GPU memory for list inputs.
        predictions = (r for start in range(0, len(paths), a.batch)
                       for r in model.predict(source=[str(x) for x in paths[start:start+a.batch]],
                                              conf=.05, iou=.7, imgsz=a.imgsz, rect=False,
                                              device=a.device, stream=True, verbose=False))
        for r in predictions:
            label = a.data/'labels'/split/Path(r.path).relative_to(base).with_suffix('.txt')
            rows = np.loadtxt(label, ndmin=2) if label.read_text().strip() else np.empty((0, 5))
            h, w = r.orig_shape
            gc = rows[:, 0]
            xy = rows[:, 1:3]*[w, h]
            wh = rows[:, 3:5]*[w, h]
            gb = np.concatenate([xy-wh/2, xy+wh/2], axis=1)
            pc = r.boxes.cls.cpu().numpy()
            pb = r.boxes.xyxy.cpu().numpy()
            ps = r.boxes.conf.cpu().numpy()
            matched = match_detections(gc, gb, pc, pb, ps, confidence=.25, iou_threshold=.5)
            records.append({'image': str(Path(r.path).relative_to(base)),
                            'gt_classes': gc.tolist(), 'gt_boxes_xyxy': gb.tolist(),
                            'pred_classes': pc[ps >= .25].tolist(),
                            'pred_boxes_xyxy': pb[ps >= .25].tolist(),
                            'scores': ps[ps >= .25].tolist(),
                            'conf025': {k: matched[k] for k in ('tp', 'fp', 'fn')}})
            if a.output:
                import cv2
                # Green = ground truth, orange = predictions. Do not auto-correct GT from predictions.
                pair = []
                for title, classes, boxes, scores, color in (
                    ('GT', gc, gb, None, (0, 200, 0)),
                    ('PRED conf >= .25', pc[ps >= .25], pb[ps >= .25], ps[ps >= .25], (0, 140, 255))):
                    scale = min(480/w, 320/h)
                    canvas = np.full((360, 480, 3), 35, dtype=np.uint8)
                    resized = cv2.resize(r.orig_img, (round(w*scale), round(h*scale)))
                    canvas[40:40+resized.shape[0], :resized.shape[1]] = resized
                    cv2.putText(canvas, title, (5, 16), 0, .45, color, 1)
                    for j, (c, box) in enumerate(zip(classes, boxes)):
                        x1, y1, x2, y2 = np.rint(box*scale).astype(int)
                        cv2.rectangle(canvas, (x1, y1+40), (x2, y2+40), color, 1)
                        text = CLASS_NAMES[int(c)] + (f' {scores[j]:.2f}' if scores is not None else '')
                        cv2.putText(canvas, text, (x1, max(52, y1+38)), 0, .35, color, 1)
                    pair.append(canvas)
                tile = np.hstack(pair)
                header = f'{len(records):02d} {Path(r.path).stem[:32]} TP{matched["tp"]} FP{matched["fp"]} FN{matched["fn"]}'
                cv2.putText(tile, header, (5, 32), 0, .4, (255, 255, 255), 1)
                tiles.append(tile)
            for (t, c), counts in totals.items():
                g, pred = gc == c, pc == c
                m = match_detections(gc[g], gb[g], pc[pred], pb[pred], ps[pred],
                                     confidence=t, iou_threshold=.5)
                for j, key in enumerate(('tp', 'fp', 'fn')):
                    counts[j] += m[key]
        for (t, c), (tp, fp, fn) in totals.items():
            f1 = 2*tp/(2*tp+fp+fn) if 2*tp+fp+fn else 0.
            print(f'{split} conf={t} {model.names[c]} TP={tp} FP={fp} FN={fn} F1={f1:.4f}')
        report['splits'][split] = {'images': records, 'totals': [
            {'confidence': t, 'class': model.names[c], 'tp': counts[0], 'fp': counts[1], 'fn': counts[2]}
            for (t, c), counts in totals.items()]}
        if a.output:
            for start in range(0, len(tiles), 4):
                target = a.output/f'{split}_{start//4+1:02d}.jpg'
                if not cv2.imwrite(str(target), np.vstack(tiles[start:start+4])):
                    raise OSError(f'Cannot write {target}')
    if a.output:
        (a.output/'review.json').write_text(json.dumps(report, indent=2)+'\n')


if __name__ == '__main__':
    main()
