#!/usr/bin/env python3
"""Train isolated obstacle weights; never overwrite the signal detector.

Uses obstacle IDs 0/1, normalized YOLO labels and complete recording splits.
Best checkpoint is selected by Ultralytics validation fitness, not training loss.
"""
import argparse
from collections import Counter
import hashlib
import json
import os
from pathlib import Path

try:
    from .label import Store
    from .obstacle_classes import CLASS_NAMES
except ImportError:
    from label import Store
    from obstacle_classes import CLASS_NAMES

ROOT = Path(__file__).resolve().parents[1]


def validate(data):
    import yaml
    cfg = yaml.safe_load(data.read_text())
    if cfg.get('names') != dict(enumerate(CLASS_NAMES)):
        raise ValueError('Expected obstacle IDs 0 child_dummy / 1 vehicle_obstacle')
    if cfg.get('path') or any(cfg.get(s) != f'images/{s}' for s in ('train', 'val')):
        raise ValueError('Expected local images/train and images/val paths')
    store = Store(data.parent, class_names=CLASS_NAMES, label_directory='labels')
    counts = Counter(); seen = {}; sessions = {}; snapshot = []
    for name in store.items:
        split, session = name.split('/')[:2]
        if split not in ('train', 'val'):
            continue
        label = store.read(name)
        if not label['saved']:
            raise ValueError(f'Missing annotation: {name}')
        image_path, label_path = store.paths(name)
        digest = hashlib.sha256(image_path.read_bytes()).hexdigest()
        if seen.setdefault(digest, split) != split or sessions.setdefault(session, split) != split:
            raise ValueError(f'Cross-split duplicate or session: {name}')
        counts[split] += 1
        for box in label['boxes']:
            counts[f'{split}/{CLASS_NAMES[box[0]]}'] += 1
        snapshot.append({'image': name, 'image_sha256': digest,
                         'label_sha256': hashlib.sha256(label_path.read_bytes()).hexdigest()})
    for split in ('train', 'val'):
        if not counts[split]:
            raise ValueError(f'No {split} images')
        for name in CLASS_NAMES:
            if not counts[f'{split}/{name}']:
                raise ValueError(f'No boxes: {split}/{name}')
    return {'counts': dict(counts), 'files': snapshot}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--data', type=Path, default=ROOT/'autonomous_vision/dataset_obstacles/data.yaml')
    p.add_argument('--model', default=str(ROOT/'yolo11n.pt'))
    p.add_argument('--epochs', type=int, default=50)
    p.add_argument('--batch', type=int, default=4)
    p.add_argument('--imgsz', type=int, default=640)
    p.add_argument('--lr0', type=float, default=1e-3)
    p.add_argument('--device', default='0')
    p.add_argument('--name', default='obstacle_50ep')
    a = p.parse_args()
    project = ROOT/'autonomous_vision/runs'
    if Path(a.name).name != a.name or a.name in ('.', '..'):
        p.error('name must be a single run folder name')
    run = project/a.name
    if run.exists():
        p.error(f'Run exists; choose another --name: {run}')
    if min(a.epochs, a.batch, a.imgsz) <= 0 or not 0 < a.lr0 < 1:
        p.error('Invalid training parameters')
    snapshot = validate(a.data.resolve())
    print(snapshot['counts'], flush=True)
    # Ultralytics' cache hash may miss same-length coordinate edits. Preserve
    # existing derived-dataset caches and force a fresh label scan on every run.
    # Restrict targets to this validated dataset's labels, never source data.
    caches = sorted((a.data.resolve().parent/'labels').rglob('*.cache'))
    if caches:
        backup = project/(a.name+'_cache_backup')
        backup.mkdir(parents=True, exist_ok=False)
        for cache in caches:
            target = backup/cache.relative_to(a.data.resolve().parent/'labels')
            target.parent.mkdir(parents=True, exist_ok=True)
            cache.rename(target)
        print(f'Preserved {len(caches)} caches at {backup}; labels will be rescanned.', flush=True)
    os.environ.setdefault('YOLO_AUTOINSTALL', 'false')
    from ultralytics import YOLO
    model = YOLO(a.model)
    # Existing turn requests prohibit default horizontal flips. Keep this
    # conservative for future schema reuse as well. No test split is used.
    model.train(data=str(a.data.resolve()), epochs=a.epochs, batch=a.batch,
                imgsz=a.imgsz, device=a.device, optimizer='AdamW', lr0=a.lr0,
                fliplr=0.0, flipud=0.0, workers=2, seed=0,
                project=str(project), name=a.name, exist_ok=False)
    saved = Path(model.trainer.save_dir)
    (saved/'dataset_snapshot.json').write_text(json.dumps(snapshot, indent=2)+'\n')
    print('Best obstacle checkpoint:', saved/'weights/best.pt', flush=True)


if __name__ == '__main__':
    main()
