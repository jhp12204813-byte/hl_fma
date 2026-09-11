"""Add one horizontal mirror per child-only TRAIN image in the obstacle dataset.

Not for the legacy signal dataset: directional signal classes must not be flipped.
Originals, validation and test stay unchanged. Mirrors retain their source session
and are augmentation, not independent recording sessions. No training is started.
"""
import argparse
import hashlib
import json
from pathlib import Path

from PIL import Image, ImageOps

try:
    from .label import Store
    from .obstacle_classes import CLASS_NAMES
    from .train_obstacles import validate
except ImportError:
    from label import Store
    from obstacle_classes import CLASS_NAMES
    from train_obstacles import validate


def flip_boxes(boxes):
    """Normalized xyxy: [x1,y1,x2,y2] -> [1-x2,y1,1-x1,y2]."""
    return [[c, 1-x2, y1, 1-x1, y2] for c, x1, y1, x2, y2 in boxes]


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--apply', action='store_true', help='Write mirrors; default is dry-run')
    a = p.parse_args()
    root = Path(__file__).resolve().parent/'dataset_obstacles'
    manifest = root/'child_flip_manifest.json'
    if manifest.exists():
        p.error('Mirrors already recorded; refusing repeated augmentation')
    before = validate(root/'data.yaml')
    store = Store(root, class_names=CLASS_NAMES, label_directory='labels')
    child_id = CLASS_NAMES.index('child_dummy')
    plans = []
    for name in store.items:
        if not name.startswith('train/') or '__hflip' in Path(name).stem:
            continue
        boxes = store.read(name)['boxes']
        if not boxes or any(b[0] != child_id for b in boxes):
            continue
        image, label = store.paths(name)
        # Lossless PNG allows exact pixel verification of the geometric transform.
        target = image.with_name(image.stem+'__hflip.png')
        target_label = label.with_name(label.stem+'__hflip.txt')
        if target.exists() or target_label.exists():
            raise ValueError(f'Destination exists: {target}')
        reflected = flip_boxes(boxes)
        store.validate(reflected)
        with Image.open(image) as im:
            if im.getexif().get(274, 1) != 1:
                raise ValueError(f'Non-normalized EXIF orientation; review before flipping: {image}')
            im.verify()
        plans.append((image, label, target, target_label, reflected))
    print('Eligible child-only train images:', len(plans))
    if not a.apply:
        return
    # Record full source snapshot first: partial runs can be audited, not repeated blindly.
    state = dict(status='in_progress', policy='child-only train; same session; no val/test flips',
                 before=before, copies=[])
    manifest.write_text(json.dumps(state, indent=2)+'\n')
    for image, label, target, target_label, boxes in plans:
        with Image.open(image) as im:
            mirrored = ImageOps.mirror(im.convert('RGB'))
            with target.open('xb') as f:
                mirrored.save(f, format='PNG')
        text = ''.join(f'{c} {(x1+x2)/2:.10f} {(y1+y2)/2:.10f} {x2-x1:.10f} {y2-y1:.10f}\n'
                       for c, x1, y1, x2, y2 in boxes)
        with target_label.open('x') as f:
            f.write(text)
        state['copies'].append(dict(source=str(image.relative_to(root)),
                                    source_label=str(label.relative_to(root)),
                                    source_sha256=digest(image), source_label_sha256=digest(label),
                                    image=str(target.relative_to(root)), label=str(target_label.relative_to(root)),
                                    image_sha256=digest(target), label_sha256=digest(target_label)))
        manifest.write_text(json.dumps(state, indent=2)+'\n')
    state['after'] = validate(root/'data.yaml')['counts']
    state['status'] = 'complete'
    manifest.write_text(json.dumps(state, indent=2)+'\n')
    print(state['after'])


if __name__ == '__main__':
    main()
