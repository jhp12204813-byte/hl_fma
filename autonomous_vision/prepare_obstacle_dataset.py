#!/usr/bin/env python3
"""Build an isolated obstacle dataset, preserving source sessions and labels.

Only legacy images with obstacle annotations are copied. Other-class boxes are
filtered, never interpreted as new obstacle IDs. MYBOX photos remain unlabelled.
Outputs are created once: refuse a nonempty destination rather than overwrite
manual work. No video extraction, training, or split randomization is performed.
"""
import argparse
from collections import Counter
import hashlib
import io
import json
import math
from pathlib import Path
import re
import shutil
import zipfile

from PIL import Image, ImageOps

try:
    from .obstacle_classes import CLASS_NAMES, LEGACY_CLASS_MAPPING
except ImportError:
    from obstacle_classes import CLASS_NAMES, LEGACY_CLASS_MAPPING

ROOT = Path(__file__).resolve().parents[1]


def mapped_lines(text):
    """Validate normalized YOLO cx/cy/w/h, retaining just source IDs 5 and 6."""
    result = []
    for line in text.splitlines():
        if not line.strip():
            continue
        fields = line.split()
        if len(fields) != 5:
            raise ValueError(f"Invalid label: {line}")
        cid = int(fields[0])
        values = list(map(float, fields[1:]))
        x, y, w, h = values
        if not all(math.isfinite(v) and 0 <= v <= 1 for v in values) or w <= 0 or h <= 0:
            raise ValueError(f"Invalid normalized box: {line}")
        if min(x-w/2, y-h/2) < -1e-6 or max(x+w/2, y+h/2) > 1+1e-6:
            raise ValueError(f"Box outside image: {line}")
        if cid in LEGACY_CLASS_MAPPING:
            result.append(str(LEGACY_CLASS_MAPPING[cid]) + " " + " ".join(fields[1:]))
    return result


def photo_seconds(name):
    match = re.search(r"P\d{8}_(\d{2})(\d{2})(\d{2})(\d{3})_", name)
    if not match:
        raise ValueError(f"Cannot infer capture sequence from {name}")
    h, m, s, ms = map(int, match.groups())
    return h*3600 + m*60 + s + ms/1000


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, default=ROOT/'dataset/detection')
    parser.add_argument('--archive', type=Path, default=ROOT/'picturesandvideos/testvideo/MYBOX.zip')
    parser.add_argument('--output', type=Path, default=ROOT/'autonomous_vision/dataset_obstacles')
    parser.add_argument('--photo-interval', type=float, default=0.8,
                        help='Minimum filename timestamp spacing between selected burst photos')
    args = parser.parse_args()
    if not math.isfinite(args.photo_interval) or args.photo_interval <= 0:
        parser.error('photo interval must be positive')
    out = args.output.resolve()
    if out.exists() and any(out.iterdir()):
        parser.error(f'Refusing to overwrite nonempty destination: {out}')
    # Preflight every retained label/image before creating derived data.
    records = []
    for split in ('train', 'val'):
        base = args.source/'annotations'/split
        for label in sorted(base.rglob('*.txt')):
            lines = mapped_lines(label.read_text())
            if not lines:
                continue
            rel = label.relative_to(base)
            stem = args.source/'images'/split/rel.with_suffix('')
            images = [stem.with_suffix(ext) for ext in ('.jpg', '.jpeg', '.png', '.bmp')
                      if stem.with_suffix(ext).is_file()]
            if len(images) != 1:
                raise ValueError(f'Expected exactly one image for {label}')
            records.append((split, rel, images[0], label, lines))
    if not records:
        raise ValueError('No legacy obstacle annotations found')
    import pi_heif
    pi_heif.register_heif_opener()
    with zipfile.ZipFile(args.archive) as archive:
        if archive.testzip() is not None:
            raise ValueError('Archive CRC check failed')
        photos = sorted(n for n in archive.namelist() if n.lower().endswith('.heic'))
        if not photos:
            raise ValueError('No HEIC photos')
        out.mkdir(parents=True, exist_ok=True)
        inventory = {'classes': CLASS_NAMES, 'legacy_mapping': LEGACY_CLASS_MAPPING,
                     'policy': 'whole_session_only; legacy positive images only',
                     'archive_sha256': hashlib.sha256(args.archive.read_bytes()).hexdigest(),
                     'photo_interval_seconds': args.photo_interval, 'items': []}
        counts = Counter()
        for split, rel, image, label, lines in records:
            dest = out/'images'/split/rel.with_suffix(image.suffix)
            target = out/'labels'/split/rel
            dest.parent.mkdir(parents=True, exist_ok=True)
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(image, dest)
            target.write_text('\n'.join(lines)+'\n')
            for line in lines:
                counts[(split, int(line.split()[0]))] += 1
            inventory['items'].append({'source': str(image.resolve()), 'source_label': str(label.resolve()),
                                       'image': str(dest.relative_to(out)), 'split': split,
                                       'session': rel.parts[0], 'status': 'mapped_label'})
        last_time = float('-inf')
        seen = set()
        selected = 0
        for name in photos:
            stamp = photo_seconds(Path(name).name)
            raw = archive.read(name)
            digest = hashlib.sha256(raw).hexdigest()
            keep = stamp-last_time >= args.photo_interval and digest not in seen
            item = {'source': str(args.archive.resolve())+'::'+name, 'sha256': digest,
                    'session': '20260907_mybox_vehicle', 'split': 'train',
                    'status': 'needs_manual_label' if keep else 'not_selected_interval_or_duplicate'}
            if keep:
                # Respect HEIC orientation; resize preserves aspect ratio and
                # labels will be drawn on the resulting image, not the source.
                with Image.open(io.BytesIO(raw)) as source:
                    im = ImageOps.exif_transpose(source).convert('RGB')
                    im.thumbnail((1920, 1920))
                    dest = out/'images/train/20260907_mybox_vehicle'/Path(name).with_suffix('.jpg').name
                    dest.parent.mkdir(parents=True, exist_ok=True)
                    im.save(dest, quality=95)
                item['image'] = str(dest.relative_to(out))
                last_time = stamp
                seen.add(digest)
                selected += 1
            inventory['items'].append(item)
        (out/'data.yaml').write_text('train: images/train\nval: images/val\nnames:\n'+
                                    ''.join(f'  {i}: {name}\n' for i, name in enumerate(CLASS_NAMES)))
        (out/'inventory.json').write_text(json.dumps(inventory, ensure_ascii=False, indent=2)+'\n')
    print('Destination:', out)
    print('Legacy images:', len(records), 'box counts:', dict(counts))
    print('Selected MYBOX photos needing labels:', selected, '/', len(photos))


if __name__ == '__main__':
    main()
