#!/usr/bin/env python3
"""Crop verified legacy panel boxes into the new classification dataset.

Legacy mission classes 7/9/10 already describe complete LED panels. Their
existing train/val/test session split is preserved; this script never performs
a random image-level split.
"""

import argparse
from collections import Counter
from pathlib import Path

import cv2


CLASS_MAP = {7: "LANE_CHANGE", 9: "RED_X", 10: "GREEN_ARROW"}
IMAGE_SUFFIXES = (".jpg", ".jpeg", ".png", ".bmp")


def build_parser():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-root", default="dataset/detection")
    parser.add_argument("--output-root", default="autonomous_vision/dataset_sign_cls")
    parser.add_argument("--padding", type=float, default=0.03,
                        help="Fractional bbox padding on each side")
    parser.add_argument("--overwrite", action="store_true")
    return parser


def find_image(image_dir, stem):
    for suffix in IMAGE_SUFFIXES:
        candidate = image_dir / f"{stem}{suffix}"
        if candidate.is_file():
            return candidate
    return None


def main():
    args = build_parser().parse_args()
    source = Path(args.source_root)
    output = Path(args.output_root)
    counts = Counter()
    skipped = Counter()
    for split in ("train", "val", "test"):
        annotation_root = source / "annotations" / split
        image_root = source / "images" / split
        if not annotation_root.is_dir() or not image_root.is_dir():
            continue
        for label_path in sorted(annotation_root.glob("*/*.txt")):
            session = label_path.parent.name
            image_path = find_image(image_root / session, label_path.stem)
            if image_path is None:
                skipped["missing_image"] += 1
                continue
            image = cv2.imread(str(image_path))
            if image is None:
                skipped["unreadable_image"] += 1
                continue
            height, width = image.shape[:2]
            for object_index, line in enumerate(label_path.read_text().splitlines()):
                fields = line.split()
                if len(fields) != 5:
                    skipped["malformed_label"] += 1
                    continue
                class_id = int(float(fields[0]))
                if class_id not in CLASS_MAP:
                    continue
                cx, cy, box_width, box_height = map(float, fields[1:])
                pad_x, pad_y = box_width * args.padding, box_height * args.padding
                x1 = max(0, int((cx - box_width / 2 - pad_x) * width))
                y1 = max(0, int((cy - box_height / 2 - pad_y) * height))
                x2 = min(width, int((cx + box_width / 2 + pad_x) * width))
                y2 = min(height, int((cy + box_height / 2 + pad_y) * height))
                if x2 - x1 < 8 or y2 - y1 < 8:
                    skipped["tiny_crop"] += 1
                    continue
                class_name = CLASS_MAP[class_id]
                destination = output / split / class_name
                destination.mkdir(parents=True, exist_ok=True)
                filename = destination / f"{session}__{label_path.stem}__obj{object_index}.jpg"
                if filename.exists() and not args.overwrite:
                    skipped["already_exists"] += 1
                    continue
                if not cv2.imwrite(str(filename), image[y1:y2, x1:x2]):
                    raise RuntimeError(f"Could not write crop: {filename}")
                counts[(split, class_name)] += 1
    for split in ("train", "val", "test"):
        summary = ", ".join(f"{name}={counts[(split, name)]}"
                            for name in ("GREEN_ARROW", "RED_X", "LANE_CHANGE"))
        print(f"{split}: {summary}")
    if skipped:
        print("skipped: " + ", ".join(f"{key}={value}" for key, value in sorted(skipped.items())))


if __name__ == "__main__":
    main()
