#!/usr/bin/env python3
"""Train GREEN_ARROW / RED_X / LANE_CHANGE image classification."""

import argparse
from pathlib import Path


EXPECTED_CLASSES = {"GREEN_ARROW", "RED_X", "LANE_CHANGE"}
IMAGE_SUFFIXES = {".jpg", ".jpeg", ".png", ".bmp"}


def build_parser():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data", default="autonomous_vision/dataset_sign_cls")
    parser.add_argument("--model", default="yolo11n-cls.pt")
    parser.add_argument("--epochs", type=int, default=100)
    parser.add_argument("--imgsz", type=int, default=224)
    parser.add_argument("--batch", type=int, default=16)
    parser.add_argument("--patience", type=int, default=30)
    parser.add_argument("--workers", type=int, default=4)
    parser.add_argument("--device", default=None)
    parser.add_argument("--project", default="autonomous_vision/runs")
    parser.add_argument("--name", default="sign_classifier")
    return parser


def validate_dataset(root):
    for split in ("train", "val"):
        split_dir = root / split
        actual = {item.name for item in split_dir.iterdir() if item.is_dir()} \
            if split_dir.is_dir() else set()
        if actual != EXPECTED_CLASSES:
            raise RuntimeError(
                f"{split} class folders must be exactly {sorted(EXPECTED_CLASSES)}; got {sorted(actual)}"
            )
        for class_name in EXPECTED_CLASSES:
            images = [item for item in (split_dir / class_name).iterdir()
                      if item.suffix.lower() in IMAGE_SUFFIXES]
            if not images:
                raise RuntimeError(f"No images in {split_dir / class_name}")


def main():
    args = build_parser().parse_args()
    data_root = Path(args.data)
    try:
        validate_dataset(data_root)
        from ultralytics import YOLO
    except (ImportError, OSError, RuntimeError) as exc:
        raise SystemExit(f"ERROR: {exc}") from exc
    project = Path(args.project).resolve()
    model = YOLO(args.model)
    model.train(data=str(data_root), epochs=args.epochs, imgsz=args.imgsz,
                batch=args.batch, device=args.device,
                project=str(project), name=args.name,
                patience=args.patience, workers=args.workers,
                # Mirroring Korean text creates impossible samples. Random
                # erasing is also too destructive for these small LED crops.
                fliplr=0.0, flipud=0.0, erasing=0.0, auto_augment=None)
    print(f"Best checkpoint: {project / args.name / 'weights/best.pt'}")


if __name__ == "__main__":
    main()
