#!/usr/bin/env python3
"""Train traffic_light / sign_panel / message_board in one detector."""

import argparse
from pathlib import Path


IMAGE_SUFFIXES = {".jpg", ".jpeg", ".png", ".bmp"}
VALID_CLASS_IDS = {0, 1, 2}


def build_parser():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data", default="autonomous_vision/dataset_detection/data.yaml")
    parser.add_argument("--model", default="yolo11n.pt")
    parser.add_argument("--epochs", type=int, default=200)
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--batch", type=int, default=4)
    parser.add_argument("--lr0", type=float, default=1e-3)
    parser.add_argument("--device", default=None)
    parser.add_argument("--project", default="autonomous_vision/runs")
    parser.add_argument("--name", default="detector")
    return parser


def validate_dataset(data_yaml):
    """Fail before model download when data or corresponding labels are absent."""
    root = data_yaml.parent
    for split in ("train", "val"):
        image_dir = root / "images" / split
        label_dir = root / "labels" / split
        if not image_dir.is_dir() or not label_dir.is_dir():
            raise FileNotFoundError(f"Missing {split} image/label directories under {root}")
        # Images are grouped below each split by recording/session.  Keep that
        # hierarchy intact to prevent adjacent video frames leaking across
        # train/validation, and validate it recursively here.
        images = [path for path in image_dir.rglob("*")
                  if path.is_file() and path.suffix.lower() in IMAGE_SUFFIXES]
        if not images:
            raise RuntimeError(f"No {split} images found in {image_dir}")
        missing = []
        for path in images:
            relative_label = path.relative_to(image_dir).with_suffix(".txt")
            if not (label_dir / relative_label).is_file():
                missing.append(path.relative_to(image_dir).as_posix())
        if missing:
            raise RuntimeError(
                f"{split} images without matching .txt labels: {', '.join(missing[:5])}. "
                "Use an empty label file only for a verified negative image."
            )
        for image in images:
            label_path = label_dir / image.relative_to(image_dir).with_suffix(".txt")
            for line_number, line in enumerate(label_path.read_text().splitlines(), start=1):
                fields = line.split()
                if len(fields) != 5:
                    raise RuntimeError(f"Invalid YOLO label at {label_path}:{line_number}")
                try:
                    class_value = float(fields[0])
                    coordinates = [float(value) for value in fields[1:]]
                except ValueError as exc:
                    raise RuntimeError(
                        f"Non-numeric YOLO label at {label_path}:{line_number}"
                    ) from exc
                class_id = int(class_value)
                if class_value != class_id or class_id not in VALID_CLASS_IDS:
                    raise RuntimeError(f"Class id must be 0, 1, or 2 at {label_path}:{line_number}")
                if not all(0.0 <= value <= 1.0 for value in coordinates) or any(
                        value <= 0.0 for value in coordinates[2:]):
                    raise RuntimeError(f"Invalid normalized bbox at {label_path}:{line_number}")


def main():
    args = build_parser().parse_args()
    data_yaml = Path(args.data)
    if not data_yaml.is_file():
        raise SystemExit(f"ERROR: dataset YAML not found: {data_yaml}")
    try:
        validate_dataset(data_yaml)
        from ultralytics import YOLO
    except (ImportError, FileNotFoundError, RuntimeError) as exc:
        raise SystemExit(f"ERROR: {exc}") from exc
    project = Path(args.project).resolve()
    model = YOLO(args.model)
    model.train(data=str(data_yaml), epochs=args.epochs, imgsz=args.imgsz,
                batch=args.batch, lr0=args.lr0, device=args.device,
                project=str(project), name=args.name)
    print(f"Best checkpoint: {project / args.name / 'weights/best.pt'}")


if __name__ == "__main__":
    main()
