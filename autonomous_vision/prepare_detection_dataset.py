#!/usr/bin/env python3
"""Copy curated whole sessions into the new 3-class detector workspace.

No old detection labels are copied because they are incomplete under the new
schema. This script never performs a random image-level split.
"""

import argparse
import json
from pathlib import Path
import shutil


DEFAULT_SESSIONS = {
    "train": [
        ("train", "20260905_c_video_154831"),
        ("train", "20260905_c_video_155655"),
    ],
    "val": [
        ("val", "20260906_e_video_143738"),
        ("val", "20260907_f_yellow_photos"),
    ],
}
IMAGE_SUFFIXES = {".jpg", ".jpeg", ".png", ".bmp"}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-root", type=Path, default=Path("dataset/detection/images"))
    parser.add_argument("--output-root", type=Path,
                        default=Path("autonomous_vision/dataset_detection"))
    args = parser.parse_args()
    manifest = {"policy": "whole_session_only", "splits": {}}
    for target_split, sessions in DEFAULT_SESSIONS.items():
        manifest["splits"][target_split] = []
        for source_split, session in sessions:
            source_dir = args.source_root / source_split / session
            if not source_dir.is_dir():
                raise SystemExit(f"ERROR: source session missing: {source_dir}")
            destination = args.output_root / "images" / target_split / session
            draft_destination = args.output_root / "draft_labels" / target_split / session
            destination.mkdir(parents=True, exist_ok=True)
            draft_destination.mkdir(parents=True, exist_ok=True)
            copied = 0
            for source_path in sorted(source_dir.iterdir()):
                if source_path.suffix.lower() not in IMAGE_SUFFIXES:
                    continue
                destination_path = destination / source_path.name
                if not destination_path.exists():
                    shutil.copy2(source_path, destination_path)
                # Existing IDs 7/9/10 are verified individual LED panels. They
                # become class 1 draft boxes, never completed labels. Blank
                # panels and message_board still need manual boxes.
                old_label = (args.source_root.parent / "annotations" / source_split /
                             session / f"{source_path.stem}.txt")
                if old_label.is_file():
                    draft_lines = []
                    for line in old_label.read_text().splitlines():
                        fields = line.split()
                        if len(fields) == 5 and int(float(fields[0])) in {7, 9, 10}:
                            draft_lines.append("1 " + " ".join(fields[1:]))
                    if draft_lines:
                        (draft_destination / f"{source_path.stem}.txt").write_text(
                            "\n".join(draft_lines) + "\n")
                copied += 1
            manifest["splits"][target_split].append({
                "source_split": source_split, "session": session, "images": copied,
            })
            print(f"{target_split}/{session}: {copied} images")
    manifest_path = args.output_root / "session_manifest.json"
    manifest_path.write_text(json.dumps(manifest, indent=2, ensure_ascii=False) + "\n")
    print(f"Manifest: {manifest_path}")


if __name__ == "__main__":
    main()
