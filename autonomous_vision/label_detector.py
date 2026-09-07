#!/usr/bin/env python3
"""Launch the existing safe local annotator for the new detector schema."""

import argparse
from http.server import ThreadingHTTPServer
import os
from pathlib import Path
import sys

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

try:
    from .label import Store, make_handler  # noqa: E402
except ImportError:
    from label import Store, make_handler  # noqa: E402


CLASS_NAMES = ["traffic_light", "sign_panel", "message_board"]
CLASS_DESCRIPTIONS = [
    "가로형 4구 일반 차량 신호등 전체 하우징",
    "신호차 상단 LED 패널 한 개 (각각 별도 박스)",
    "신호차 하단 중앙의 큰 문자 전광판",
]
CLASS_SHORTCUTS = ["0", "1", "2"]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset-root", type=Path,
                        default=PROJECT_ROOT / "autonomous_vision/dataset_detection")
    parser.add_argument("--port", type=int, default=8766)
    args = parser.parse_args()
    store = Store(args.dataset_root,
                  token=os.environ.pop("DETECTOR_LABEL_SESSION_TOKEN", None),
                  class_names=CLASS_NAMES,
                  descriptions=CLASS_DESCRIPTIONS,
                  shortcuts=CLASS_SHORTCUTS,
                  label_directory="labels",
                  draft_directory="draft_labels")
    if not store.items:
        parser.error("No images found in train/val")
    server = ThreadingHTTPServer(("127.0.0.1", args.port), make_handler(store))
    print(f"Label {len(store.items)} images at http://127.0.0.1:{server.server_port}", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
