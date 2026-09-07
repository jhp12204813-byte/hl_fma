#!/usr/bin/env python3
"""Use the shared safe annotator for obstacle IDs 0/1 on localhost:8767."""
import argparse
from pathlib import Path
from http.server import ThreadingHTTPServer
try:
    from .label import Store, make_handler
    from .obstacle_classes import CLASS_NAMES, CLASS_DESCRIPTIONS
except ImportError:
    from label import Store, make_handler
    from obstacle_classes import CLASS_NAMES, CLASS_DESCRIPTIONS


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', type=int, default=8767)
    parser.add_argument('--dataset-root', type=Path,
                        default=Path(__file__).resolve().parent/'dataset_obstacles')
    parser.add_argument('--label-directory', choices=['labels', 'labels_clean'], default='labels',
                        help='Use labels_clean for review without editing original labels.')
    parser.add_argument('--child-only', action='store_true',
                        help='Show images currently containing child_dummy (not a vehicle mislabel audit).')
    parser.add_argument('--exclude-flips', action='store_true',
                        help='Review source images once; synchronize derived flips after review.')
    args = parser.parse_args()
    if args.label_directory == 'labels_clean' and not (args.dataset_root/'labels_clean').is_dir():
        parser.error('labels_clean is missing; prepare a separate review copy first.')
    store = Store(args.dataset_root, class_names=CLASS_NAMES,
                  descriptions=CLASS_DESCRIPTIONS, shortcuts=['0', '1'], label_directory=args.label_directory)
    if args.exclude_flips:
        store.items = [n for n in store.items if '__hflip' not in Path(n).stem]
    if args.child_only:
        store.items = [n for n in store.items if any(b[0] == 0 for b in store.read(n)['boxes'])]
    store.allowed = set(store.items)
    if not store.items:
        parser.error('No obstacle images found. Prepare the derived dataset first.')
    server = ThreadingHTTPServer(('127.0.0.1', args.port), make_handler(store))
    print(f'Obstacle labels: {len(store.items)} images at http://127.0.0.1:{args.port}; saving to {store.labels}', flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == '__main__':
    main()
