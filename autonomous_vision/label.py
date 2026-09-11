#!/usr/bin/env python3
"""Local, dependency-free YOLO annotation server. Run, then open localhost:8765.

Images stay in their existing split/session. Only an explicit save creates a
label. Existing labels are backed up, and revision checks prevent stale tabs
from overwriting newer edits. Coordinates on the wire are normalized xyxy;
on disk they are YOLO class/cx/cy/w/h, as consumed by dataset.py.
"""
import argparse
import hashlib
import json
import math
import mimetypes
import os
from pathlib import Path
import secrets
import tempfile
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

DEFAULT_CLASS_NAMES = ["object"]
DEFAULT_CLASS_DESCRIPTIONS = ["object"]
DEFAULT_CLASS_SHORTCUTS = ["0"]

ROOT = Path(__file__).resolve().parents[1]


class Store:
    def __init__(self, root, token=None, class_names=None, descriptions=None,
                 shortcuts=None, label_directory='annotations', draft_directory=None):
        self.root = Path(root).resolve()
        self.images = self.root / 'images'
        self.labels = self.root / label_directory
        self.drafts = self.root / draft_directory if draft_directory else None
        self.class_names = class_names or DEFAULT_CLASS_NAMES
        self.descriptions = descriptions or DEFAULT_CLASS_DESCRIPTIONS
        self.shortcuts = shortcuts or DEFAULT_CLASS_SHORTCUTS
        self.items = [p.relative_to(self.images).as_posix()
                      for split in ('train', 'val', 'test')
                      for p in sorted((self.images / split).rglob('*'))
                      if p.is_file() and p.suffix.lower() in {'.jpg', '.jpeg', '.png', '.bmp'}]
        self.allowed = set(self.items)
        self.token = token or secrets.token_urlsafe(32)
        self.lock = threading.Lock()

    def paths(self, name):
        if name not in self.allowed:
            raise ValueError('Unknown image')
        image = (self.images / name).resolve()
        label = (self.labels / Path(name).with_suffix('.txt')).resolve()
        if not image.is_relative_to(self.images) or not label.is_relative_to(self.labels):
            raise ValueError('Path outside dataset')
        return image, label

    def read(self, name):
        _, label = self.paths(name)
        raw = label.read_bytes() if label.exists() else None
        display_raw = raw
        if display_raw is None and self.drafts is not None:
            draft = (self.drafts / Path(name).with_suffix('.txt')).resolve()
            if draft.is_relative_to(self.drafts) and draft.exists():
                display_raw = draft.read_bytes()
        boxes = []
        if display_raw is not None:
            for line in display_raw.decode('utf-8').splitlines():
                if not line.strip():
                    continue
                c, x, y, w, h = map(float, line.split())
                if not c.is_integer():
                    raise ValueError('Non-integer class ID in existing label')
                corners = [x-w/2, y-h/2, x+w/2, y+h/2]
                # Decimal YOLO serialization can put a border at -1e-10 or
                # 1+1e-10. Correct only rounding noise, not invalid annotations.
                corners = [min(1, max(0, v)) if -1e-8 <= v <= 1+1e-8 else v for v in corners]
                boxes.append([int(c), *corners])
            self.validate(boxes)
        return {'boxes': boxes, 'saved': raw is not None,
                'revision': hashlib.sha256(raw).hexdigest() if raw is not None else None}

    def validate(self, boxes):
        if not isinstance(boxes, list) or len(boxes) > 1000:
            raise ValueError('Invalid box list')
        for box in boxes:
            if not isinstance(box, list) or len(box) != 5:
                raise ValueError('Expected class ID and normalized xyxy')
            c, x1, y1, x2, y2 = box
            if type(c) is not int or not 0 <= c < len(self.class_names):
                raise ValueError('Invalid class ID')
            if not all(type(v) in (int, float) and math.isfinite(v) for v in box[1:]):
                raise ValueError('Invalid coordinate')
            if not (0 <= x1 < x2 <= 1 and 0 <= y1 < y2 <= 1):
                raise ValueError('Box must have positive area and lie within the image')

    def save(self, data):
        name, boxes = data['image'], data['boxes']
        self.validate(boxes)
        if not boxes and data.get('confirmed_empty') is not True:
            raise ValueError('Confirm that this image has no target objects before saving empty labels')
        _, label = self.paths(name)
        with self.lock:
            current = self.read(name)
            if current['revision'] != data['revision']:
                raise ValueError('Label changed in another tab. Reload the image before editing.')
            lines = []
            for c, x1, y1, x2, y2 in boxes:
                lines.append(f'{c} {(x1+x2)/2:.10f} {(y1+y2)/2:.10f} {x2-x1:.10f} {y2-y1:.10f}\n')
            label.parent.mkdir(parents=True, exist_ok=True)
            if label.exists():
                backup = self.root / 'label_history' / Path(name).with_suffix('') / (current['revision'] + '.txt')
                backup.parent.mkdir(parents=True, exist_ok=True)
                if not backup.exists():
                    backup.write_bytes(label.read_bytes())
            # Atomic replacement means an interrupted save cannot truncate a label.
            temp_name = None
            try:
                with tempfile.NamedTemporaryFile(mode='w', encoding='utf-8', dir=label.parent, delete=False) as f:
                    temp_name = f.name
                    f.write(''.join(lines))
                    f.flush()
                    os.fsync(f.fileno())
                os.replace(temp_name, label)
            finally:
                if temp_name and Path(temp_name).exists():
                    Path(temp_name).unlink()
            return self.read(name)


def make_handler(store):
    class Handler(BaseHTTPRequestHandler):
        def send(self, body, content_type='application/json', status=200):
            if not isinstance(body, bytes):
                body = json.dumps(body, ensure_ascii=False).encode('utf-8')
            self.send_response(status)
            self.send_header('Content-Type', content_type)
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            self.send_header('X-Content-Type-Options', 'nosniff')
            self.end_headers()
            self.wfile.write(body)

        def local_request(self):
            return self.headers.get('Host') in {f'127.0.0.1:{self.server.server_port}', f'localhost:{self.server.server_port}'}

        def do_GET(self):
            if not self.local_request():
                return self.send({'error': 'Local requests only'}, status=403)
            try:
                url = urlparse(self.path)
                name = parse_qs(url.query).get('image', [''])[0]
                if url.path == '/':
                    return self.send(Path(__file__).with_name('label.html').read_bytes(), 'text/html; charset=utf-8')
                if url.path == '/api/catalog':
                    return self.send({'classes': store.class_names,
                                      'descriptions': store.descriptions,
                                      'shortcuts': store.shortcuts, 'token': store.token,
                                      'items': [{'name': n, 'saved': store.paths(n)[1].exists()} for n in store.items]})
                if url.path == '/api/label':
                    return self.send(store.read(name))
                if url.path == '/api/image':
                    path, _ = store.paths(name)
                    return self.send(path.read_bytes(), mimetypes.guess_type(path.name)[0] or 'application/octet-stream')
                self.send({'error': 'Not found'}, status=404)
            except (ValueError, OSError, KeyError) as exc:
                self.send({'error': str(exc)}, status=400)

        def do_POST(self):
            if not self.local_request() or self.headers.get('X-Label-Token') != store.token:
                return self.send({'error': 'Invalid local session'}, status=403)
            try:
                if self.path != '/api/label':
                    return self.send({'error': 'Not found'}, status=404)
                length = int(self.headers.get('Content-Length', '0'))
                if not 0 < length <= 200000:
                    raise ValueError('Invalid request size')
                data = json.loads(self.rfile.read(length))
                self.send(store.save(data))
            except (ValueError, OSError, KeyError, TypeError) as exc:
                self.send({'error': str(exc)}, status=400)

        def log_message(self, *_):
            pass
    return Handler


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dataset-root', type=Path,
                        default=ROOT / 'autonomous_vision/dataset_detection')
    parser.add_argument('--port', type=int, default=8765)
    args = parser.parse_args()
    # During a controlled restart, retain the current token so already open tabs
    # can save their in-memory boxes before refreshing for newly added classes.
    store = Store(args.dataset_root, token=os.environ.pop('MISSION_LABEL_SESSION_TOKEN', None))
    if not store.items:
        parser.error('No images found in train/val/test')
    server = ThreadingHTTPServer(('127.0.0.1', args.port), make_handler(store))
    print(f'Label {len(store.items)} images at http://127.0.0.1:{server.server_port}', flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == '__main__':
    main()
