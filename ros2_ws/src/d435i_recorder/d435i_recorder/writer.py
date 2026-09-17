"""Bounded ingestion; ordered MP4/JSONL with bounded depth and analysis workers."""
import json
import queue
import threading
import time
from collections import Counter, deque
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime
from pathlib import Path
import cv2
import numpy as np
from .stop_line import candidates, measure
from .depth_png import write_depth_png


def dump(path, value):
    temporary = path.with_suffix(path.suffix + '.tmp')
    temporary.write_text(json.dumps(value, indent=2, allow_nan=False) + '\n')
    temporary.replace(path)


class MeasuredQueue(queue.Queue):
    """Peak measured under Queue's mutex, before a consumer can dequeue."""
    def _init(self, maxsize):
        super()._init(maxsize)
        self.peak = 0

    def _put(self, item):
        super()._put(item)
        if item is not None:
            self.peak = max(self.peak, self._qsize())


class Writer:
    def __init__(self, root, metadata, capacity=512):
        self.path = Path(root) / datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        self.path.mkdir(parents=True, exist_ok=False)
        (self.path / 'depth').mkdir()
        self.metadata = dict(metadata, start_wall_ns=time.time_ns(), state='recording')
        self.queue = MeasuredQueue(maxsize=capacity)
        self.lock = threading.Lock()
        self.closed = False
        self.errors = []
        self.dropped = Counter()
        self.counts = Counter()
        self.latest_results = []
        self.queue_peak = 0
        self.overflow_events = []
        self.depth_pending_peak = 0
        self.analysis_pending_peak = 0
        self.latency = {}
        self.latency_totals = {}
        self.latency_max = {}
        self.latency_counts = Counter()
        dump(self.path / 'session.json', self.metadata)
        self.thread = threading.Thread(target=self._run, name='d435i-writer')
        self.thread.start()

    def submit(self, kind, data, meta):
        with self.lock:
            if self.closed:
                return False
            try:
                self.queue.put_nowait((kind, data, meta, time.monotonic_ns()))
                self.queue_peak = self.queue.peak
                return True
            except queue.Full:
                self.dropped[kind] += 1
                if len(self.overflow_events) < 100:
                    self.overflow_events.append(dict(kind=kind, wall_ns=time.time_ns(),
                        monotonic_ns=time.monotonic_ns(), queue_size=self.queue.qsize()))
                if 'queue overflow' not in self.errors:
                    self.errors.append('queue overflow')
                return False

    def observe_latency(self, name, start_ns):
        milliseconds = (time.monotonic_ns() - start_ns) / 1e6
        self.latency.setdefault(name, deque(maxlen=10000)).append(milliseconds)
        self.latency_counts[name] += 1
        self.latency_totals[name] = self.latency_totals.get(name, 0.) + milliseconds
        self.latency_max[name] = max(self.latency_max.get(name, 0.), milliseconds)
        return milliseconds

    def metrics(self):
        return dict(queue_capacity=self.queue.maxsize, queue_peak=self.queue.peak,
                    overflow_events=list(self.overflow_events),
                    analysis_pending_capacity=8, analysis_pending_peak=self.analysis_pending_peak,
                    depth_workers=2, depth_pending_capacity=32, depth_pending_peak=self.depth_pending_peak,
                    png_encoding='16-bit grayscale, filter None, zlib level 0',
                    latency_ms={name: dict(count=self.latency_counts[name],
                        mean=self.latency_totals[name]/self.latency_counts[name],
                        max=self.latency_max[name], p95_recent=float(np.percentile(values, 95)))
                        for name, values in self.latency.items()},
                    latency_note='wall duration; p95 uses last <=10000 samples; writes include OS cache, not fsync')

    def close(self, extra=None):
        with self.lock:
            self.closed = True
        self.queue.put(None)
        self.thread.join()
        self.metadata.update(extra or {})
        self.metadata.update(end_wall_ns=time.time_ns(), state='closed',
                             stream_counts=dict(self.counts), writer_errors=self.errors,
                             dropped_counts=dict(self.dropped), writer_metrics=self.metrics())
        dump(self.path / 'session.json', self.metadata)
        from .verification import verify
        result = verify(self.path)
        self.metadata['verification_summary'] = result['status']
        dump(self.path / 'session.json', self.metadata)
        return result

    def _run(self):
        end_of_queue = False
        files = {}
        video = None
        depths = deque(maxlen=90)
        pending = deque()
        last_analysis = None
        depth_worker = ThreadPoolExecutor(max_workers=2, thread_name_prefix='d435i-depth')
        depth_jobs = deque()
        analysis_worker = ThreadPoolExecutor(max_workers=1, thread_name_prefix='d435i-analysis')

        def save_depth(filename, data):
            begin = time.monotonic_ns()
            write_depth_png(self.path / filename, data)
            return begin, time.monotonic_ns()

        def collect_depth(block=False):
            while depth_jobs and (block or depth_jobs[0][0].done()):
                future, record = depth_jobs.popleft()
                begin, end = future.result()
                duration = (end - begin) / 1e6
                name = 'depth_png_write'
                self.latency.setdefault(name, deque(maxlen=10000)).append(duration)
                self.latency_counts[name] += 1
                self.latency_totals[name] = self.latency_totals.get(name, 0.) + duration
                self.latency_max[name] = max(self.latency_max.get(name, 0.), duration)
                row('depth_frames', dict(record, write_latency_ms=duration))
                self.counts['depth'] += 1
                if block:
                    break

        def row(name, value):
            files[name].write(json.dumps(value, allow_nan=False) + '\n')

        def analyze(force=False):
            now = time.monotonic()
            while pending and (force or now - pending[0][0] >= .12):
                if not force and not pending[0][2].done():
                    break
                _, stamp, future = pending.popleft()
                boxes = future.result()
                results = [measure(stamp, box, depths, self.metadata['depth_scale_m'])
                           for box in boxes]
                if not results:
                    results = [dict(rgb_timestamp_ns=stamp, bbox=None,
                                    detection_status='no_candidate', depth_status='not_applicable',
                                    matched_depth_timestamp_ns=None, timestamp_difference_ns=None,
                                    optical_z_m=None, valid_pixel_count=0, valid_ratio=0.)]
                for result in results:
                    row('stop_lines', result)
                    self.counts['stop_lines'] += 1
                self.latest_results = results

        try:
            for name in ('frames', 'depth_frames', 'imu', 'device_metadata', 'stop_lines'):
                files[name] = (self.path / (name + '.jsonl')).open('x')
            video = cv2.VideoWriter(str(self.path / 'color.mp4'),
                                    cv2.VideoWriter_fourcc(*'mp4v'), 30., (640, 480))
            if not video.isOpened():
                raise RuntimeError('MP4 writer could not open')
            while True:
                item = self.queue.get()
                if item is None:
                    end_of_queue = True
                    while depth_jobs:
                        collect_depth(True)
                    analyze(True)
                    break
                kind, data, meta, queued_ns = item
                self.observe_latency("queue_wait", queued_ns)
                try:
                    if kind == 'rgb':
                        if data.shape != (480, 640, 3) or data.dtype != np.uint8:
                            raise ValueError('RGB must be uint8 640x480 BGR')
                        write_started = time.monotonic_ns()
                        video.write(data)
                        meta = dict(meta, write_latency_ms=self.observe_latency("mp4_write", write_started))
                        row('frames', dict(meta, index=self.counts[kind]))
                        stamp = meta['timestamp_ns']
                        if last_analysis is None or stamp - last_analysis >= 100_000_000 or stamp < last_analysis:
                            if len(pending) >= 8:
                                pending[0][2].result()
                                analyze(True)
                            pending.append((time.monotonic(), stamp, analysis_worker.submit(candidates, data)))
                            self.analysis_pending_peak = max(self.analysis_pending_peak, len(pending))
                            last_analysis = stamp
                    elif kind == 'depth':
                        if data.shape != (480, 640) or data.dtype != np.uint16:
                            raise ValueError('depth must be uint16 640x480')
                        collect_depth()
                        if len(depth_jobs) >= 32:
                            collect_depth(True)
                        index = self.counts['depth'] + len(depth_jobs)
                        filename = f'depth/{index:06d}.png'
                        record = dict(meta, index=index, filename=filename,
                                      depth_scale_m=self.metadata['depth_scale_m'])
                        depth_jobs.append((depth_worker.submit(save_depth, filename, data), record))
                        self.depth_pending_peak = max(self.depth_pending_peak, len(depth_jobs))
                        depths.append((meta['timestamp_ns'], data))
                    else:
                        row(kind, dict(meta, **data))
                    if kind != 'depth':
                        self.counts[kind] += 1
                    analyze()
                except Exception as exc:
                    if len(self.errors) < 100:
                        self.errors.append(f'{kind}: {exc}')
        except Exception as exc:
            self.errors.append(f'writer fatal: {exc}')
            # Keep draining so close cannot deadlock on a full queue after failure.
            if not end_of_queue:
                while self.queue.get() is not None:
                    pass
        finally:
            depth_worker.shutdown(wait=True)
            analysis_worker.shutdown(wait=True)
            if video is not None:
                video.release()
            for handle in files.values():
                handle.close()
