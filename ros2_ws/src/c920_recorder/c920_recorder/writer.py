"""One bounded queue and ordered writer. Original images never receive overlays."""
import json
import queue
import threading
import time
from datetime import datetime
from pathlib import Path
import cv2
from .stop_line import candidates


def dump(path, value):
    path=Path(path)
    temporary=path.with_suffix(path.suffix+'.tmp')
    temporary.write_text(json.dumps(value,indent=2,allow_nan=False)+'\n')
    temporary.replace(path)


class MeasuredQueue(queue.Queue):
    def _init(self,maxsize):
        super()._init(maxsize);self.peak=0

    def _put(self,item):
        super()._put(item);self.peak=max(self.peak,self._qsize())


class Writer:
    def __init__(self,root,metadata,capacity=128):
        if capacity<1:raise ValueError('queue capacity must be positive')
        root=Path(root).expanduser();root.mkdir(parents=True,exist_ok=True)
        # Never overwrite a session started in the same second.
        self.path=root/datetime.now().strftime('%Y%m%d_%H%M%S')
        self.path.mkdir(exist_ok=False)
        self.metadata=dict(metadata,state='recording',started_wall_ns=time.time_ns())
        self.queue=MeasuredQueue(capacity);self.lock=threading.Lock()
        self.errors=[];self.overflow=0;self.count=0;self.closed=False;self.latest_results=[]
        self.max_write_latency_ms=0.;self.total_write_latency_ms=0.
        self.thread=threading.Thread(target=self._run,name='c920-writer')
        dump(self.path/'session.json',self.metadata);self.thread.start()

    def submit(self,frame,meta):
        with self.lock:
            if self.closed or self.errors:return False
            try:self.queue.put_nowait((frame,dict(meta)))
            except queue.Full:
                self.overflow+=1;self.errors.append('writer queue overflow')
                return False
            return True

    def close(self,details):
        with self.lock:self.closed=True
        self.queue.put(None);self.thread.join()
        self.metadata.update(state='closed',stop=details,stream_counts={'rgb':self.count},
            writer_errors=self.errors,queue_overflow=self.overflow,queue_peak=self.queue.peak,
            queue_capacity=self.queue.maxsize,closed_wall_ns=time.time_ns(),
            mean_mp4_write_latency_ms=self.total_write_latency_ms/max(1,self.count),
            max_mp4_write_latency_ms=self.max_write_latency_ms)
        dump(self.path/'session.json',self.metadata)

    def _run(self):
        video=None;sentinel=False
        try:
            with (self.path/'frames.jsonl').open('w') as frames, (self.path/'stop_lines.jsonl').open('w') as stops:
                while True:
                    item=self.queue.get()
                    if item is None:sentinel=True;break
                    frame,meta=item
                    height,width=frame.shape[:2]
                    if frame.shape!=(self.metadata['height'],self.metadata['width'],3):raise ValueError('RGB resolution changed')
                    if video is None:
                        video=cv2.VideoWriter(str(self.path/'color.mp4'),cv2.VideoWriter_fourcc(*'mp4v'),self.metadata['fps'],(width,height))
                        if not video.isOpened():raise RuntimeError('MP4 writer open failed')
                    start=time.monotonic_ns();video.write(frame);elapsed=(time.monotonic_ns()-start)/1e6
                    self.total_write_latency_ms+=elapsed;self.max_write_latency_ms=max(self.max_write_latency_ms,elapsed)
                    index=self.count
                    frames.write(json.dumps(dict(meta,index=index))+'\n');self.count+=1
                    boxes=candidates(frame);self.latest_results=boxes
                    stops.write(json.dumps(dict(index=index,timestamp_ns=meta['timestamp_ns'],
                        candidates=boxes,distance_available=False,detection_status='rgb_candidates'))+'\n')
        except Exception as exc:
            self.errors.append(f'writer error: {type(exc).__name__}: {exc}')
        finally:
            if video is not None:video.release()
            # Drain after failure so finalization never blocks on a full queue.
            if not sentinel:
                while self.queue.get() is not None:pass
