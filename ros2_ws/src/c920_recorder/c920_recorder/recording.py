"""Image-only ROS subscriber with monotonic watchdog and asynchronous close."""
from collections import deque
from pathlib import Path
import threading
import time
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from rclpy.qos import qos_profile_sensor_data
from .writer import Writer
from .verification import verify


class Recorder:
    def __init__(self,node,root=None,topic='/c920/image_raw',width=640,height=480,fps=30.,capacity=128):
        self.node=node;self.root=Path(root or Path.home()/'c920_recordings_ku')
        self.topic=topic;self.width=width;self.height=height;self.fps=fps;self.capacity=capacity
        self.bridge=CvBridge();self.lock=threading.RLock();self.writer=None;self.path=None
        self.preview=None;self.last_seen=None;self.receive_times=deque(maxlen=120)
        self.result=None;self.finishing=False;self.finalizer=None;self.stop_details=None;self.error=None
        self.subscription=node.create_subscription(Image,topic,self.receive,qos_profile_sensor_data)
        self.timer=node.create_timer(.1,self.poll)

    def ready(self):
        if self.node.count_publishers(self.topic)!=1:return 'Require exactly one RGB publisher'
        if self.preview is None or self.last_seen is None or time.monotonic()-self.last_seen>1:return 'Waiting for fresh RGB'
        if self.preview.shape!=(self.height,self.width,3):return 'RGB resolution differs from configured profile'
        return None

    @property
    def receive_fps(self):
        values=list(self.receive_times)
        return (len(values)-1)/(values[-1]-values[0]) if len(values)>1 and values[-1]>values[0] else 0.

    def start(self):
        with self.lock:
            if self.writer or self.finishing:raise RuntimeError('Recording/finalizing already active')
            reason=self.ready()
            if reason:raise RuntimeError(reason)
            self.writer=Writer(self.root,dict(camera='Logitech C920',topic=self.topic,width=self.width,
                height=self.height,fps=self.fps,codec='mp4v',overlays_in_video=False),self.capacity)
            self.path=self.writer.path;self.result=None;self.error=None;self.stop_details=None
            self.started=time.monotonic()

    def receive(self,msg):
        now=time.monotonic();mono=time.monotonic_ns();wall=time.time_ns()
        try:frame=self.bridge.imgmsg_to_cv2(msg,'bgr8')
        except Exception as exc:
            self.error=f'RGB conversion error: {exc}'
            self.stop(self.error,automatic=True);return
        with self.lock:
            self.preview=frame;self.last_seen=now;self.receive_times.append(now)
            if self.writer:
                meta=dict(timestamp_ns=msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec,
                    frame_id=msg.header.frame_id,receive_monotonic_ns=mono,receive_wall_ns=wall,encoding=msg.encoding)
                if not self.writer.submit(frame,meta):self.stop('; '.join(self.writer.errors),automatic=True)

    def poll(self):
        with self.lock:
            if not self.writer:return
            age=time.monotonic()-max(self.started,self.last_seen or self.started)
            if age>=3:self.stop(f'RGB timeout, last message {age:.2f}s ago',automatic=True)
            elif self.writer.errors:self.stop('; '.join(self.writer.errors),automatic=True)

    def stop(self,reason='Manual stop',automatic=False):
        with self.lock:
            if self.writer is None:return
            writer=self.writer;self.writer=None;self.finishing=True
            details=dict(automatic=automatic,reason=reason,message=('AUTO STOP: ' if automatic else 'STOP: ')+reason,
                         monotonic_ns=time.monotonic_ns(),wall_ns=time.time_ns())
            self.stop_details=details
            def finish():
                try:
                    writer.close(details);self.result=verify(writer.path)
                except Exception as exc:
                    self.error=f'Finalization failed: {exc}'
                    self.result=dict(status='FAIL',failures=[self.error],warnings=[])
                finally:self.finishing=False
            self.finalizer=threading.Thread(target=finish,name='c920-finalize');self.finalizer.start()

    def shutdown(self):
        self.stop('Recorder shutdown')
        if self.finalizer:self.finalizer.join()
        self.node.destroy_timer(self.timer);self.node.destroy_subscription(self.subscription)
