"""Opt-in real-device A/B diagnostic. Ablations intentionally produce incomplete sessions.

Run with ROS sourced, PYTHONNOUSERSITE=1 QT_QPA_PLATFORM=offscreen python3
ab_device.py --root PATH [--seconds 25]. Starts only camera when none exists.
Each case uses a fresh process; no vehicle topics or commands.
"""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time


def cpu(pid):
    try:
        fields = Path(f'/proc/{pid}/stat').read_text().split(') ')[1].split()
        return (int(fields[11]) + int(fields[12])) / os.sysconf('SC_CLK_TCK')
    except OSError:
        return None


def system_cpu():
    values = list(map(int, Path('/proc/stat').read_text().splitlines()[0].split()[1:]))
    return sum(values[:8]), values[3] + values[4]


def trial(args):
    import cv2
    import numpy as np
    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from rclpy.qos_event import SubscriptionEventCallbacks
    from sensor_msgs.msg import Image, Imu
    from realsense2_camera_msgs.msg import Metadata
    from python_qt_binding.QtCore import QObject
    from python_qt_binding.QtWidgets import QApplication
    from d435i_recorder import writer as writer_module, plugin as plugin_module
    from d435i_recorder.recording import TOPICS
    app = QApplication([])
    rclpy.init()
    events = []
    original_create_node = rclpy.create_node
    def create_node(*args_node, **kwargs_node):
        new_node = original_create_node(*args_node, **kwargs_node)
        original_sub = new_node.create_subscription
        def subscription(*a, **kw):
            topic = a[1]
            if args.reliable_images and a[0] is Image:
                a = list(a)
                a[3] = QoSProfile(depth=a[3].depth, reliability=ReliabilityPolicy.RELIABLE)
            kw['event_callbacks'] = SubscriptionEventCallbacks(
                message_lost=lambda event: events.append(dict(topic=topic, total=event.total_count)))
            return original_sub(*a, **kw)
        new_node.create_subscription = subscription
        return new_node
    rclpy.create_node = create_node
    node = rclpy.create_node('d435i_ab_' + args.case)
    png_times = []
    original_png = writer_module.write_depth_png
    def png(*a):
        start, start_cpu = time.monotonic(), time.thread_time()
        if args.case != 'png_off':
            original_png(*a)
        png_times.append(((time.monotonic()-start)*1000, (time.thread_time()-start_cpu)*1000))
    writer_module.write_depth_png = png
    if args.case == 'analysis_off':
        writer_module.candidates = plugin_module.candidates = lambda image: []
    original_submit = writer_module.Writer.submit
    def submit(self, kind, data, meta):
        if args.case == 'imu_off' and kind == 'imu':
            return True
        return original_submit(self, kind, data, meta)
    writer_module.Writer.submit = submit
    received = {key: [] for key in ('rgb', 'depth', 'imu', 'device_metadata')}
    metadata_numbers = []
    active = [False]
    def observe(kind, msg):
        if active[0]:
            received[kind].append((msg.header.stamp.sec*10**9+msg.header.stamp.nanosec, time.monotonic_ns()))
            if kind == 'device_metadata':
                try:
                    data = json.loads(msg.json_data)
                    metadata_numbers.append(data.get('frame_number', data.get('frame_counter')))
                except Exception:
                    metadata_numbers.append(None)
    plugin = None
    if args.case == 'camera_only':
        for kind, typ in [('rgb', Image), ('depth', Image), ('imu', Imu), ('device_metadata', Metadata)]:
            node.create_subscription(typ, TOPICS[kind], lambda msg, k=kind: observe(k, msg),
                                     QoSProfile(depth=400 if kind == 'imu' else 30,
                                                reliability=ReliabilityPolicy.BEST_EFFORT))
    else:
        from d435i_recorder.recording import Recorder
        original_receive = Recorder.receive
        def receive(self, kind, msg):
            observe(kind, msg)
            original_receive(self, kind, msg)
        Recorder.receive = receive
        class Context(QObject):
            def __init__(self):
                super().__init__()
                self.node = node
            def add_widget(self, widget):
                widget.show()
        context = Context()
        plugin = plugin_module.RecorderPlugin(context)
        rec = plugin.recorder
        rec.root = args.root
        plugin.autoplay.setChecked(False)
        if args.case == 'preview_off':
            plugin.timer.stop()
    from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
    executor = MultiThreadedExecutor() if args.executor == 'multi' else SingleThreadedExecutor()
    executor.add_node(node)
    quit_spin = threading.Event()
    def spin():
        while not quit_spin.is_set():
            executor.spin_once(timeout_sec=.02)
    spinner = threading.Thread(target=spin)
    spinner.start()
    beat_stop = threading.Event()
    beats = []
    def heartbeat():
        last = time.monotonic()
        while not beat_stop.wait(.005):
            now = time.monotonic()
            if active[0]:
                beats.append((now-last)*1000)
            last = now
    heartbeat_thread = threading.Thread(target=heartbeat)
    heartbeat_thread.start()
    def pump_until(end):
        while time.monotonic() < end:
            app.processEvents()
            time.sleep(.002)
    result = None
    try:
        pump_until(time.monotonic()+5)
        if plugin:
            deadline = time.monotonic()+20
            while rec.ready() and time.monotonic()<deadline:
                pump_until(time.monotonic()+.1)
            rec.start()
            rec.writer.metadata['diagnostic_ablation'] = args.case
        c0, sys0, start_cpu, start = cpu(args.camera_pid), system_cpu(), time.process_time(), time.monotonic()
        active[0] = True
        pump_until(start+args.seconds)
        active[0] = False
        elapsed = time.monotonic()-start
        proc_pct = 100*(time.process_time()-start_cpu)/elapsed
        c1, sys1 = cpu(args.camera_pid), system_cpu()
        if plugin:
            prematurely_stopped = rec.writer is None
            rec.stop(reason=f'A/B {args.case} duration {args.seconds}s reached')
            while rec.finishing:
                pump_until(time.monotonic()+.05)
            result = rec.result
        else:
            prematurely_stopped = False
        stats = {}
        for kind, rows in received.items():
            if len(rows)<2:
                stats[kind] = dict(count=len(rows))
                continue
            stamps = np.array([r[0] for r in rows], dtype=np.int64)
            deltas = np.diff(stamps)/1e9
            stats[kind] = dict(count=len(rows), ros_hz=(len(rows)-1)/((stamps[-1]-stamps[0])/1e9),
                receive_hz=(len(rows)-1)/((rows[-1][1]-rows[0][1])/1e9), max_gap_s=float(deltas.max()),
                estimated_missing_periods=int(np.maximum(np.rint(deltas*30)-1,0).sum()) if kind!='imu' else None)
        numbers = [n for n in metadata_numbers if isinstance(n, (int,float))]
        summary = dict(case=args.case, executor=args.executor, reliable_images=args.reliable_images,
            publisher_qos={k:[str(x.qos_profile.reliability) for x in node.get_publishers_info_by_topic(TOPICS[k])] for k in ['rgb','depth']}, seconds=elapsed, receive=stats, process_cpu_percent=proc_pct,
            camera_cpu_percent=100*(c1-c0)/elapsed if c0 is not None and c1 is not None else None,
            system_busy_percent=100*(1-(sys1[1]-sys0[1])/(sys1[0]-sys0[0])),
            cpu_percent_note='100% is one logical CPU; system busy is whole host',
            png_wall_mean_ms=float(np.mean([v[0] for v in png_times])) if png_times else None,
            png_thread_cpu_mean_ms=float(np.mean([v[1] for v in png_times])) if png_times else None,
            heartbeat_p95_ms=float(np.percentile(beats,95)) if beats else None,
            heartbeat_max_ms=max(beats) if beats else None,
            heartbeat_note='5ms heartbeat delay includes GIL and OS scheduling; not a GIL profiler',
            metadata_frame_number_missing=sum(max(int(b-a)-1,0) for a,b in zip(numbers,numbers[1:])),
            metadata_number_samples=len(numbers), rmw_message_lost_events=events,
            loss_note='Empty RMW event list does not prove no DDS/SDK drops. Timestamp gaps remain reported.',
            stopped_early=prematurely_stopped, session=str(rec.path) if plugin else None, verification=result)
        Path(args.root, args.case+'.json').write_text(json.dumps(summary,indent=2)+'\n')
        print(json.dumps({k:v for k,v in summary.items() if k!='verification'}),flush=True)
    finally:
        active[0] = False
        beat_stop.set(); heartbeat_thread.join()
        quit_spin.set(); spinner.join(); executor.shutdown()
        if plugin:
            plugin.shutdown_plugin(); plugin.widget.close()
        node.destroy_node();rclpy.shutdown()


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--root',required=True)
    parser.add_argument('--seconds',type=float,default=25)
    parser.add_argument('--only-case', choices=['camera_only','preview_off','preview_on','png_off','analysis_off','imu_off'])
    parser.add_argument('--case',choices=['camera_only','preview_off','preview_on','png_off','analysis_off','imu_off'])
    parser.add_argument('--reliable-images', action='store_true')
    parser.add_argument('--executor', choices=['single','multi'], default='multi')
    parser.add_argument('--camera-pid',type=int,default=0)
    args=parser.parse_args()
    Path(args.root).mkdir(parents=True,exist_ok=True)
    if args.case:
        trial(args);return
    import rclpy
    from d435i_recorder.runner import discover,camera_process_running
    rclpy.init();node=rclpy.create_node('d435i_ab_launcher');existing=discover(node);node.destroy_node();rclpy.shutdown()
    camera=None
    try:
        if not existing:
            if camera_process_running():
                raise RuntimeError('Camera process exists outside visible graph; refusing duplicate')
            camera=subprocess.Popen(['ros2','launch','realsense2_camera','rs_launch.py','serial_no:=_142122070689',
                'rgb_camera.color_profile:=640,480,30','depth_module.depth_profile:=640,480,30',
                'enable_accel:=true','enable_gyro:=true','unite_imu_method:=2','enable_sync:=false',
                'align_depth.enable:=true'],start_new_session=True)
        time.sleep(5)
        pids=[]
        for p in Path('/proc').glob('[0-9]*/cmdline'):
            try:
                if Path(os.fsdecode(p.read_bytes().split(b'\0')[0])).name=='realsense2_camera_node':pids.append(int(p.parent.name))
            except OSError:pass
        camera_pid=pids[0] if len(pids)==1 else 0
        for case in ([args.only_case] if args.only_case else ['camera_only','preview_off','preview_on','png_off','analysis_off','imu_off']):
            subprocess.run([sys.executable,__file__,'--root',args.root,'--seconds',str(args.seconds),
                            '--case',case,'--camera-pid',str(camera_pid),'--executor',args.executor] + (['--reliable-images'] if args.reliable_images else []),check=True)
    finally:
        if camera and camera.poll() is None:
            os.killpg(camera.pid,signal.SIGINT);camera.wait(timeout=15)


if __name__=='__main__':main()
