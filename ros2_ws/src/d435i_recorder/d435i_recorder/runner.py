"""Guarded launcher, optionally with camera, and headless hardware smoke recorder."""
import argparse
import fcntl
import os
import signal
import subprocess
import time
from pathlib import Path
import rclpy
from .recording import Recorder, TOPICS


def camera_process_running():
    for path in Path('/proc').glob('[0-9]*/cmdline'):
        try:
            argv = path.read_bytes().split(b'\0')
            if argv and Path(os.fsdecode(argv[0])).name == 'realsense2_camera_node':
                return True
            if b'launch' in argv and b'realsense2_camera' in argv:
                return True
        except (OSError, ValueError):
            continue
    return False


def discover(node, seconds=2.):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=.1)
    nodes = node.get_node_names_and_namespaces()
    return any(name == 'camera' and namespace == '/camera' for name, namespace in nodes) or any(
        node.count_publishers(topic) for topic in TOPICS.values())


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--with-camera', action='store_true')
    parser.add_argument('--seconds', type=float, help='Headless recording duration; omit for rqt')
    parser.add_argument('--root', default=str(Path.home() / 'd435i_recordings_ku'), help='Headless output root')
    options = parser.parse_args(args)
    if options.seconds is not None and options.seconds <= 0:
        parser.error('--seconds must be positive')
    rclpy.init(args=[])
    node = rclpy.create_node('d435i_recorder_launcher')
    camera = gui = rec = None
    lock = open('/home/idp2/.d435i_recorder_camera.lock', 'a')
    try:
        # Cooperating launchers cannot race each other through discovery.
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        existing = discover(node)
        if options.with_camera and existing:
            print('Existing camera detected: reusing it; no duplicate driver started.', flush=True)
        elif options.with_camera:
            if camera_process_running():
                raise RuntimeError('RealSense process already exists but is not visible in this ROS graph; refusing duplicate driver')
            command = ['ros2', 'launch', 'realsense2_camera', 'rs_launch.py',
                       'serial_no:=_142122070689', 'rgb_camera.color_profile:=640,480,30',
                       'depth_module.depth_profile:=640,480,30', 'enable_accel:=true',
                       'enable_gyro:=true', 'unite_imu_method:=2', 'enable_sync:=false',
                       'align_depth.enable:=true']
            camera = subprocess.Popen(command, start_new_session=True)
        elif not existing:
            raise RuntimeError('No existing camera. Use --with-camera to start one.')
        if options.seconds is None:
            gui = subprocess.Popen(['rqt', '--force-discover', '--standalone', 'd435i_recorder.plugin.RecorderPlugin'],
                                   start_new_session=True)
            if gui.wait() != 0:
                raise RuntimeError('rqt exited with an error')
        else:
            rec = Recorder(node, options.root)
            deadline = time.monotonic() + 25
            while rec.ready() and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.1)
            rec.start()
            print('Recording: ' + str(rec.path), flush=True)
            deadline = time.monotonic() + options.seconds
            while rec.writer and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.05)
            rec.stop(reason=f'Scheduled duration {options.seconds:g}s reached')
            rec.finalizer.join()
            print(rec.result, flush=True)
            if rec.result['status'] == 'FAIL':
                raise RuntimeError('Recording verification FAIL')
    except KeyboardInterrupt:
        pass
    finally:
        if rec:
            rec.shutdown()
        for process in (gui, camera):
            if process and process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGTERM)
                    process.wait(timeout=5)
        lock.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
