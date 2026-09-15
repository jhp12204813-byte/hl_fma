#!/usr/bin/env python3
"""Field diagnostic ONLY; bypasses production control. Wire allowlist: X, Tdddd.

Requires the ROS2 workspace environment for the existing controller imports.
Dry-run never opens serial. Do not run alongside other camera/control tools.
"""
import argparse
from collections import deque
import math
import re
from pathlib import Path
import sys
import threading
import time

import cv2
import serial

ROOT = Path(__file__).resolve().parents[1]
for package in ('fma_vehicle', 'fma_perception', 'fma_control'):
    sys.path.insert(0, str(ROOT / 'ros2_ws/src' / package))
sys.path.insert(0, str(ROOT / 'ros2_ws/src/fma_perception/tools'))
from replay_c920_lane import make_mask
from fma_control.lane_follow_node import FollowConfig, calculate_command, process_lane
from fma_perception.c920_bev import C920BEV
from fma_perception.paper_lane_tracker import PaperLaneConfig, PaperLaneTracker
from fma_vehicle.vehicle_controller_node import ConversionConfig, steering_angle_to_adc

PORT = '/dev/serial/by-id/usb-STMicroelectronics_STM32_STLink_0671FF505055877267173020-if02'
MAX_AGE = .25
COMMAND_PERIOD = .10  # at most 10 Hz steering


class SteeringPort:
    """No generic command API: propulsion bytes cannot be requested."""
    def __init__(self, port, enabled):
        self.serial = None
        if enabled:
            self.serial = serial.Serial(port, 115200, timeout=.1, write_timeout=.1, exclusive=True)
            try:
                self.stop()
            except BaseException:
                self.close()
                raise

    def _write(self, payload):
        if payload != b'X' and re.fullmatch(rb'T[0-9]{4}', payload) is None:
            raise ValueError('Only X and Tdddd are permitted')
        if self.serial is not None and self.serial.write(payload) != len(payload):
            raise IOError('Incomplete steering diagnostic write')

    def stop(self):
        self._write(b'X')

    def steer(self, angle):
        if not math.isfinite(angle):
            self.stop()
            raise ValueError('Nonfinite steering')
        adc = steering_angle_to_adc(angle)
        calibration = ConversionConfig()
        if not min(calibration.steering_right_adc, calibration.steering_left_adc) <= adc <= max(
                calibration.steering_right_adc, calibration.steering_left_adc):
            self.stop()
            raise ValueError('ADC outside existing calibration')
        self._write(f'T{adc:04d}'.encode('ascii'))
        return adc

    def close(self):
        if self.serial is not None:
            try:
                self.stop()
            finally:
                self.serial.close()
                self.serial = None


class Gate:
    def __init__(self):
        self.streak = 0
        self.sequence = 0
        self.stamp = -math.inf
        self.result = {}
        self.state = 'ACQUIRING'

    def update(self, sequence, stamp, result, now):
        # A lost queue entry or an inter-frame timeout breaks continuity.
        if sequence != self.sequence + 1 or stamp - self.stamp >= MAX_AGE:
            self.streak = 0
        self.sequence, self.stamp, self.result = sequence, stamp, result
        if not result.get('valid') or not 0 <= now - stamp < MAX_AGE:
            self.streak = 0
            self.state = 'INVALID'
        else:
            self.streak += 1
            self.state = 'STEERING' if self.streak >= 5 else 'ACQUIRING'

    def fresh(self, now):
        if not 0 <= now - self.stamp < MAX_AGE:
            self.streak = 0
            self.state = 'INVALID'
        return self.state == 'STEERING'


def capture(args, samples, lock, stopping):
    cap = None
    sequence = 0
    try:
        bev = C920BEV(ROOT / 'config/c920_bev_calibration.yaml')
        tracker = PaperLaneTracker(PaperLaneConfig(min_pair_width_px=args.lane_min_width_m / bev.resolution_m))
        config = FollowConfig()  # same gains and adaptive lookahead
        cap = cv2.VideoCapture(args.device, cv2.CAP_V4L2)
        if not cap.isOpened():
            raise RuntimeError('C920 open failed')
        for prop, value in ((cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG')),
                            (cv2.CAP_PROP_FRAME_WIDTH, 640), (cv2.CAP_PROP_FRAME_HEIGHT, 480),
                            (cv2.CAP_PROP_FPS, 30), (cv2.CAP_PROP_BUFFERSIZE, 1)):
            cap.set(prop, value)
        previous = time.monotonic()
        while not stopping.is_set():
            stamp = time.monotonic()
            ok, frame = cap.read()
            if not ok or frame.shape[:2] != (480, 640):
                raise RuntimeError('C920 read failed or frame size mismatch')
            _, mask, _ = make_mask(frame, bev)
            lane, _ = process_lane(tracker, mask)
            result = calculate_command(lane, bev, config)
            now = time.monotonic()
            result['fps'] = 1 / max(now - previous, 1e-9)
            sequence += 1
            with lock:
                samples.append((sequence, stamp, result))
            previous = now
    except Exception as error:
        with lock:
            samples.append((sequence + 1, time.monotonic(), {'valid': False, 'error': str(error)}))
    finally:
        if cap is not None:
            cap.release()


def run(args):
    if not math.isfinite(args.lane_min_width_m) or args.lane_min_width_m <= 0:
        raise ValueError('lane-min-width-m must be finite and positive')
    output = SteeringPort(args.port, args.enable_steering)
    stopping, lock = threading.Event(), threading.Lock()
    samples = deque(maxlen=64)
    worker = None
    gate = Gate()
    last_command = last_log = -math.inf
    try:
        worker = threading.Thread(target=capture, args=(args, samples, lock, stopping), daemon=True)
        worker.start()
        while True:
            now = time.monotonic()
            with lock:
                pending = list(samples)
                samples.clear()
            for sequence, stamp, result in pending:
                gate.update(sequence, stamp, result, now)
                if gate.state == 'INVALID':
                    output.stop()
                if 'error' in result:
                    raise RuntimeError(result['error'])
            allowed = gate.fresh(now)
            if now - last_command >= COMMAND_PERIOD:
                if allowed:
                    output.steer(gate.result['steering_cmd_rad'])
                else:
                    output.stop()
                last_command = now
            if now - last_log >= 1.0:
                result = gate.result
                adc = steering_angle_to_adc(result['steering_cmd_rad']) if allowed else None
                mode = 'STEERING ENABLED' if args.enable_steering else 'DRY-RUN'
                print(f"{mode} STATE={gate.state} lane_valid={bool(result.get('valid')) and gate.state != 'INVALID'} "
                      f"lateral_error_m={result.get('lateral_error_m')} "
                      f"heading_error_deg={result.get('heading_error_deg')} "
                      f"steering_cmd_rad={result.get('steering_cmd_rad') if allowed else None} "
                      f"steering_adc={adc} FPS={result.get('fps', 0):.1f}", flush=True)
                last_log = now
            time.sleep(.02)
    finally:
        stopping.set()
        try:
            output.close()  # X before close, also on Ctrl+C or capture exceptions
        finally:
            if worker is not None and worker.ident is not None:
                worker.join(timeout=1.0)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--device', default='/dev/video14')
    parser.add_argument('--port', default=PORT)
    parser.add_argument('--lane-min-width-m', type=float, default=3.50)
    parser.add_argument('--enable-steering', action='store_true')
    args = parser.parse_args()
    try:
        run(args)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
