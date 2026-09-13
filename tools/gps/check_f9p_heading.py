#!/usr/bin/env python3
"""Passive FRONT USB heading diagnostic; never sends configuration or polls."""
import argparse
import json
import sys
import time

import serial

from f9p_common import HeadingMonitor, RTCMCounter, UBXStream, positive


def run(argv=None, rtcm=False):
    ap = argparse.ArgumentParser(description=__doc__ if not rtcm else 'Count FRONT RXM-RTCM reports; no writes')
    source = ap.add_mutually_exclusive_group(required=True)
    source.add_argument('--front-port', help='Explicit FRONT/Rover USB device')
    source.add_argument('--input', help='Offline recorded UBX/mixed binary stream')
    ap.add_argument('--baudrate', type=int, default=115200)
    ap.add_argument('--duration', type=positive, default=30.0, help='Bounded observation window in seconds')
    ap.add_argument('--interval', type=positive, default=1.0, help='Console report interval in seconds')
    if not rtcm:
        ap.add_argument('--baseline', type=positive, default=.92)
        ap.add_argument('--tolerance', type=positive, default=.05, help='Absolute baseline tolerance in m')
        ap.add_argument('--stale-seconds', type=positive, default=2.5)
    args = ap.parse_args(argv)
    if args.baudrate <= 0:
        raise ValueError('baudrate must be positive')
    monitor = RTCMCounter() if rtcm else HeadingMonitor(args.baseline, args.tolerance, args.stale_seconds)
    port = (open(args.input, 'rb') if args.input else
            serial.Serial(args.front_port, args.baudrate, timeout=.2, exclusive=True))
    with port:
        stream = UBXStream(port)
        start = time.monotonic()
        next_report = start + args.interval
        while time.monotonic() - start < args.duration:
            before = port.tell() if args.input else None
            for msg in stream.read():
                if rtcm:
                    monitor.update(msg)
                else:
                    monitor.update(msg, time.monotonic())
            now = time.monotonic()
            if now >= next_report:
                result = monitor.snapshot() if rtcm else monitor.snapshot(now)
                print(json.dumps({'elapsed_s': round(now - start, 2), **result}, allow_nan=False), flush=True)
                next_report = now + args.interval
            if args.input and port.tell() == before:
                break
        result = monitor.snapshot() if rtcm else monitor.snapshot(time.monotonic())
        passed = (all(row['received'] > 0 for row in monitor.counts.values())
                  and monitor.subtypes_4072.get(0, 0) > 0) if rtcm else result['pass']
        print(json.dumps({'result': 'PASS' if passed else 'FAIL', 'ubx_parse_errors': stream.errors,
                          **result}, allow_nan=False))
        return 0 if passed else 1


if __name__ == '__main__':
    try:
        sys.exit(run())
    except (Exception, KeyboardInterrupt) as error:
        print(f'FAIL: {error}', file=sys.stderr)
        sys.exit(1)
