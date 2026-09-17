#!/usr/bin/env python3
"""Receive-only NMEA course recorder. Rows are individual GGA/RMC observations.

Missing fields remain empty; GGA and RMC are never merged across GPS epochs.
Counts/distance use GGA only (RMC fallback if no GGA was received).
Raw files contain the exact received bytes, including malformed sentences.
Optional markers: type a digit followed by Enter; GPS reception is independent.
"""
import argparse
import csv
from datetime import datetime
import math
import os
from pathlib import Path
import signal
import sys
import termios
import threading
import time

import serial

FIELDS = ['local_timestamp', 'gps_utc', 'latitude_deg', 'longitude_deg',
          'altitude_m', 'fix_valid', 'fix_quality', 'satellites', 'hdop',
          'speed_mps', 'course_deg', 'raw_sentence_type']
MARKERS = dict(zip('123456789', ['START', 'STOP_LINE', 'S_CURVE',
    'TRAFFIC_LIGHT', 'PERPENDICULAR_PARKING', 'EMERGENCY_ZONE',
    'PARALLEL_PARKING', 'LANE_CONTROL', 'FINISH']))


def stamp():
    return datetime.now().astimezone().isoformat(timespec='milliseconds')


def number(value):
    result = float(value)
    if not math.isfinite(result):
        raise ValueError('nonfinite value')
    return result


def coordinate(value, hemi, latitude):
    if not value or not hemi:
        return ''
    if hemi not in ('NS' if latitude else 'EW'):
        raise ValueError('hemisphere')
    v = number(value)
    degrees, minutes = divmod(v, 100)
    result = degrees + minutes / 60
    if v < 0 or minutes >= 60 or result > (90 if latitude else 180):
        raise ValueError('coordinate range')
    return -result if hemi in 'SW' else result


def parse(line, timestamp):
    try:
        text = line.decode('ascii').strip()
        if not text.startswith('$'):
            return None
        body, checksum = text[1:].split('*')
        actual = 0
        for char in body:
            actual ^= ord(char)
        if len(checksum) != 2 or actual != int(checksum, 16):
            return None
        p = body.split(',')
        kind = p[0][-3:]
        if kind not in ('GGA', 'RMC'):
            return None
        row = dict.fromkeys(FIELDS, '')
        row.update(local_timestamp=timestamp, gps_utc=p[1], raw_sentence_type=p[0])
        if kind == 'GGA':
            row['latitude_deg'] = coordinate(p[2], p[3], True)
            row['longitude_deg'] = coordinate(p[4], p[5], False)
            row['fix_quality'] = int(p[6]) if p[6] else ''
            # 6=estimated, 7=manual, 8=simulation are not live GPS fixes.
            row['fix_valid'] = int(row['fix_quality'] in (1, 2, 3, 4, 5)) if p[6] else ''
            row['satellites'] = int(p[7]) if p[7] else ''
            row['hdop'] = number(p[8]) if p[8] else ''
            row['altitude_m'] = number(p[9]) if p[9] and p[10] == 'M' else ''
        else:
            row['latitude_deg'] = coordinate(p[3], p[4], True)
            row['longitude_deg'] = coordinate(p[5], p[6], False)
            row['fix_valid'] = int(p[2] == 'A' and (len(p) <= 12 or p[12] not in ('N', 'E', 'M', 'S'))) if p[2] in ('A', 'V') else ''
            row['speed_mps'] = number(p[7]) * 1852 / 3600 if p[7] else ''
            row['course_deg'] = number(p[8]) if p[8] else ''
            if p[9] and p[1]:
                date = datetime.strptime(p[9], '%d%m%y').date().isoformat()
                row['gps_utc'] = f'{date}T{p[1][:2]}:{p[1][2:4]}:{p[1][4:]}Z'
        return row
    except (ValueError, IndexError, UnicodeError):
        return None


def valid(row):
    return row.get('fix_valid') == 1 and row.get('latitude_deg', '') != '' and row.get('longitude_deg', '') != ''


def haversine(a, b):
    la, lb = math.radians(a[0]), math.radians(b[0])
    dlat, dlon = lb - la, math.radians(b[1] - a[1])
    h = math.sin(dlat / 2)**2 + math.cos(la) * math.cos(lb) * math.sin(dlon / 2)**2
    return 6371000 * 2 * math.asin(math.sqrt(min(1, h)))


class Stats:
    def __init__(self):
        self.good = self.bad = self.count = 0
        self.start = self.end = self.previous = None
        self.distance = self.hdop_sum = self.hdop_count = 0
        self.sat_min = self.sat_max = None

    def add(self, row):
        self.count += 1
        if valid(row):
            self.good += 1
            point = (row['latitude_deg'], row['longitude_deg'])
            if self.previous is not None:
                self.distance += haversine(self.previous, point)
            if self.start is None:
                self.start = point
            self.end = self.previous = point
        else:
            self.bad += 1
            self.previous = None
        if row['hdop'] != '':
            self.hdop_sum += row['hdop']
            self.hdop_count += 1
        if row['satellites'] != '':
            v = row['satellites']
            self.sat_min = v if self.sat_min is None else min(v, self.sat_min)
            self.sat_max = v if self.sat_max is None else max(v, self.sat_max)


def current_baud(port):
    fd = os.open(port, os.O_RDONLY | os.O_NOCTTY | os.O_NONBLOCK)
    try:
        speed = termios.tcgetattr(fd)[4]
    finally:
        os.close(fd)
    for name in dir(termios):
        if name.startswith('B') and name[1:].isdigit() and getattr(termios, name) == speed:
            if int(name[1:]) > 0:
                return int(name[1:])
    raise ValueError('Cannot read current baud; supply a verified --baud value')


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--port', choices=['/dev/ttyACM1'], default='/dev/ttyACM1')
    ap.add_argument('--baud', type=int, help='Verified host baud; default reads existing tty setting')
    ap.add_argument('--duration', type=float, default=0, help='Seconds; 0 records until Ctrl+C')
    ap.add_argument('--markers', action='store_true', help='Digit + Enter records optional marker')
    args = ap.parse_args()
    if args.duration < 0 or (args.baud is not None and args.baud <= 0):
        ap.error('duration must be nonnegative and baud positive')
    stop = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    started = time.monotonic()
    files, paths = [], []
    receiver = None
    stats = {'GGA': Stats(), 'RMC': Stats()}
    latest = {}
    lock = threading.Lock()
    exit_code = 0
    try:
        baud = args.baud if args.baud is not None else current_baud(args.port)
        receiver = serial.Serial(port=None, baudrate=baud, timeout=0.1, exclusive=True)
        receiver.dtr = False
        receiver.rts = False
        receiver.port = args.port
        receiver.open()
        folder = Path.home() / 'fma_gps_logs'
        folder.mkdir(parents=True, exist_ok=True)
        base = folder / datetime.now().strftime('course_%Y%m%d_%H%M%S_%f')
        def create(suffix, binary=False):
            path = Path(str(base) + suffix)
            f = open(path, 'xb' if binary else 'x', **({} if binary else {'newline': '', 'encoding': 'utf-8'}))
            paths.append(path)
            files.append(f)
            return f
        raw = create('_raw.txt', True)
        csv_file = create('.csv')
        writer = csv.DictWriter(csv_file, fieldnames=FIELDS)
        writer.writeheader()
        marker_writer = None
        if args.markers:
            marker_writer = csv.DictWriter(create('_markers.csv'), fieldnames=['marker_timestamp', 'marker', 'gps_age_s'] + FIELDS)
            marker_writer.writeheader()
        print(f'GPS={args.port} | host baud={baud} (protocol verified only after reception)')
        print(f'Logs: {base}*', flush=True)
        if args.markers:
            print('Markers (digit + Enter): ' + ' | '.join(f'{k} {v}' for k, v in MARKERS.items()), flush=True)
        # Terminal output/input run in a daemon; a slow terminal never blocks reception.
        def display():
            while not stop.wait(0.25):
                with lock:
                    g = latest.get('GGA') or latest.get('RMC')
                    r = latest.get('RMC')
                    s = stats['GGA'] if stats['GGA'].count else stats['RMC']
                    points = s.good
                row, age = (g[0], time.monotonic() - g[1]) if g else ({}, float('inf'))
                speed = r[0]['speed_mps'] if r and time.monotonic() - r[1] < 2 else ''
                state = 'FIX=OK' if age < 2 and valid(row) else 'NO GPS FIX'
                if age >= 2:
                    state += ' | NO RECENT GGA/RMC'
                print(f"\r{state} | LAT={row.get('latitude_deg', '')} | LON={row.get('longitude_deg', '')} | SAT={row.get('satellites', '')} | HDOP={row.get('hdop', '')} | SPEED={speed} m/s | POINTS={points}    ", end='', flush=True)
        threading.Thread(target=display, daemon=True).start()
        # Marker writer has its own file; serialize with periodic flush via lock.
        def markers():
            for line in sys.stdin:
                if stop.is_set():
                    break
                name = MARKERS.get(line.strip())
                if name:
                    with lock:
                        if stop.is_set():
                            break
                        g = latest.get('GGA') or latest.get('RMC')
                        age = time.monotonic() - g[1] if g else ''
                        row = dict(g[0]) if g and age < 2 else {}
                        marker_writer.writerow(dict(row, marker_timestamp=stamp(), marker=name, gps_age_s=age))
                        files[-1].flush()
        if args.markers:
            threading.Thread(target=markers, daemon=True).start()
        buffer = b''
        synced = time.monotonic()
        while not stop.is_set() and (not args.duration or time.monotonic() - started < args.duration):
            chunk = receiver.read(min(max(receiver.in_waiting, 1), 65536))
            if chunk:
                raw.write(chunk)
                buffer += chunk
                while b'\n' in buffer:
                    line, buffer = buffer.split(b'\n', 1)
                    row = parse(line, stamp())
                    if row:
                        writer.writerow(row)
                        kind = row['raw_sentence_type'][-3:]
                        with lock:
                            stats[kind].add(row)
                            latest[kind] = (row, time.monotonic())
                if len(buffer) > 65536:
                    buffer = buffer[-4096:]
            if time.monotonic() - synced >= 1:
                with lock:
                    for f in files:
                        f.flush()
                        os.fsync(f.fileno())
                synced = time.monotonic()
    except Exception as exc:
        print(f'\nERROR: {exc}', file=sys.stderr)
        exit_code = 1
    finally:
        stop.set()
        if receiver is not None:
            try:
                receiver.close()
            except Exception as exc:
                print(f'\nSerial close error: {exc}', file=sys.stderr)
                exit_code = 1
        with lock:
            for f in files:
                try:
                    f.flush()
                    os.fsync(f.fileno())
                except OSError as exc:
                    print(f'\nSave error: {exc}', file=sys.stderr)
                    exit_code = 1
                finally:
                    f.close()
        s = stats['GGA'] if stats['GGA'].count else stats['RMC']
        print('\nLog files: ' + (', '.join(map(str, paths)) or '(none)'))
        print(f'Duration: {time.monotonic() - started:.1f} s | Valid points: {s.good} | Invalid/unknown points: {s.bad}')
        print(f'Start: {s.start} | End: {s.end} | Approx. distance: {s.distance:.2f} m')
        print(f'Average HDOP: {s.hdop_sum / s.hdop_count if s.hdop_count else "N/A"} | Satellites min/max: {s.sat_min}/{s.sat_max}')
        print(f'Checksum-valid sentences: GGA={stats["GGA"].count}, RMC={stats["RMC"].count}')
        print('Counts/distance: GGA observations (RMC fallback); distance includes GPS drift, excludes gaps with invalid fixes.')
        if not s.count:
            print('NOT READY: no checksum-valid GGA/RMC received.')
            exit_code = 1
    return exit_code


if __name__ == '__main__':
    sys.exit(main())
