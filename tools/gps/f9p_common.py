"""Validated F9P settings and bounded UBX I/O; no hardware access on import."""
import math
import time

from pyubx2 import UBXReader

COMMON = {
    'CFG_RATE_MEAS': 1000,
    'CFG_RATE_NAV': 1,
    'CFG_SIGNAL_GLO_ENA': 0,
    'CFG_SIGNAL_GLO_L1_ENA': 0,
    'CFG_SIGNAL_GLO_L2_ENA': 0,
    'CFG_SIGNAL_BDS_ENA': 1,
    'CFG_SIGNAL_BDS_B1_ENA': 1,
    'CFG_SIGNAL_BDS_B2_ENA': 1,
}
REAR = {
    **COMMON,
    'CFG_UART2_BAUDRATE': 38400,
    'CFG_UART2OUTPROT_UBX': 0,
    'CFG_UART2OUTPROT_NMEA': 0,
    'CFG_UART2OUTPROT_RTCM3X': 1,
    'CFG_MSGOUT_RTCM_3X_TYPE4072_0_UART2': 1,
    'CFG_MSGOUT_RTCM_3X_TYPE1074_UART2': 1,
    'CFG_MSGOUT_RTCM_3X_TYPE1094_UART2': 1,
    'CFG_MSGOUT_RTCM_3X_TYPE1124_UART2': 1,
}
FRONT = {
    **COMMON,
    'CFG_UART1_BAUDRATE': 38400,
    'CFG_UART1INPROT_RTCM3X': 1,
    'CFG_NAVHPG_DGNSSMODE': 3,
    'CFG_USBOUTPROT_UBX': 1,
    'CFG_MSGOUT_UBX_NAV_RELPOSNED_USB': 1,
    'CFG_MSGOUT_UBX_RXM_RTCM_USB': 1,
}
RTCM_TYPES = (4072, 1074, 1094, 1124)


def positive(value):
    number = float(value)
    if not math.isfinite(number) or number <= 0:
        raise ValueError('must be finite and positive')
    return number


class UBXStream:
    """Incremental bounded framing; timeouts cannot hang on NMEA/noisy input."""
    def __init__(self, port):
        self.port = port
        self.buffer = bytearray()
        self.errors = 0

    def read(self):
        self.buffer.extend(self.port.read(256))
        messages = []
        while self.buffer:
            start = self.buffer.find(b'\xb5\x62')
            if start < 0:
                self.buffer[:] = self.buffer[-1:] if self.buffer[-1:] == b'\xb5' else b''
                break
            del self.buffer[:start]
            if len(self.buffer) < 6:
                break
            size = int.from_bytes(self.buffer[4:6], 'little') + 8
            if size > 4096:
                self.errors += 1
                del self.buffer[:2]
                continue
            if len(self.buffer) < size:
                break
            raw = bytes(self.buffer[:size])
            try:
                msg = UBXReader.parse(raw)
            except Exception:
                self.errors += 1
                del self.buffer[:2]
                continue
            del self.buffer[:size]
            messages.append(msg)
        return messages

    def exchange(self, packet, accept, timeout=3.0):
        # Only configuration/polling callers write; passive diagnostics never call this.
        self.port.reset_input_buffer()
        self.buffer.clear()
        raw = packet.serialize()
        if self.port.write(raw) != len(raw):
            raise RuntimeError('Partial UBX write; configuration may be incomplete')
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            for msg in self.read():
                if accept(msg):
                    return msg
        raise TimeoutError(f'No response to {packet.identity}; do not assume success')


def heading_status(msg, baseline=0.92, tolerance=0.05):
    """pyubx2 combines HP length in cm, and scales heading/accuracy to degrees."""
    if msg.identity != 'NAV-RELPOSNED' or msg.version != 1 or len(msg.payload) != 64:
        raise ValueError('Expected 64-byte NAV-RELPOSNED version 1')
    length = msg.relPosLength / 100.0
    result = {
        'relPosLength_m': length,
        'heading_deg': msg.relPosHeading % 360,
        'carrSoln': msg.carrSoln,
        'relPosValid': msg.relPosValid,
        'isMoving': msg.isMoving,
        'relPosHeadingValid': msg.relPosHeadingValid,
        'accHeading_deg': msg.accHeading,
        'baseline_error_m': abs(length - baseline),
    }
    reasons = []
    for field, expected in [('carrSoln', 2), ('relPosValid', 1),
                            ('isMoving', 1), ('relPosHeadingValid', 1)]:
        if result[field] != expected:
            reasons.append(f'{field}!={expected}')
    if result['baseline_error_m'] > tolerance + 1e-12:
        reasons.append('baseline_out_of_tolerance')
    result['pass'] = not reasons
    result['reasons'] = reasons
    # Float/raw heading remains visible diagnostically, never exposed as valid.
    result['valid_heading_deg'] = result['heading_deg'] if result['pass'] else None
    return result


class HeadingMonitor:
    def __init__(self, baseline, tolerance, stale_seconds):
        self.baseline, self.tolerance, self.stale_seconds = baseline, tolerance, stale_seconds
        self.latest = None
        self.received_at = None
        self.itow = None

    def update(self, msg, now):
        if msg.identity != 'NAV-RELPOSNED':
            return
        status = heading_status(msg, self.baseline, self.tolerance)
        if msg.iTOW == self.itow:
            return  # Repeated navigation epoch must not extend freshness.
        self.latest, self.received_at, self.itow = status, now, msg.iTOW

    def snapshot(self, now):
        if self.latest is None:
            return {'pass': False, 'reasons': ['no_RELPOSNED'], 'valid_heading_deg': None}
        result = dict(self.latest)
        result['reasons'] = list(self.latest['reasons'])
        result['age_s'] = now - self.received_at
        if result['age_s'] >= self.stale_seconds:
            result.update({'pass': False, 'valid_heading_deg': None})
            result['reasons'].append('stale_RELPOSNED')
        return result


class RTCMCounter:
    def __init__(self):
        self.counts = {kind: {'received': 0, 'used': 0, 'not_used': 0, 'unknown': 0}
                       for kind in RTCM_TYPES}
        self.crc_failed = 0
        self.other = 0
        self.subtypes_4072 = {}

    def update(self, msg):
        if msg.identity != 'RXM-RTCM':
            return
        if msg.crcFailed:
            # Per protocol, msgType/subType may themselves be corrupt.
            self.crc_failed += 1
            return
        if msg.msgType not in self.counts:
            self.other += 1
            return
        row = self.counts[msg.msgType]
        row['received'] += 1
        row[{1: 'not_used', 2: 'used'}.get(msg.msgUsed, 'unknown')] += 1
        if msg.msgType == 4072:
            self.subtypes_4072[msg.subType] = self.subtypes_4072.get(msg.subType, 0) + 1

    def snapshot(self):
        return {'types': self.counts, 'crc_failed_unattributed': self.crc_failed,
                'other_types': self.other, '4072_subtypes': self.subtypes_4072}
