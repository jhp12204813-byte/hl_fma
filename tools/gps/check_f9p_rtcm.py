#!/usr/bin/env python3
"""Passive count of FRONT receiver RXM-RTCM reports, not USB RTCM output."""
import sys

from check_f9p_heading import run


if __name__ == '__main__':
    try:
        sys.exit(run(rtcm=True))
    except (Exception, KeyboardInterrupt) as error:
        print(f'FAIL: {error}', file=sys.stderr)
        sys.exit(1)
