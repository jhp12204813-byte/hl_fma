#!/usr/bin/env python3
"""Replay only the user-validated settings. Default RAM; --save opts into persistence."""
import argparse
from contextlib import ExitStack
import json
import os
import sys

import serial
from pyubx2 import UBXMessage, POLL, TXN_NONE, SET_LAYER_RAM, SET_LAYER_BBR, SET_LAYER_FLASH

from f9p_common import REAR, FRONT, UBXStream

LAYERS = {'ram': 0, 'bbr': 1, 'flash': 2}  # VALGET layer IDs differ from VALSET masks.


def config_packet(settings, save=False):
    layers = SET_LAYER_RAM | SET_LAYER_BBR | SET_LAYER_FLASH if save else SET_LAYER_RAM
    return UBXMessage.config_set(layers, TXN_NONE, list(settings.items()))


def verify(stream, settings, layer):
    response = stream.exchange(
        UBXMessage.config_poll(LAYERS[layer], 0, list(settings)),
        lambda msg: msg.identity == 'CFG-VALGET' and msg.layer == LAYERS[layer])
    mismatches = {key: {'expected': value, 'actual': getattr(response, key, None)}
                  for key, value in settings.items() if getattr(response, key, None) != value}
    if mismatches:
        raise RuntimeError(f'{layer} mismatch: {json.dumps(mismatches)}')


def identify(stream):
    msg = stream.exchange(UBXMessage('MON', 'MON-VER', POLL),
                          lambda response: response.identity == 'MON-VER')
    extensions = [value.decode('ascii', errors='replace').strip('\x00')
                  for name, value in vars(msg).items() if name.startswith('extension_')]
    if not any(value.startswith('MOD=ZED-F9P') for value in extensions):
        raise RuntimeError(f'MON-VER did not identify ZED-F9P: {extensions}')
    return extensions


def apply(stream, settings, save):
    ack = stream.exchange(config_packet(settings, save),
                          lambda msg: msg.identity in ('ACK-ACK', 'ACK-NAK')
                          and msg.clsID == 6 and msg.msgID == 0x8A)
    if ack.identity == 'ACK-NAK':
        raise RuntimeError('Receiver rejected CFG-VALSET; no automatic retry')
    for layer in (('ram', 'bbr', 'flash') if save else ('ram',)):
        verify(stream, settings, layer)


def parser():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--rear-port', required=True)
    ap.add_argument('--front-port', required=True)
    mode = ap.add_mutually_exclusive_group()
    mode.add_argument('--ram-only', action='store_true', help='Default: RAM only')
    mode.add_argument('--save', action='store_true', help='Write RAM+BBR+FLASH explicitly')
    mode.add_argument('--verify-only', action='store_true', help='MON-VER/VALGET polls only; no settings written')
    ap.add_argument('--layer', choices=LAYERS, default='ram', help='Layer for --verify-only')
    ap.add_argument('--baudrate', type=int, default=115200, help='USB serial host rate, not UART link setting')
    return ap


def main(argv=None):
    args = parser().parse_args(argv)
    if os.path.realpath(args.rear_port) == os.path.realpath(args.front_port):
        raise ValueError('Rear and front must be distinct devices')
    if args.baudrate <= 0:
        raise ValueError('baudrate must be positive')
    if args.layer != 'ram' and not args.verify_only:
        raise ValueError('--layer is only for --verify-only')
    with ExitStack() as stack:
        devices = []
        # Open and identify BOTH explicit USB ports before applying either configuration.
        for role, path, settings in [('REAR', args.rear_port, REAR), ('FRONT', args.front_port, FRONT)]:
            port = stack.enter_context(serial.Serial(path, args.baudrate, timeout=.2,
                                                     write_timeout=1, exclusive=True))
            stream = UBXStream(port)
            print(role, path, identify(stream))
            devices.append((role, stream, settings))
        for role, stream, settings in devices:
            if args.verify_only:
                verify(stream, settings, args.layer)
                print(f'{role}: {args.layer} readback PASS (no VALSET)')
            else:
                apply(stream, settings, args.save)
                print(f'{role}: {"RAM+BBR+FLASH" if args.save else "RAM"} ACK/readback PASS')
    return 0


if __name__ == '__main__':
    try:
        sys.exit(main())
    except (Exception, KeyboardInterrupt) as error:
        print(f'FAIL: {error}. A started write may have partially applied; no rollback/retry.', file=sys.stderr)
        sys.exit(1)
