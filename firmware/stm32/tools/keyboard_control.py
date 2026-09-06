#!/usr/bin/env python3
"""Raw keyboard-to-NUCLEO serial bridge for the EncoderTest firmware."""

import argparse
import os
import select
import sys
import termios
import time
import tty

import serial


ARROW_KEYS = {
    b"\x1b[A": b"W",
    b"\x1b[B": b"S",
    b"\x1b[D": b"A",
    b"\x1b[C": b"D",
}
VALID_KEYS = b"wWsSaAdDcCpPxX qQ"


def read_key(stdin_fd: int) -> bytes:
    first = os.read(stdin_fd, 1)
    if first != b"\x1b":
        return first

    sequence = bytearray(first)
    for _ in range(2):
        ready, _, _ = select.select([stdin_fd], [], [], 0.03)
        if not ready:
            break
        sequence.extend(os.read(stdin_fd, 1))
    return ARROW_KEYS.get(bytes(sequence), b"X")


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Control the NUCLEO-F401RE vehicle with raw WASD keys."
    )
    parser.add_argument("port", nargs="?", default="/dev/ttyACM0")
    args = parser.parse_args()

    if not sys.stdin.isatty():
        print("Run this program from an interactive terminal.", file=sys.stderr)
        return 2

    stdin_fd = sys.stdin.fileno()
    old_terminal = termios.tcgetattr(stdin_fd)
    board = None

    try:
        board = serial.Serial(args.port, 115200, timeout=0)
        board.reset_input_buffer()
        board.write(b"X")

        print("W/Up=forward  S/Down=reverse  A/Left=left  D/Right=right")
        print("C=center  Space/X=STOP  P=status  Q=STOP and exit")
        print("Drive and steering stop within 0.7 s if key messages stop.")
        tty.setcbreak(stdin_fd)

        while True:
            readable, _, _ = select.select(
                [stdin_fd, board.fileno()], [], [], 0.05
            )

            if board.fileno() in readable:
                data = board.read(board.in_waiting or 1)
                if data:
                    os.write(sys.stdout.fileno(), data)

            if stdin_fd in readable:
                key = read_key(stdin_fd)
                if key == b"\x03":
                    raise KeyboardInterrupt
                if key in VALID_KEYS:
                    command = key.upper()
                else:
                    command = b"X"
                board.write(command)
                if key in b"qQ":
                    time.sleep(0.1)
                    break
    except serial.SerialException as error:
        print(f"Serial error: {error}", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        pass
    finally:
        if board is not None and board.is_open:
            board.write(b"X")
            time.sleep(0.1)
            board.close()
        termios.tcsetattr(stdin_fd, termios.TCSADRAIN, old_terminal)
        print("\nKeyboard control ended; STOP was sent.")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
