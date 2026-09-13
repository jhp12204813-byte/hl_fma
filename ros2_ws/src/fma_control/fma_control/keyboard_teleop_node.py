"""Terminal duty-percentage manual source; speed sign indicates direction, not regulation."""
from dataclasses import dataclass
import math
import os
import select
import sys
import termios
import time
import tty

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

from fma_interfaces.msg import DriveCommand, VehicleFeedback


@dataclass
class TeleopState:
    drive: str = 'STOP'
    throttle_percent: int = 0
    steering: float = 0.0
    quit_requested: bool = False

    def handle_key(self, key):
        if self.quit_requested:
            return
        key = key.lower()
        if key in ('w', 's'):
            requested = 'FORWARD' if key == 'w' else 'REVERSE'
            if self.throttle_percent == 0:
                self.drive, self.throttle_percent = requested, 5
            elif self.drive == requested:
                self.throttle_percent = min(100, self.throttle_percent + 5)
            else:
                self.throttle_percent = max(0, self.throttle_percent - 5)
                if self.throttle_percent == 0:
                    self.drive = 'STOP'
        elif key == 'a':
            self.steering = min(0.2810, self.steering + 0.05)
        elif key == 'd':
            self.steering = max(-0.3054, self.steering - 0.05)
        elif key == 'c':
            self.steering = 0.0
        elif key in (' ', 'x', 'q', '\x03'):
            self.drive = 'STOP'
            self.throttle_percent = 0
            self.quit_requested = key in ('q', '\x03')

    @property
    def speed_mps(self):
        # The controller uses the sign only; actual speed comes from feedback.
        return {'STOP': 0.0, 'FORWARD': 0.2, 'REVERSE': -0.2}[self.drive]


def throttle_to_ccr(throttle_percent):
    """Convert only at the manual publisher boundary; wire units remain counts.

    Current contract: DRIVE_PWM_MAX = ARR = 799, so one period is 800 counts.
    Round to nearest count and retain the firmware's safe CCR ceiling at 100%.
    """
    percent = max(0, min(100, throttle_percent))
    period_counts = DriveCommand.DRIVE_PWM_MAX + 1
    return min(DriveCommand.DRIVE_PWM_MAX, (percent * period_counts + 50) // 100)


def speed_display(speed_mps):
    return f'{abs(speed_mps) * 3.6:.2f} km/h'


class KeyboardTeleopNode(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')
        self.state = TeleopState()
        self.feedback = None
        self.feedback_received = None
        self.publisher = self.create_publisher(DriveCommand, '/cmd/manual', 1)
        self.subscription = self.create_subscription(
            VehicleFeedback, '/vehicle/feedback', self.on_feedback, 1)
        self.timer = self.create_timer(
            0.1, self.publish_command, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def on_feedback(self, msg):
        self.feedback = msg
        self.feedback_received = time.monotonic()

    def publish_command(self):
        msg = DriveCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = ''
        msg.speed_mps = self.state.speed_mps
        msg.steering_angle_rad = self.state.steering
        msg.emergency_stop = False
        msg.pwm_control = True
        msg.drive_pwm = throttle_to_ccr(self.state.throttle_percent)
        self.publisher.publish(msg)

    def handle_key(self, key):
        self.state.handle_key(key)
        if key.lower() in ('w', 's', 'a', 'd', 'c', ' ', 'x', 'q', '\x03'):
            self.publish_command()  # STOP need not wait for the 10 Hz timer.

    def stop_repeatedly(self):
        self.state.drive = 'STOP'
        self.state.throttle_percent = 0
        self.state.quit_requested = True
        try:
            self.timer.cancel()
        except Exception as error:
            print(f"Timer cancel failed: {error}", file=sys.stderr)
        # Best effort while ROS context is alive. No motion callbacks run here.
        for _ in range(3):
            try:
                self.publish_command()
                time.sleep(0.05)
            except (Exception, KeyboardInterrupt) as error:
                print(f'STOP publish failed: {error}', file=sys.stderr)

    def screen(self):
        feedback = self.feedback
        measured = '--' if feedback is None else {
            0: 'STOP', 1: 'FORWARD', 2: 'REVERSE'}.get(feedback.drive_state, 'UNKNOWN')
        speed = '-- km/h' if feedback is None else speed_display(feedback.speed_mps)
        adc = '--' if feedback is None else str(feedback.steering_adc)
        stale = (self.feedback_received is not None
                 and time.monotonic() - self.feedback_received >= 1.0)
        return (
            '========================================\n'
            ' FMA KEYBOARD TELEOP\n'
            'W: 전진 +5%p / 후진 -5%p  S: 반대  A: 좌조향  D: 우조향\n'
            'C: 중앙  SPACE/X: 정지  Q/Ctrl+C: 정지 후 종료\n'
            '다른 /cmd/manual publisher를 실행하지 마세요. 종료 후 timeout이면 자율주행이 재개될 수 있습니다.\n'
            'STOP 상태에서는 실제 조향이 움직이지 않을 수 있음\n'
            'Throttle은 duty %이며 실제 속도 목표가 아닙니다.\n'
            f'THROTTLE TARGET : {self.state.throttle_percent}%\n'
            f'DRIVE TARGET : {self.state.drive}\n'
            f'STEER TARGET : {self.state.steering:+.4f} rad '
            f'({math.degrees(self.state.steering):+.1f} deg)\n'
            f'DRIVE FEEDBACK : {measured}\n'
            f'SPEED : {speed}\nADC   : {adc}\n'
            f'FEEDBACK : {"STALE" if stale else "--" if feedback is None else "received"}\n'
            '========================================\n')


def run_terminal(node, stream):
    """Own terminal restoration and STOP cleanup even if setup/UI/spin fails."""
    saved = None
    try:
        fd = stream.fileno()
        saved = termios.tcgetattr(fd)
        tty.setraw(fd)
        escape = ''
        last_display = -math.inf
        while rclpy.ok() and not node.state.quit_requested:
            if select.select([fd], [], [], 0)[0]:
                data = os.read(fd, 1)
                if not data:
                    raise EOFError('Terminal input closed')
                key = data.decode('ascii', errors='ignore')
                # Ignore ANSI arrow/function-key sequences, including uppercase A/D.
                if key == '\x03':
                    node.handle_key(key)
                elif escape:
                    if escape == '\x1b' and key in ('[', 'O'):
                        escape += key
                    elif key and ('@' <= key <= '~'):
                        escape = ''
                elif key == '\x1b':
                    escape = key
                else:
                    node.handle_key(key)
            if node.state.quit_requested:
                break
            rclpy.spin_once(node, timeout_sec=0.02)
            if time.monotonic() - last_display >= 0.1:
                # Raw mode disables newline translation; use CRLF for display.
                print('\x1b[2J\x1b[H' + node.screen().replace('\n', '\r\n'), end='', flush=True)
                last_display = time.monotonic()
    finally:
        try:
            node.stop_repeatedly()
        finally:
            if saved is not None:
                termios.tcsetattr(fd, termios.TCSANOW, saved)


def main(args=None):
    if not sys.stdin.isatty():
        print('keyboard_teleop requires an interactive TTY terminal.', file=sys.stderr)
        return 1
    node = None
    # Keep Python Ctrl+C handling; do not shut ROS down before STOP cleanup.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    try:
        node = KeyboardTeleopNode()
        run_terminal(node, sys.stdin)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as error:
        print(f'keyboard_teleop stopped: {error}', file=sys.stderr)
        return 1
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0
