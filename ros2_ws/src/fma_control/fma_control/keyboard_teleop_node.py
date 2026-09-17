"""Terminal manual-command source; +/-0.2 means direction, not speed regulation."""
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
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

from fma_interfaces.msg import DriveCommand, VehicleFeedback


@dataclass
class TeleopState:
    drive: str = 'STOP'
    steering: float = 0.0
    quit_requested: bool = False

    pwm_percent: int = 0
    pwm_step: int = 5
    pwm_max: int = 50

    def __post_init__(self):
        if type(self.pwm_step) is not int or not 1 <= self.pwm_step <= 100:
            raise ValueError('manual_pwm_step_percent must be an integer in 1..100')
        if type(self.pwm_max) is not int or not 0 <= self.pwm_max <= 50:
            raise ValueError('manual_pwm_max_percent must be an integer in 0..50')

    def handle_key(self, key):
        if self.quit_requested:
            return
        key = key.lower()
        if key in ('w', 's'):
            requested = 'FORWARD' if key == 'w' else 'REVERSE'
            if self.drive not in ('STOP', requested):
                self.pwm_percent = max(0, self.pwm_percent - self.pwm_step)
                if self.pwm_percent == 0:
                    self.drive = 'STOP'
            else:
                self.pwm_percent = min(self.pwm_max, self.pwm_percent + self.pwm_step)
                self.drive = requested if self.pwm_percent else 'STOP'
        elif key == 'a':
            self.steering = min(0.2810, self.steering + 0.05)
        elif key == 'd':
            self.steering = max(-0.3054, self.steering - 0.05)
        elif key == 'c':
            self.steering = 0.0
        elif key in (' ', 'x', 'q', '\x03'):
            self.drive = 'STOP'
            self.pwm_percent = 0
            self.quit_requested = key in ('q', '\x03')

    @property
    def speed_mps(self):
        # The controller uses the sign only; actual speed comes from feedback.
        return {'STOP': 0.0, 'FORWARD': 0.2, 'REVERSE': -0.2}[self.drive]


def speed_display(speed_mps):
    return f'{abs(speed_mps) * 3.6:.2f} km/h'


class KeyboardTeleopNode(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')
        self.state = TeleopState(
            pwm_step=self.declare_parameter('manual_pwm_step_percent', 5,
                                           ParameterDescriptor(read_only=True)).value,
            pwm_max=self.declare_parameter('manual_pwm_max_percent', 50,
                                          ParameterDescriptor(read_only=True)).value)
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
        msg.use_pwm_override = True
        msg.drive_pwm_percent = self.state.pwm_percent
        self.publisher.publish(msg)

    def handle_key(self, key):
        self.state.handle_key(key)
        if key.lower() in ('w', 's', 'a', 'd', 'c', ' ', 'x', 'q', '\x03'):
            self.publish_command()  # STOP need not wait for the 10 Hz timer.

    def stop_repeatedly(self):
        self.state.drive = 'STOP'
        self.state.pwm_percent = 0
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
            'W: 전진 가속/후진 감속  S: 전진 감속/후진 가속  A: 좌조향  D: 우조향\n'
            'C: 중앙  SPACE/X: 정지  Q/Ctrl+C: 정지 후 종료\n'
            '다른 /cmd/manual publisher를 실행하지 마세요. 종료 후 timeout이면 자율주행이 재개될 수 있습니다.\n'
            'STOP 상태에서도 A/D/C 조향 가능. 반대 방향 키는 PWM 감속 → STOP → 다음 입력에 방향 전환.\n'
            '+/-0.2 명령은 방향 표시이며 실제 속도 목표가 아닙니다.\n'
            f'DRIVE TARGET : {self.state.drive}\n'
            f'PWM   : {self.state.pwm_percent} % (실제 속도와 다름)\n'
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
