"""Mission-conditioned dummy avoidance state machine.

S_CURVE:
    DETECT -> AVOID_OUT -> PASS_STRAIGHT -> AVOID_RETURN -> WAIT_CLEAR

CHILD_DUMMY:
    DETECT -> STOP_HOLD -> AVOID_OUT -> PASS_STRAIGHT -> AVOID_RETURN -> WAIT_CLEAR

Waypoint / MissionState selects the behavior mode.
D435i obstacle position selects the pass side.
Encoder travel controls the avoidance trajectory length.
"""

from dataclasses import dataclass
from enum import Enum

from fma_interfaces.msg import MissionState


class ManeuverMode(Enum):
    NONE = 'NONE'
    AVOID_ONLY = 'AVOID_ONLY'
    STOP_THEN_AVOID = 'STOP_THEN_AVOID'


class ManeuverPhase(Enum):
    IDLE = 'IDLE'
    STOP_HOLD = 'STOP_HOLD'
    AVOID_OUT = 'AVOID_OUT'
    PASS_STRAIGHT = 'PASS_STRAIGHT'
    AVOID_RETURN = 'AVOID_RETURN'
    WAIT_CLEAR = 'WAIT_CLEAR'


class PassSide(Enum):
    # REP-103: + steering = left
    LEFT = 1
    RIGHT = -1


@dataclass(frozen=True)
class ObstacleSample:
    distance_m: float
    y_m: float
    confidence: float
    in_path: bool = True


@dataclass(frozen=True)
class ManeuverConfig:
    # S_CURVE avoidance trigger.
    trigger_distance_m: float = 2.0

    # CHILD_DUMMY:
    # Detecting it far away is OK, but STOP only when it is close.
    child_trigger_distance_m: float = 2.0

    # Once CHILD_DUMMY has triggered, keep considering an in-path
    # obstacle dangerous out to this distance until it is moved aside.
    child_clear_distance_m: float = 3.0
    child_clear_hold_sec: float = 2.0

    min_confidence: float = 0.5

    # Object near vehicle center uses preferred side.
    center_deadband_m: float = 0.15
    preferred_center_side: PassSide = PassSide.RIGHT

    # Initial field-test values. Tune on the vehicle.
    steering_rad: float = 0.20
    drive_pwm: int = 120

    # CHILD_DUMMY full-stop hold before avoidance.
    stop_hold_sec: float = 3.0

    # Avoid sideways, pass the obstacle completely, then return.
    avoid_out_distance_m: float = 1.00
    avoid_out_hold_sec: float = 1.0
    pass_straight_distance_m: float = 1.30
    avoid_return_distance_m: float = 1.00

    # Firmware calibration:
    # wheel circumference ~= 0.880 m
    # encoder ~= 371.2 counts/revolution
    encoder_m_per_count: float = 0.880 / 371.2

    # Require clear perception before allowing another trigger.
    clear_frames: int = 5


@dataclass(frozen=True)
class ManeuverOutput:
    phase: ManeuverPhase
    command_active: bool
    stop: bool
    steering_rad: float
    drive_pwm: int
    pass_side: PassSide | None


def mode_for_mission(mission: int, active: bool) -> ManeuverMode:
    if not active:
        return ManeuverMode.NONE

    if mission == MissionState.S_CURVE:
        return ManeuverMode.AVOID_ONLY

    if mission == MissionState.CHILD_DUMMY:
        return ManeuverMode.STOP_THEN_AVOID

    return ManeuverMode.NONE


def obstacle_is_valid(
    obstacle: ObstacleSample | None,
    config: ManeuverConfig,
    max_distance_m: float | None = None,
) -> bool:
    if obstacle is None:
        return False

    distance_limit = (
        config.trigger_distance_m
        if max_distance_m is None
        else max_distance_m
    )

    return (
        obstacle.in_path
        and obstacle.confidence >= config.min_confidence
        and 0.0 < obstacle.distance_m <= distance_limit
    )


def choose_pass_side(
    y_m: float,
    config: ManeuverConfig,
) -> PassSide:
    # Obstacle on LEFT -> pass RIGHT.
    if y_m > config.center_deadband_m:
        return PassSide.RIGHT

    # Obstacle on RIGHT -> pass LEFT.
    if y_m < -config.center_deadband_m:
        return PassSide.LEFT

    # Nearly centered obstacle: deterministic configurable preference.
    return config.preferred_center_side


class DummyManeuverController:
    def __init__(self, config: ManeuverConfig | None = None):
        self.config = config or ManeuverConfig()

        self.mode = ManeuverMode.NONE
        self.phase = ManeuverPhase.IDLE
        self.pass_side = None

        self.phase_start_time = None
        self.phase_start_encoder = None
        self.clear_count = 0
        self.clear_start_time = None

    def reset(self):
        self.mode = ManeuverMode.NONE
        self.phase = ManeuverPhase.IDLE
        self.pass_side = None
        self.phase_start_time = None
        self.phase_start_encoder = None
        self.clear_count = 0
        self.clear_start_time = None

    def _enter_phase(
        self,
        phase: ManeuverPhase,
        now_sec: float,
        encoder_count: int | None,
    ):
        self.phase = phase
        self.phase_start_time = now_sec
        self.phase_start_encoder = encoder_count

    def _travelled_m(self, encoder_count: int | None) -> float:
        if (
            encoder_count is None
            or self.phase_start_encoder is None
        ):
            return 0.0

        delta = abs(encoder_count - self.phase_start_encoder)

        return delta * self.config.encoder_m_per_count

    def update(
        self,
        *,
        mission: int,
        mission_active: bool,
        obstacle: ObstacleSample | None,
        now_sec: float,
        encoder_count: int | None,
        vehicle_stopped: bool = False,
    ) -> ManeuverOutput:
        new_mode = mode_for_mission(mission, mission_active)

        if new_mode is ManeuverMode.NONE:
            self.reset()
            return self.output()

        if new_mode != self.mode:
            self.reset()
            self.mode = new_mode

        # CHILD_DUMMY can be visible from far away.  Do not STOP until
        # it enters the close trigger zone.  After STOP has triggered,
        # use a wider distance while waiting for the path to become clear.
        if (
            self.mode is ManeuverMode.STOP_THEN_AVOID
            and self.phase is ManeuverPhase.IDLE
        ):
            obstacle_distance_limit = (
                self.config.child_trigger_distance_m
            )
        elif self.mode is ManeuverMode.STOP_THEN_AVOID:
            obstacle_distance_limit = (
                self.config.child_clear_distance_m
            )
        else:
            obstacle_distance_limit = self.config.trigger_distance_m

        valid_obstacle = obstacle_is_valid(
            obstacle,
            self.config,
            max_distance_m=obstacle_distance_limit,
        )

        if self.phase is ManeuverPhase.IDLE:
            if valid_obstacle:
                if self.mode is ManeuverMode.STOP_THEN_AVOID:
                    # CHILD_DUMMY does not dodge. Stop and wait until
                    # the obstacle is physically moved out of our path.
                    self.pass_side = None
                    self._enter_phase(
                        ManeuverPhase.STOP_HOLD,
                        now_sec,
                        encoder_count,
                    )
                else:
                    # S_CURVE: choose the empty side and avoid.
                    self.pass_side = choose_pass_side(
                        obstacle.y_m,
                        self.config,
                    )
                    self._enter_phase(
                        ManeuverPhase.AVOID_OUT,
                        now_sec,
                        encoder_count,
                    )

        elif self.phase is ManeuverPhase.STOP_HOLD:
            # CHILD_DUMMY:
            # - Keep STOP active for as long as the obstacle blocks the path.
            # - When it disappears / moves out of path, start a 2 s clear timer.
            # - If it reappears during those 2 s, reset the timer.
            # - Only release mission control after 2 continuous clear seconds.
            if valid_obstacle:
                self.clear_start_time = None
            else:
                if self.clear_start_time is None:
                    self.clear_start_time = now_sec

                clear_elapsed = now_sec - self.clear_start_time

                if (
                    vehicle_stopped
                    and clear_elapsed >= self.config.child_clear_hold_sec
                ):
                    self.phase = ManeuverPhase.IDLE
                    self.pass_side = None
                    self.phase_start_time = None
                    self.phase_start_encoder = None
                    self.clear_start_time = None
                    self.clear_count = 0

        elif self.phase is ManeuverPhase.AVOID_OUT:
            # S_CURVE: use encoder travel so the avoidance shape
            # remains consistent when PWM changes.
            if (
                self._travelled_m(encoder_count)
                >= self.config.avoid_out_distance_m
            ):
                self._enter_phase(
                    ManeuverPhase.PASS_STRAIGHT,
                    now_sec,
                    encoder_count,
                )

        elif self.phase is ManeuverPhase.PASS_STRAIGHT:
            if (
                self._travelled_m(encoder_count)
                >= self.config.pass_straight_distance_m
            ):
                self._enter_phase(
                    ManeuverPhase.AVOID_RETURN,
                    now_sec,
                    encoder_count,
                )

        elif self.phase is ManeuverPhase.AVOID_RETURN:
            if (
                self._travelled_m(encoder_count)
                >= self.config.avoid_return_distance_m
            ):
                self._enter_phase(
                    ManeuverPhase.WAIT_CLEAR,
                    now_sec,
                    encoder_count,
                )
                self.clear_count = 0

        elif self.phase is ManeuverPhase.WAIT_CLEAR:
            # Release mission driving command here so the normal
            # driving controller can resume.
            if valid_obstacle:
                self.clear_count = 0
            else:
                self.clear_count += 1

            if self.clear_count >= self.config.clear_frames:
                # Rearmed for another obstacle in the same mission.
                self.phase = ManeuverPhase.IDLE
                self.pass_side = None
                self.clear_count = 0

        return self.output()

    def output(self) -> ManeuverOutput:
        if self.phase is ManeuverPhase.STOP_HOLD:
            return ManeuverOutput(
                phase=self.phase,
                command_active=True,
                stop=True,
                steering_rad=0.0,
                drive_pwm=0,
                pass_side=self.pass_side,
            )

        if self.phase is ManeuverPhase.AVOID_OUT:
            steering = (
                self.config.steering_rad
                * self.pass_side.value
            )

            return ManeuverOutput(
                phase=self.phase,
                command_active=True,
                stop=False,
                steering_rad=steering,
                drive_pwm=self.config.drive_pwm,
                pass_side=self.pass_side,
            )

        if self.phase is ManeuverPhase.PASS_STRAIGHT:
            return ManeuverOutput(
                phase=self.phase,
                command_active=True,
                stop=False,
                steering_rad=0.0,
                drive_pwm=self.config.drive_pwm,
                pass_side=self.pass_side,
            )

        if self.phase is ManeuverPhase.AVOID_RETURN:
            steering = (
                -self.config.steering_rad
                * self.pass_side.value
            )

            return ManeuverOutput(
                phase=self.phase,
                command_active=True,
                stop=False,
                steering_rad=steering,
                drive_pwm=self.config.drive_pwm,
                pass_side=self.pass_side,
            )

        return ManeuverOutput(
            phase=self.phase,
            command_active=False,
            stop=False,
            steering_rad=0.0,
            drive_pwm=0,
            pass_side=self.pass_side,
        )
