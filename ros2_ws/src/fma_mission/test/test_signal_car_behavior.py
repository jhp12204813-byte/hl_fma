from fma_interfaces.msg import LaneSignal

from fma_mission.signal_car_behavior import (
    KEEP,
    LEFT,
    WAIT,
    SignalCarConfig,
    SignalCarVoter,
)


def make_voter():
    return SignalCarVoter(
        SignalCarConfig(
            window_sec=2.0,
            min_valid_samples=4,
            majority_ratio=0.75,
            min_confidence=0.60,
        )
    )


def send(voter, t, states, confidence=0.95):
    left, center, right = states
    return voter.update(
        t,
        left,
        center,
        right,
        confidence,
        confidence,
        confidence,
    )


def test_blinking_keep():
    voter = make_voter()

    good = (
        LaneSignal.CLOSED,
        LaneSignal.OPEN,
        LaneSignal.CLOSED,
    )
    off = (
        LaneSignal.UNKNOWN,
        LaneSignal.UNKNOWN,
        LaneSignal.UNKNOWN,
    )

    assert send(voter, 0.0, good) == WAIT
    assert send(voter, 0.2, off, 0.0) == WAIT
    assert send(voter, 0.4, good) == WAIT
    assert send(voter, 0.6, off, 0.0) == WAIT
    assert send(voter, 0.8, good) == WAIT
    assert send(voter, 1.0, off, 0.0) == WAIT
    assert send(voter, 1.2, good) == KEEP


def test_blinking_left():
    voter = make_voter()

    good = (
        LaneSignal.OPEN,
        LaneSignal.CLOSED,
        LaneSignal.CLOSED,
    )
    off = (
        LaneSignal.UNKNOWN,
        LaneSignal.UNKNOWN,
        LaneSignal.UNKNOWN,
    )

    assert send(voter, 0.0, good) == WAIT
    assert send(voter, 0.2, off, 0.0) == WAIT
    assert send(voter, 0.4, good) == WAIT
    assert send(voter, 0.8, good) == WAIT
    assert send(voter, 1.2, good) == LEFT


def test_right_open_is_invalid():
    voter = make_voter()

    strange = (
        LaneSignal.OPEN,
        LaneSignal.CLOSED,
        LaneSignal.OPEN,
    )

    for i in range(8):
        result = send(voter, i * 0.2, strange)

    assert result == WAIT


def test_all_closed_waits():
    voter = make_voter()

    closed = (
        LaneSignal.CLOSED,
        LaneSignal.CLOSED,
        LaneSignal.CLOSED,
    )

    for i in range(8):
        result = send(voter, i * 0.2, closed)

    assert result == WAIT


def test_low_confidence_waits():
    voter = make_voter()

    left = (
        LaneSignal.OPEN,
        LaneSignal.CLOSED,
        LaneSignal.CLOSED,
    )

    for i in range(8):
        result = send(
            voter,
            i * 0.2,
            left,
            confidence=0.40,
        )

    assert result == WAIT
