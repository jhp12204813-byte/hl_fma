"""Safely interpret stable sign-panel states without generating controls."""

from dataclasses import dataclass

try:
    from . import config
except ImportError:
    import config


@dataclass(frozen=True)
class SignBoardDecision:
    lane_change_required: bool
    target_lane: str
    target_slot: object
    current_lane_allowed: bool
    board_valid: bool
    reason: str


class SignBoardInterpreter:
    """Convert stable slot labels to perception-only lane intent."""

    def interpret(self, current_lane, slot_states):
        current_lane = str(current_lane).upper()
        lane_to_slot = {lane: slot for slot, lane in config.SLOT_LANE_MAPPING.items()}
        if current_lane not in lane_to_slot:
            return SignBoardDecision(False, "UNKNOWN", None, False, False,
                                     "UNKNOWN_CURRENT_LANE")
        by_slot = {state.slot: state.stable_state for state in slot_states}
        current_slot = lane_to_slot[current_lane]
        current_state = by_slot.get(current_slot, "UNKNOWN")
        if current_state == "UNKNOWN":
            return SignBoardDecision(False, "UNKNOWN", None, False, False,
                                     "CURRENT_SLOT_UNKNOWN")
        if current_state == "RED_X":
            return SignBoardDecision(False, "UNKNOWN", None, False, True,
                                     "CURRENT_LANE_RED_X")
        if current_state == "GREEN_ARROW":
            return SignBoardDecision(False, "UNKNOWN", None, True, True,
                                     "CURRENT_LANE_GREEN_ARROW")
        if current_state != "LANE_CHANGE":
            return SignBoardDecision(False, "UNKNOWN", None, False, False,
                                     "UNSUPPORTED_CURRENT_SLOT_STATE")

        green_slots = [slot for slot, state in by_slot.items() if state == "GREEN_ARROW"]
        if len(green_slots) != 1:
            reason = "NO_GREEN_ARROW" if not green_slots else "MULTIPLE_GREEN_ARROWS"
            return SignBoardDecision(False, "UNKNOWN", None, False, False, reason)
        target_slot = green_slots[0]
        target_lane = config.SLOT_LANE_MAPPING[target_slot]
        if target_lane == current_lane:
            return SignBoardDecision(False, "UNKNOWN", None, False, False,
                                     "TARGET_EQUALS_CURRENT_LANE")
        return SignBoardDecision(True, target_lane, target_slot, False, True,
                                 "LANE_CHANGE_TO_GREEN_ARROW")
