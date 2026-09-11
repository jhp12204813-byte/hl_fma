"""Generic consecutive-frame state stabilization."""


class StableStateFilter:
    """Confirm new labels and tolerate a short run of UNKNOWN observations."""

    def __init__(self, confirm_frames=3, lost_frames=3, unknown="UNKNOWN"):
        self.confirm_frames = max(1, int(confirm_frames))
        self.lost_frames = max(1, int(lost_frames))
        self.unknown = unknown
        self.stable_state = unknown
        self.candidate = unknown
        self.candidate_count = 0
        self.lost_count = 0

    def update(self, raw_state):
        if raw_state == self.unknown:
            self.lost_count += 1
            self.candidate = self.unknown
            self.candidate_count = 0
            if self.lost_count >= self.lost_frames:
                self.stable_state = self.unknown
            return self.stable_state

        self.lost_count = 0
        if raw_state == self.candidate:
            self.candidate_count += 1
        else:
            self.candidate = raw_state
            self.candidate_count = 1
        if self.candidate_count >= self.confirm_frames:
            self.stable_state = raw_state
        return self.stable_state
