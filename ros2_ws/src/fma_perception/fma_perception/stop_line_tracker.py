from dataclasses import dataclass


@dataclass
class StopLineTrackerConfig:
    confirm_frames: int = 3

    # 실제 출력은 오래된 거리값을 너무 오래 유지하지 않는다.
    max_hold_missed_frames: int = 15

    # 하지만 같은 물리적 정지선의 identity는 더 오래 기억해서
    # 검출이 잠깐 끊겨도 새 이벤트로 만들지 않는다.
    max_reassociate_missed_frames: int = 90

    max_distance_increase_m: float = 0.35
    new_line_jump_m: float = 1.2


class StopLineTracker:
    def __init__(self, cfg=None):
        self.cfg = cfg or StopLineTrackerConfig()
        self.event_id = 0
        self.reset()

    def reset(self):
        self.active = False
        self.confirm_count = 0
        self.missed_count = 0
        self.distance_m = None
        self.confidence = 0.0

    def update(self, detection):
        detected = bool(
            detection.get(
                "stop_line_detected",
                detection.get("detected", False),
            )
        )

        distance = detection.get(
            "stop_line_distance_m",
            detection.get("distance_m"),
        )

        confidence = float(
            detection.get(
                "stop_line_confidence",
                detection.get("confidence", 0.0),
            )
        )

        new_event = False

        if detected and distance is not None:
            distance = float(distance)

            if self.distance_m is None:
                # 첫 후보
                self.distance_m = distance
                self.confirm_count = 1
                self.missed_count = 0

            elif self.active:
                jump = distance - self.distance_m

                if jump > self.cfg.new_line_jump_m:
                    # 기존 정지선보다 갑자기 훨씬 먼 선:
                    # 새 정지선 후보로 시작하지만 아직 event 확정 X
                    self.active = False
                    self.confirm_count = 1
                    self.distance_m = distance
                    self.missed_count = 0

                else:
                    # 같은 정지선.
                    # 가까워지는 변화는 허용하고,
                    # 작은 거리 증가도 측정 오차로 허용.
                    if jump <= self.cfg.max_distance_increase_m:
                        self.distance_m = distance

                    self.missed_count = 0

            else:
                # 아직 confirm되지 않은 후보
                jump = distance - self.distance_m

                if jump > self.cfg.max_distance_increase_m:
                    # 후보가 갑자기 멀리 바뀌면 이전 후보 폐기
                    self.distance_m = distance
                    self.confirm_count = 1
                else:
                    self.distance_m = distance
                    self.confirm_count += 1

                self.missed_count = 0

                if self.confirm_count >= self.cfg.confirm_frames:
                    self.active = True
                    self.event_id += 1
                    new_event = True

            self.confidence = confidence

        else:
            if self.distance_m is not None:
                self.missed_count += 1

                if (
                    self.missed_count
                    > self.cfg.max_reassociate_missed_frames
                ):
                    self.reset()

        # 오래된 거리값을 실제 STOP 출력으로 유지하지 않는다.
        stop_tracked = bool(
            self.active
            and self.missed_count
            <= self.cfg.max_hold_missed_frames
        )

        return {
            "stop_tracked": stop_tracked,
            "tracked_stop_distance_m": (
                None
                if self.distance_m is None
                else float(self.distance_m)
            ),
            "tracked_stop_confidence": float(self.confidence),
            "stop_confirm_count": int(self.confirm_count),
            "stop_missed_count": int(self.missed_count),
            "new_stop_event": bool(new_event),
            "stop_event_id": int(self.event_id),
        }
