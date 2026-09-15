from dataclasses import dataclass

import cv2
import numpy as np


@dataclass
class StopLineConfig:
    min_distance_m: float = 0.8
    max_distance_m: float = 4.0

    fallback_half_width_m: float = 1.4

    # A stop line must occupy a large fraction of the lane horizontally.
    min_row_coverage_ratio: float = 0.42
    min_continuous_ratio: float = 0.38

    # Physical stop-line thickness.
    min_thickness_m: float = 0.04
    max_thickness_m: float = 0.55

    # Only bridge small cracks in painted lines.
    continuity_close_m: float = 0.08

    # Multiple nearby thin transverse bands may indicate a crossing.
    crosswalk_min_bands: int = 3
    crosswalk_max_span_m: float = 1.5

    # Broken/cracked portions of one physical stop line can appear as
    # several adjacent horizontal bands. Merge them before counting
    # crosswalk stripes.
    merge_band_gap_m: float = 0.15


class StopLineDetector:
    def __init__(
        self,
        resolution_m,
        far_m,
        width_px,
        config=None,
    ):
        self.resolution_m = float(resolution_m)
        self.far_m = float(far_m)
        self.width_px = int(width_px)
        self.config = config or StopLineConfig()

    def _row_to_distance(self, v):
        return self.far_m - float(v) * self.resolution_m

    def _fit_x(self, fit, y):
        if fit is None:
            return None

        y0 = float(fit.get("y_min", -1))
        y1 = float(fit.get("y_max", -1))

        if not (y0 <= y <= y1):
            return None

        return float(
            np.polyval(
                fit["coefficients"],
                y,
            )
        )

    def _corridor_mask(self, shape, lane_result):
        h, w = shape
        roi = np.zeros((h, w), dtype=np.uint8)

        left = lane_result.get("left")
        right = lane_result.get("right")

        fallback_half_px = int(round(
            self.config.fallback_half_width_m
            / self.resolution_m
        ))

        cx = w // 2

        for v in range(h):
            d = self._row_to_distance(v)

            if not (
                self.config.min_distance_m
                <= d
                <= self.config.max_distance_m
            ):
                continue

            lx = self._fit_x(left, v)
            rx = self._fit_x(right, v)

            if (
                lx is not None
                and rx is not None
                and rx > lx
            ):
                x0 = int(round(lx))
                x1 = int(round(rx))
            else:
                x0 = cx - fallback_half_px
                x1 = cx + fallback_half_px

            x0 = max(0, min(w - 1, x0))
            x1 = max(0, min(w, x1))

            if x1 > x0:
                roi[v, x0:x1] = 255

        return roi

    @staticmethod
    def _longest_run(row):
        b = np.asarray(row > 0, dtype=np.uint8)

        if not np.any(b):
            return 0

        padded = np.pad(b, (1, 1))
        edges = np.diff(padded.astype(np.int8))

        starts = np.flatnonzero(edges == 1)
        ends = np.flatnonzero(edges == -1)

        if len(starts) == 0:
            return 0

        return int(np.max(ends - starts))

    def detect(self, white_bev, lane_result):
        if white_bev.ndim != 2:
            raise ValueError(
                "white_bev must be single-channel"
            )

        cfg = self.config

        corridor = self._corridor_mask(
            white_bev.shape,
            lane_result,
        )

        raw = cv2.bitwise_and(
            white_bev,
            corridor,
        )

        # Close only tiny cracks. Do NOT join separated road letters.
        close_px = max(
            1,
            int(round(
                cfg.continuity_close_m
                / self.resolution_m
            )),
        )

        kernel = cv2.getStructuringElement(
            cv2.MORPH_RECT,
            (close_px, 1),
        )

        continuity = cv2.morphologyEx(
            raw,
            cv2.MORPH_CLOSE,
            kernel,
        )

        h, w = raw.shape

        row_good = np.zeros(h, dtype=bool)
        row_score = np.zeros(h, dtype=float)
        row_bounds = {}

        for v in range(h):
            d = self._row_to_distance(v)

            if not (
                cfg.min_distance_m
                <= d
                <= cfg.max_distance_m
            ):
                continue

            xs = np.flatnonzero(corridor[v] > 0)

            if len(xs) == 0:
                continue

            x0 = int(xs[0])
            x1 = int(xs[-1]) + 1

            corridor_width = x1 - x0

            if corridor_width <= 0:
                continue

            raw_row = raw[v, x0:x1]
            cont_row = continuity[v, x0:x1]

            coverage = (
                np.count_nonzero(raw_row)
                / float(corridor_width)
            )

            longest = (
                self._longest_run(cont_row)
                / float(corridor_width)
            )

            score = (
                0.55 * coverage
                + 0.45 * longest
            )

            row_score[v] = score
            row_bounds[v] = (
                x0,
                x1,
                coverage,
                longest,
            )

            if (
                coverage >= cfg.min_row_coverage_ratio
                and longest >= cfg.min_continuous_ratio
            ):
                row_good[v] = True

        # Group adjacent good rows into transverse-band candidates.
        candidates = []

        good_rows = np.flatnonzero(row_good)

        if len(good_rows):
            # Worn/cracked stop paint often breaks a physical 30-55 cm band
            # into several short runs of good rows.  Merge nearby runs using
            # the existing physical merge_band_gap_m limit instead of
            # requiring pixel-perfect row adjacency.
            max_gap_px = max(
                1,
                int(round(
                    cfg.merge_band_gap_m
                    / self.resolution_m
                )),
            )

            groups = np.split(
                good_rows,
                np.where(np.diff(good_rows) > max_gap_px)[0] + 1,
            )

            for rows in groups:
                if len(rows) == 0:
                    continue

                y0 = int(rows[0])
                y1 = int(rows[-1])

                thickness_px = y1 - y0 + 1
                thickness_m = (
                    thickness_px
                    * self.resolution_m
                )

                if not (
                    cfg.min_thickness_m
                    <= thickness_m
                    <= cfg.max_thickness_m
                ):
                    continue

                best_v = int(
                    rows[
                        np.argmax(
                            row_score[rows]
                        )
                    ]
                )

                x0, x1, coverage, longest = (
                    row_bounds[best_v]
                )

                distance = self._row_to_distance(
                    0.5 * (y0 + y1)
                )

                confidence = float(np.clip(
                    0.5 * coverage
                    + 0.5 * longest,
                    0.0,
                    1.0,
                ))

                candidates.append({
                    "x": int(x0),
                    "y": int(y0),
                    "width": int(x1 - x0),
                    "height": int(thickness_px),
                    "distance_m": float(distance),
                    "coverage_ratio": float(coverage),
                    "continuous_ratio": float(longest),
                    "confidence": confidence,
                })

        candidates.sort(
            key=lambda c: c["distance_m"]
        )

        # Repeated transverse lines close together:
        # mark separately instead of choosing one as STOP.
        crosswalk = False

        if len(candidates) >= cfg.crosswalk_min_bands:
            ds = sorted(
                c["distance_m"]
                for c in candidates
            )

            n = cfg.crosswalk_min_bands

            for i in range(len(ds) - n + 1):
                g = ds[i:i+n]

                if (
                    max(g) - min(g)
                    <= cfg.crosswalk_max_span_m
                ):
                    crosswalk = True
                    break

        best = None

        if candidates and not crosswalk:
            # Vehicle approaches the nearest valid stop line first.
            best = min(
                candidates,
                key=lambda c: c["distance_m"],
            )

        return {
            "stop_line_detected": best is not None,
            "stop_line_distance_m": (
                None
                if best is None
                else best["distance_m"]
            ),
            "stop_line_confidence": (
                0.0
                if best is None
                else best["confidence"]
            ),
            "crosswalk_detected": bool(crosswalk),
            "candidate_count": len(candidates),
            "candidates": candidates,
            "debug_mask": raw,
            "corridor_mask": corridor,
        }
