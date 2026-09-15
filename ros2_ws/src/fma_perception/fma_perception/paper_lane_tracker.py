"""Paper-style BEV lane tracker.

Input:
    binary BEV mask, uint8 HxW (0 or >0)

Pipeline:
    histogram seeds
    -> independent left/right sliding windows
    -> quadratic RANSAC fit
    -> previous-fit search
    -> center when both lanes exist

No vehicle control or hardware access.
"""

from dataclasses import dataclass
import numpy as np


@dataclass
class PaperLaneConfig:
    nwindows: int = 9
    margin_px: int = 45
    minpix: int = 8
    min_points: int = 35
    ransac_iterations: int = 80
    residual_px: float = 8.0
    joint_residual_px: float = 8.0
    previous_margin_px: int = 35

    # Re-acquisition after intersections / temporary lane loss.
    reacquire_after_weak_frames: int = 3

    # Keep following the surviving lane for a while.
    # Search for the missing side around the expected lane-width offset.
    reacquire_after_single_frames: int = 15
    missing_side_margin_px: int = 100
    initial_pair_width_px: float = 325.0
    # None uses the symmetric C920 BEV origin (width / 2).
    vehicle_center_x_px: float = None

    # Lane tracker only: suppress very wide horizontal paint.
    # Raw BEV remains untouched for stop-line detection.
    max_horizontal_run_px: int = 30

    # Pair-quality checks.
    # No assumed physical lane width is used here.
    min_pair_overlap_px: int = 30
    # Reject fake pairs made from two edges of one physical lane marking.
    # Current C920 metric-BEV data has the real pair concentrated near
    # 400-500 px, while false narrow pairs appear below ~350 px.
    min_pair_width_px: float = 350.0
    max_width_variation_ratio: float = 0.35
    max_heading_difference: float = 0.50



def _suppress_wide_horizontal_runs(mask, max_run_px):
    """Remove only wide horizontal runs from a COPY of the lane mask."""
    out = mask.copy()
    binary = mask > 0
    suppressed = 0

    for y in range(mask.shape[0]):
        xs = np.flatnonzero(binary[y])
        if len(xs) == 0:
            continue

        runs = np.split(
            xs,
            np.flatnonzero(np.diff(xs) > 1) + 1
        )

        for run in runs:
            if len(run) > max_run_px:
                out[y, run] = 0
                suppressed += len(run)

    return out, suppressed

def _ransac_poly2(x, y, cfg):
    if len(x) < cfg.min_points:
        return None

    t = y.astype(np.float64)
    x = x.astype(np.float64)

    rng = np.random.default_rng(7)
    best = None
    best_inliers = None

    for _ in range(cfg.ransac_iterations):
        ids = rng.choice(len(x), 3, replace=False)

        if np.ptp(t[ids]) < 10:
            continue

        try:
            coeff = np.polyfit(t[ids], x[ids], 2)
        except np.linalg.LinAlgError:
            continue

        residual = np.abs(x - np.polyval(coeff, t))
        inliers = residual <= cfg.residual_px

        if best_inliers is None or inliers.sum() > best_inliers.sum():
            best = coeff
            best_inliers = inliers

    if best_inliers is None or best_inliers.sum() < cfg.min_points:
        return None

    try:
        coeff = np.polyfit(t[best_inliers], x[best_inliers], 2)
    except np.linalg.LinAlgError:
        return None

    residual = np.sqrt(
        np.mean(
            (
                x[best_inliers]
                - np.polyval(coeff, t[best_inliers])
            ) ** 2
        )
    )

    return {
        "coefficients": coeff,
        "points": int(best_inliers.sum()),
        "residual_px": float(residual),
        "y_min": int(y[best_inliers].min()),
        "y_max": int(y[best_inliers].max()),
        "_inlier_x": x[best_inliers].copy(),
        "_inlier_y": t[best_inliers].copy(),
    }


def _pair_quality(left, right, cfg):
    y0 = max(left["y_min"], right["y_min"])
    y1 = min(left["y_max"], right["y_max"])

    quality = {
        "valid": False,
        "reason": "no_overlap",
        "overlap_px": max(0, y1-y0),
    }

    if y1 <= y0:
        return quality

    ys = np.linspace(y0, y1, 60)
    trim = max(1, int(len(ys) * 0.10))
    ys_q = ys[trim:-trim] if len(ys) > 2*trim else ys

    lx = np.polyval(left["coefficients"], ys_q)
    rx = np.polyval(right["coefficients"], ys_q)
    widths = rx-lx

    ld = np.polyval(np.polyder(left["coefficients"]), ys_q)
    rd = np.polyval(np.polyder(right["coefficients"]), ys_q)

    median_width = float(np.median(widths))
    p10 = float(np.percentile(widths, 10))
    p90 = float(np.percentile(widths, 90))

    width_variation = (
        float((p90-p10) / abs(median_width))
        if abs(median_width) > 1e-6 else float("inf")
    )

    heading_difference = float(np.percentile(np.abs(ld-rd), 90))
    crossing_ratio = float(np.mean(widths <= 0))

    quality.update(
        median_width_px=median_width,
        width_p10_px=p10,
        width_p90_px=p90,
        width_variation_ratio=width_variation,
        heading_difference=heading_difference,
        crossing_ratio=crossing_ratio,
    )

    if crossing_ratio > 0.10 or p10 <= 0:
        quality["reason"] = "lane_order_failed"
    elif y1-y0 < cfg.min_pair_overlap_px:
        quality["reason"] = "overlap_short"
    elif median_width < cfg.min_pair_width_px:
        quality["valid"] = False
        quality["reason"] = "lane_width_too_narrow"

    elif width_variation > cfg.max_width_variation_ratio:
        quality["valid"] = False
        quality["reason"] = "width_variation_failed"
    elif heading_difference > cfg.max_heading_difference:
        quality["reason"] = "heading_difference_failed"
    else:
        quality["valid"] = True
        quality["reason"] = "pair_valid"

    return quality


def _joint_refit(left, right, cfg):
    if "_inlier_x" not in left or "_inlier_x" not in right:
        return None

    lx = np.asarray(left["_inlier_x"], float)
    ly = np.asarray(left["_inlier_y"], float)
    rx = np.asarray(right["_inlier_x"], float)
    ry = np.asarray(right["_inlier_y"], float)

    kl = np.ones(len(lx), dtype=bool)
    kr = np.ones(len(rx), dtype=bool)

    params = None

    for _ in range(3):
        if kl.sum() < cfg.min_points or kr.sum() < cfg.min_points:
            return None

        A_l = np.c_[
            ly[kl]**2,
            ly[kl],
            np.ones(kl.sum()),
            np.zeros(kl.sum()),
        ]
        A_r = np.c_[
            ry[kr]**2,
            ry[kr],
            np.zeros(kr.sum()),
            np.ones(kr.sum()),
        ]

        # Give left/right lanes equal total influence.
        # Otherwise the side with many more pixels dominates
        # the shared curvature/heading.
        wl = 1.0 / np.sqrt(max(1, kl.sum()))
        wr = 1.0 / np.sqrt(max(1, kr.sum()))

        A = np.vstack([A_l * wl, A_r * wr])
        target = np.r_[lx[kl] * wl, rx[kr] * wr]

        try:
            params = np.linalg.lstsq(A, target, rcond=None)[0]
        except np.linalg.LinAlgError:
            return None

        a, b, cl, cr = params

        lres = np.abs(lx - (a*ly**2 + b*ly + cl))
        rres = np.abs(rx - (a*ry**2 + b*ry + cr))

        kl = lres <= cfg.joint_residual_px
        kr = rres <= cfg.joint_residual_px

    if kl.sum() < cfg.min_points or kr.sum() < cfg.min_points:
        return None

    a, b, cl, cr = params

    lfit = dict(left)
    rfit = dict(right)

    lfit.update(
        coefficients=np.array([a, b, cl]),
        points=int(kl.sum()),
        residual_px=float(np.sqrt(np.mean(
            (lx[kl] - (a*ly[kl]**2 + b*ly[kl] + cl))**2
        ))),
        y_min=int(ly[kl].min()),
        y_max=int(ly[kl].max()),
        _inlier_x=lx[kl],
        _inlier_y=ly[kl],
    )

    rfit.update(
        coefficients=np.array([a, b, cr]),
        points=int(kr.sum()),
        residual_px=float(np.sqrt(np.mean(
            (rx[kr] - (a*ry[kr]**2 + b*ry[kr] + cr))**2
        ))),
        y_min=int(ry[kr].min()),
        y_max=int(ry[kr].max()),
        _inlier_x=rx[kr],
        _inlier_y=ry[kr],
    )

    return lfit, rfit


class PaperLaneTracker:
    def __init__(self, cfg=None, *, diagnostics=False):
        self.cfg = cfg or PaperLaneConfig()
        self.diagnostics = diagnostics
        self.previous_left = None
        self.previous_right = None
        self.weak_streak = 0
        self.single_streak = 0
        self.last_valid_pair = None
        self.frames_since_valid = 0
        self.last_valid_width_px = float(
            self.cfg.initial_pair_width_px
        )

    def reset(self):
        self.previous_left = None
        self.previous_right = None
        self.weak_streak = 0
        self.single_streak = 0
        self.last_valid_pair = None
        self.frames_since_valid = 0
        self.last_valid_width_px = float(
            self.cfg.initial_pair_width_px
        )

    def _fit_candidates(self, x, y):
        fit = _ransac_poly2(x, y, self.cfg)
        if self.diagnostics and fit is not None:
            # Preserve the exact search output BEFORE RANSAC rejection. Joint
            # refit copies these keys, so telemetry follows the selected fit.
            fit['_candidate_x'] = x.copy()
            fit['_candidate_y'] = y.copy()
        return fit

    def _search_around(self, mask, coeff, margin_px):
        ys, xs = np.nonzero(mask > 0)

        if len(xs) == 0:
            return None

        expected = np.polyval(coeff, ys)
        keep = np.abs(xs - expected) <= float(margin_px)

        return self._fit_candidates(xs[keep], ys[keep])

    def _search_previous(self, mask, coeff):
        return self._search_around(
            mask,
            coeff,
            self.cfg.previous_margin_px,
        )

    def _search_missing_side(self, mask, visible_fit, direction):
        if visible_fit is None:
            return None

        coeff = np.asarray(
            visible_fit["coefficients"],
            dtype=float,
        ).copy()

        # x(y) = ay^2 + by + c.
        # Moving to the opposite lane only changes the intercept.
        coeff[2] += (
            float(direction)
            * self.last_valid_width_px
        )

        return self._search_around(
            mask,
            coeff,
            self.cfg.missing_side_margin_px,
        )

    def _sliding_window(self, mask, base_x):
        h, w = mask.shape
        ys, xs = np.nonzero(mask > 0)

        if len(xs) == 0:
            return None

        window_h = max(1, h // self.cfg.nwindows)
        current_x = float(base_x)

        collected = []

        for win in range(self.cfg.nwindows):
            y_high = h - win * window_h
            y_low = max(0, h - (win + 1) * window_h)

            keep = (
                (ys >= y_low)
                & (ys < y_high)
                & (xs >= current_x - self.cfg.margin_px)
                & (xs <= current_x + self.cfg.margin_px)
            )

            ids = np.flatnonzero(keep)

            if len(ids):
                collected.extend(ids.tolist())

                if len(ids) >= self.cfg.minpix:
                    current_x = float(np.mean(xs[ids]))

        if len(collected) < self.cfg.min_points:
            return None

        ids = np.unique(collected)
        return self._fit_candidates(xs[ids], ys[ids])

    def _vehicle_center(self, width):
        center = self.cfg.vehicle_center_x_px
        return width / 2.0 if center is None else float(center)

    def _selection_quality(self, left, right, width):
        quality = _pair_quality(left, right, self.cfg)
        # Use the nearest COMMON observed row, not extrapolated image bottom.
        y = min(left["y_max"], right["y_max"])
        lx = float(np.polyval(left["coefficients"], y))
        rx = float(np.polyval(right["coefficients"], y))
        ego = self._vehicle_center(width)
        if not lx < ego < rx:
            return {**quality, "valid": False, "reason": "ego_not_enclosed"}
        if (quality["valid"] and self.last_valid_pair is not None
                and self.frames_since_valid < self.cfg.reacquire_after_single_frames):
            # Retain the confirmed boundaries through short occlusions. A new
            # adjacent boundary must not replace a missing one immediately.
            displacement = []
            for fit, previous in zip((left, right), self.last_valid_pair):
                ys = np.linspace(max(fit["y_min"], previous["y_min"]),
                                 min(fit["y_max"], previous["y_max"]), 20)
                if ys[-1] < ys[0]:
                    displacement.append(float("inf"))
                else:
                    displacement.append(float(np.max(np.abs(
                        np.polyval(fit["coefficients"], ys)
                        - np.polyval(previous["coefficients"], ys)))))
            if max(displacement) > self.cfg.previous_margin_px:
                return {**quality, "valid": False, "reason": "pair_lock_mismatch"}
        return quality

    def _cold_pair(self, mask):
        h, w = mask.shape
        weights = np.linspace(0.5, 1.0, h)[:, None]
        hist = np.sum((mask > 0) * weights, axis=0)
        candidates = []
        # Nonmaximum suppression prevents fitting both edges of the same paint.
        while hist.max() > 0:
            seed = int(np.argmax(hist))
            hist[max(0, seed-self.cfg.margin_px):min(w, seed+self.cfg.margin_px+1)] = 0
            fit = self._sliding_window(mask, seed)
            if fit is not None:
                candidates.append(fit)
        ego = self._vehicle_center(w)
        lefts = [f for f in candidates
                 if np.polyval(f["coefficients"], f["y_max"]) < ego]
        rights = [f for f in candidates
                  if np.polyval(f["coefficients"], f["y_max"]) > ego]
        pairs = []
        for left in lefts:
            for right in rights:
                quality = self._selection_quality(left, right, w)
                y = min(left["y_max"], right["y_max"])
                lx, rx = (float(np.polyval(f["coefficients"], y)) for f in (left, right))
                if not lx < ego < rx:
                    continue
                # Existing width/shape gates first, then centered initial pair.
                # Once locked, temporal gating above overrides this preference.
                score = (not quality["valid"], abs((lx+rx)/2-ego), rx-lx,
                         left["residual_px"]+right["residual_px"])
                pairs.append((score, left, right))
        if pairs:
            _, left, right = min(pairs, key=lambda entry: entry[0])
            return left, right
        def nearest(fits):
            return min(fits, key=lambda f: abs(
                np.polyval(f["coefficients"], f["y_max"])-ego)) if fits else None
        return nearest(lefts), nearest(rights)

    def process(self, mask):
        if mask.ndim != 2:
            raise ValueError("BEV mask must be HxW")

        # Never modify the caller's raw BEV.
        # Stop-line/crosswalk logic can still use the original mask.
        mask, suppressed_pixels = _suppress_wide_horizontal_runs(
            mask, self.cfg.max_horizontal_run_px
        )

        h, w = mask.shape

        left = None
        right = None

        # If tracking has been weak/single-sided for several frames,
        # stop trusting the old polynomial and perform a cold search.
        force_full_search = (
            self.weak_streak >= self.cfg.reacquire_after_weak_frames
            or self.single_streak >= self.cfg.reacquire_after_single_frames
        )

        if not force_full_search:
            # Fast path: search around the previous frame's fits.
            if self.previous_left is not None:
                left = self._search_previous(
                    mask,
                    self.previous_left["coefficients"],
                )

            if self.previous_right is not None:
                right = self._search_previous(
                    mask,
                    self.previous_right["coefficients"],
                )

        # If exactly one lane survives, use it only as a SEARCH GUIDE
        # for the missing side. This never creates a synthetic lane or
        # center; the missing lane still has to be fitted from real pixels.
        if not force_full_search:
            if left is not None and right is None:
                right = self._search_missing_side(
                    mask,
                    left,
                    +1.0,
                )

            elif right is not None and left is None:
                left = self._search_missing_side(
                    mask,
                    right,
                    -1.0,
                )

        # Evaluate all cold candidates together instead of independent maxima.
        if force_full_search or left is None or right is None:
            left, right = self._cold_pair(mask)

        result = {
            "left": left,
            "right": right,
            "state": "NONE",
            "center": None,
            "temporary_center": None,
            "center_source": "NONE",
            "horizontal_suppressed_pixels": int(suppressed_pixels),
            "force_full_search": bool(force_full_search),
            "reacquire_attempted": False,
            "reacquire_used": False,
        }

        if left is not None and right is not None:
            result["state"] = "BOTH_VISUAL"

            quality = self._selection_quality(left, right, w)

            # Important recovery path:
            # previous-fit search may find two lanes but they can belong
            # to stale/incorrect tracks after an intersection.
            # If the pair is weak, immediately try a fresh histogram +
            # sliding-window search on BOTH sides.
            if not quality["valid"] and not force_full_search:
                result["reacquire_attempted"] = True

                cold_left, cold_right = self._cold_pair(mask)

                if cold_left is not None and cold_right is not None:
                    cold_quality = self._selection_quality(cold_left, cold_right, w)

                    if cold_quality["valid"]:
                        left = cold_left
                        right = cold_right
                        quality = cold_quality

                        result["left"] = left
                        result["right"] = right
                        result["reacquire_used"] = True

            result["joint_refit_attempted"] = False
            result["joint_refit_used"] = False
            result["joint_refit_candidate_reason"] = None

            if (
                not quality["valid"]
                and quality["reason"] in {
                    "lane_order_failed",
                    "width_variation_failed",
                    "heading_difference_failed",
                }
            ):
                result["joint_refit_attempted"] = True
                candidate = _joint_refit(left, right, self.cfg)

                if candidate is None:
                    result["joint_refit_candidate_reason"] = "fit_failed"
                else:
                    joint_left, joint_right = candidate
                    result["joint_candidate_left_range"] = [
                        joint_left["y_min"], joint_left["y_max"]
                    ]
                    result["joint_candidate_right_range"] = [
                        joint_right["y_min"], joint_right["y_max"]
                    ]
                    result["joint_candidate_left_points"] = joint_left["points"]
                    result["joint_candidate_right_points"] = joint_right["points"]

                    joint_quality = self._selection_quality(
                        joint_left, joint_right, w
                    )
                    result["joint_refit_candidate_reason"] = (
                        joint_quality["reason"]
                    )

                    # Only accept a joint fit that independently passes
                    # the existing pair-quality checks.
                    if joint_quality["valid"]:
                        left = joint_left
                        right = joint_right
                        quality = joint_quality
                        result["left"] = left
                        result["right"] = right
                        result["joint_refit_used"] = True

            if quality["valid"]:
                y0 = max(left["y_min"], right["y_min"])
                y1 = min(left["y_max"], right["y_max"])

                result["center"] = {
                    "coefficients": (
                        left["coefficients"] + right["coefficients"]
                    ) / 2.0,
                    "y_min": y0,
                    "y_max": y1,
                }
                result["center_source"] = "PAIR_VALID"

            result["pair_quality"] = quality
            result["pair_state"] = (
                "PAIR_VALID" if quality["valid"] else "PAIR_WEAK"
            )

        elif left is not None:
            result["state"] = "LEFT_ONLY"

        elif right is not None:
            result["state"] = "RIGHT_ONLY"

        # Single-lane fallback center.
        #
        # IMPORTANT:
        # - The detected lane itself is NEVER used as center.
        # - This is kept separate from result["center"].
        # - PAIR_WEAK / NONE do not get a temporary center.
        #
        # x(y) = ay^2 + by + c, so a lateral offset only changes c.
        fallback_allowed = (
            self.last_valid_pair is not None
            and self.last_valid_width_px >= self.cfg.min_pair_width_px
            and self.frames_since_valid < self.cfg.reacquire_after_single_frames
        )
        if fallback_allowed and result["state"] in {"LEFT_ONLY", "RIGHT_ONLY"}:
            side = 0 if left is not None else 1
            visible = left if left is not None else right
            previous = self.last_valid_pair[side]
            y = min(visible["y_max"], previous["y_max"])
            fallback_allowed = (
                y >= max(visible["y_min"], previous["y_min"])
                and abs(np.polyval(visible["coefficients"], y)
                        - np.polyval(previous["coefficients"], y)) <= self.cfg.previous_margin_px
            )
        if fallback_allowed and result["state"] == "LEFT_ONLY" and left is not None:
            coeff = np.asarray(
                left["coefficients"],
                dtype=float,
            ).copy()

            coeff[2] += 0.5 * self.last_valid_width_px

            result["temporary_center"] = {
                "coefficients": coeff,
                "y_min": int(left["y_min"]),
                "y_max": int(left["y_max"]),
            }
            result["center_source"] = "LEFT_ONLY_OFFSET"

        elif fallback_allowed and result["state"] == "RIGHT_ONLY" and right is not None:
            coeff = np.asarray(
                right["coefficients"],
                dtype=float,
            ).copy()

            coeff[2] -= 0.5 * self.last_valid_width_px

            result["temporary_center"] = {
                "coefficients": coeff,
                "y_min": int(right["y_min"]),
                "y_max": int(right["y_max"]),
            }
            result["center_source"] = "RIGHT_ONLY_OFFSET"

        # Record the width used for single-lane fallback.
        result["reference_lane_width_px"] = float(
            self.last_valid_width_px
        )

        # Update recovery state only after the final classification.
        pair_state = result.get("pair_state")

        if pair_state == "PAIR_VALID":
            self.weak_streak = 0
            self.single_streak = 0

            q = result.get("pair_quality") or {}
            width = q.get("median_width_px")

            if width is not None and 350.0 <= float(width) <= 550.0:
                # Start with a measured width, never blend an assumed width
                # into the first confirmed single-line fallback.
                self.last_valid_width_px = (float(width) if self.last_valid_pair is None else
                    0.8 * self.last_valid_width_px + 0.2 * float(width))

        elif pair_state == "PAIR_WEAK":
            self.weak_streak += 1
            self.single_streak = 0

        elif result["state"] in {"LEFT_ONLY", "RIGHT_ONLY"}:
            self.single_streak += 1
            self.weak_streak = 0

        else:
            # NONE: there is nothing useful to trust on the next frame.
            self.weak_streak = 0
            self.single_streak = 0

        if pair_state == "PAIR_VALID":
            self.last_valid_pair = (left, right)
            self.frames_since_valid = 0
        else:
            self.frames_since_valid += 1

        self.previous_left = left
        self.previous_right = right

        result["weak_streak"] = int(self.weak_streak)
        result["single_streak"] = int(self.single_streak)

        return result
