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

    # Lane tracker only: suppress very wide horizontal paint.
    # Raw BEV remains untouched for stop-line detection.
    max_horizontal_run_px: int = 30

    # Pair-quality checks.
    # No assumed physical lane width is used here.
    min_pair_overlap_px: int = 30
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
    elif width_variation > cfg.max_width_variation_ratio:
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
    def __init__(self, cfg=None):
        self.cfg = cfg or PaperLaneConfig()
        self.previous_left = None
        self.previous_right = None

    def reset(self):
        self.previous_left = None
        self.previous_right = None

    def _search_previous(self, mask, coeff):
        ys, xs = np.nonzero(mask > 0)

        if len(xs) == 0:
            return None

        expected = np.polyval(coeff, ys)
        keep = np.abs(xs - expected) <= self.cfg.previous_margin_px

        return _ransac_poly2(xs[keep], ys[keep], self.cfg)

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
        return _ransac_poly2(xs[ids], ys[ids], self.cfg)

    def _histogram_seeds(self, mask):
        h, w = mask.shape

        # 전체 BEV를 사용하되 가까운 영역에 약간 더 높은 가중치.
        weights = np.linspace(0.5, 1.0, h)[:, None]
        hist = np.sum((mask > 0) * weights, axis=0)

        mid = w // 2

        left_hist = hist[:mid]
        right_hist = hist[mid:]

        left = None
        right = None

        if left_hist.size and left_hist.max() > 0:
            left = int(np.argmax(left_hist))

        if right_hist.size and right_hist.max() > 0:
            right = int(np.argmax(right_hist) + mid)

        return left, right

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

        # 이전 프레임 주변부터 탐색.
        if self.previous_left is not None:
            left = self._search_previous(
                mask, self.previous_left["coefficients"]
            )

        if self.previous_right is not None:
            right = self._search_previous(
                mask, self.previous_right["coefficients"]
            )

        # 실패한 쪽만 histogram + sliding window 재탐색.
        if left is None or right is None:
            left_seed, right_seed = self._histogram_seeds(mask)

            if left is None and left_seed is not None:
                left = self._sliding_window(mask, left_seed)

            if right is None and right_seed is not None:
                right = self._sliding_window(mask, right_seed)

        self.previous_left = left
        self.previous_right = right

        result = {
            "left": left,
            "right": right,
            "state": "NONE",
            "center": None,
            "horizontal_suppressed_pixels": int(suppressed_pixels),
        }

        if left is not None and right is not None:
            result["state"] = "BOTH_VISUAL"

            quality = _pair_quality(left, right, self.cfg)

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

                    joint_quality = _pair_quality(
                        joint_left, joint_right, self.cfg
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

            result["pair_quality"] = quality
            result["pair_state"] = (
                "PAIR_VALID" if quality["valid"] else "PAIR_WEAK"
            )

        elif left is not None:
            result["state"] = "LEFT_ONLY"

        elif right is not None:
            result["state"] = "RIGHT_ONLY"

        return result
