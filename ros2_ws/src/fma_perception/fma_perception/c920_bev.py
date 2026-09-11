from pathlib import Path

import cv2
import numpy as np
import yaml


class C920BEV:
    def __init__(self, yaml_path):
        yaml_path = Path(yaml_path).expanduser()

        with yaml_path.open("r", encoding="utf-8") as f:
            cfg = yaml.safe_load(f)

        self.image_width = int(cfg["image_width"])
        self.image_height = int(cfg["image_height"])

        self.H = np.asarray(
            cfg["homography_image_to_bev"],
            dtype=np.float64,
        )

        bev = cfg["bev"]

        self.width = int(bev["width_px"])
        self.height = int(bev["height_px"])

        self.resolution_m = float(bev["resolution_m"])
        self.half_width_m = float(bev["half_width_m"])
        self.near_m = float(bev["near_m"])
        self.far_m = float(bev["far_m"])

    def warp_to_bev(self, frame):
        h, w = frame.shape[:2]

        if (w, h) != (self.image_width, self.image_height):
            raise ValueError(
                f"Expected {self.image_width}x{self.image_height}, "
                f"got {w}x{h}"
            )

        return cv2.warpPerspective(
            frame,
            self.H,
            (self.width, self.height),
            flags=cv2.INTER_LINEAR,
        )

    def bev_pixel_to_ground(self, u, v):
        x = (u - self.width / 2.0) * self.resolution_m
        y = self.far_m - v * self.resolution_m
        return float(x), float(y)

    def ground_to_bev_pixel(self, x, y):
        u = x / self.resolution_m + self.width / 2.0
        v = (self.far_m - y) / self.resolution_m
        return float(u), float(v)
