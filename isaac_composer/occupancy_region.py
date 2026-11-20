"""Read a ROS map directly for spawn/path constraints without starting Nav2."""
from __future__ import annotations

import math
from pathlib import Path

import numpy as np
from PIL import Image
import yaml
from scipy import ndimage


class OccupancyRegion:
    def __init__(self, map_yaml: str | Path, clearance_m: float = 0.0):
        self.path = Path(map_yaml).resolve()
        config = yaml.safe_load(self.path.read_text())
        self.config = config
        self.resolution = float(config['resolution'])
        self.origin = tuple(map(float, config['origin']))
        clearance_m = float(clearance_m)
        free_threshold = float(config.get('free_thresh', 0.196))
        occupied_threshold = float(config.get('occupied_thresh', 0.65))
        if (not math.isfinite(self.resolution) or self.resolution <= 0
                or len(self.origin) != 3 or not all(map(math.isfinite, self.origin))
                or not math.isfinite(clearance_m) or clearance_m < 0
                or not 0 <= free_threshold < occupied_threshold <= 1):
            raise ValueError('Invalid occupancy map resolution, origin, thresholds, or clearance')
        image_path = Path(config['image'])
        if not image_path.is_absolute():
            image_path = self.path.parent / image_path
        self.image_path = image_path.resolve()
        with Image.open(image_path) as image:
            rgba = np.asarray(image.convert('RGBA'), dtype=np.float64)
        gray = rgba[:, :, :3].mean(axis=2) / 255.0
        occupancy = gray if config.get('negate', 0) else 1.0 - gray
        free = (occupancy < free_threshold) & (rgba[:, :, 3] == 255)
        # Pad as unknown so clearance also applies at image boundaries.
        distance = ndimage.distance_transform_edt(np.pad(free, 1))[1:-1, 1:-1] * self.resolution
        self.allowed = free & (distance > clearance_m)
        if not self.allowed.any():
            raise ValueError('Occupancy map has no free space after clearance filtering')
        self.height, self.width = self.allowed.shape

    def allows_xy(self, x, y):
        x, y = np.broadcast_arrays(np.asarray(x, dtype=float), np.asarray(y, dtype=float))
        dx, dy = x - self.origin[0], y - self.origin[1]
        c, s = math.cos(self.origin[2]), math.sin(self.origin[2])
        local_x, local_y = c * dx + s * dy, -s * dx + c * dy
        finite = np.isfinite(local_x) & np.isfinite(local_y)
        col = np.floor(np.where(finite, local_x, -1) / self.resolution).astype(int)
        row = self.height - 1 - np.floor(np.where(finite, local_y, -1) / self.resolution).astype(int)
        valid = finite & (col >= 0) & (col < self.width) & (row >= 0) & (row < self.height)
        return valid & self.allowed[np.clip(row, 0, self.height - 1), np.clip(col, 0, self.width - 1)]

    def allows_segment(self, start, end):
        count = max(1, math.ceil(math.dist(start[:2], end[:2]) / (self.resolution * 0.5)))
        alpha = np.linspace(0, 1, count + 1)
        x = start[0] + alpha * (end[0] - start[0])
        y = start[1] + alpha * (end[1] - start[1])
        return bool(self.allows_xy(x, y).all())
