"""Static occupancy-map robot routes, independent of pedestrian NavMesh queries."""

from __future__ import annotations

import heapq
import math
from pathlib import Path

import numpy as np

from .occupancy_region import OccupancyRegion


class StaticMapPath:
    """Eight-connected Dijkstra with a reusable, goal-rooted search frontier.

    Unknown/occupied cells and corner cutting are blocked. All map data is
    static: people, robot sensor scans and animation do not modify this graph.
    """

    path_source = "static_occupancy_map"

    def __init__(self, map_file, clearance_m, floor_z=0.0, allowed_region_xy=None):
        from scipy import ndimage

        self.raw = OccupancyRegion(map_file)
        self.safe = OccupancyRegion(map_file, clearance_m=clearance_m)
        self.path = self.raw.path
        self.clearance_m, self.floor_z = float(clearance_m), float(floor_z)
        self.resolution = self.raw.resolution
        self.mask = self.safe.allowed.copy()
        if allowed_region_xy is not None:
            x0, y0, x1, y1 = allowed_region_xy
            rows, cols = np.indices(self.mask.shape)
            x, y = self._world_xy(rows, cols)
            self.mask &= (x >= x0) & (x <= x1) & (y >= y0) & (y <= y1)
        if not self.mask.any():
            raise ValueError("Static map has no robot-clearance free cells in the allowed region")
        self.labels, _ = ndimage.label(self.mask)
        self._goal = None
        self._heap = []
        self._distances = self._next = self._settled = None

    def _world_xy(self, row, col):
        x = (col + 0.5) * self.resolution
        y = (self.mask.shape[0] - row - 0.5) * self.resolution
        ox, oy, yaw = self.raw.origin
        c, s = math.cos(yaw), math.sin(yaw)
        return ox + c * x - s * y, oy + s * x + c * y

    def world_point(self, row, col):
        x, y = self._world_xy(row, col)
        return [float(x), float(y), self.floor_z]

    def _local(self, point):
        ox, oy, yaw = self.raw.origin
        dx, dy = float(point[0]) - ox, float(point[1]) - oy
        c, s = math.cos(yaw), math.sin(yaw)
        return (c * dx + s * dy) / self.resolution, (-s * dx + c * dy) / self.resolution

    def cell(self, point):
        x, y = self._local(point)
        if not math.isfinite(x) or not math.isfinite(y):
            return -1, -1
        return self.mask.shape[0] - 1 - math.floor(y), math.floor(x)

    @staticmethod
    def _inside(mask, row, col):
        return 0 <= row < mask.shape[0] and 0 <= col < mask.shape[1] and bool(mask[row, col])

    def is_allowed_world(self, point):
        return self._inside(self.mask, *self.cell(point))

    def allows_segment(self, start, end, raw=False):
        """Supercover traversal, including both cells touched at grid corners."""
        mask = self.raw.allowed if raw else self.mask
        x, y = self._local(start)
        ex, ey = self._local(end)
        if not all(map(math.isfinite, (x, y, ex, ey))):
            return False
        cx, cy, tx, ty = math.floor(x), math.floor(y), math.floor(ex), math.floor(ey)
        h = mask.shape[0]

        def free(col, bottom_row):
            return self._inside(mask, h - 1 - bottom_row, col)

        if not free(cx, cy) or not free(tx, ty):
            return False
        dx, dy = ex - x, ey - y
        sx, sy = (1 if dx > 0 else -1), (1 if dy > 0 else -1)
        dt_x = 1 / abs(dx) if dx else math.inf
        dt_y = 1 / abs(dy) if dy else math.inf
        next_x = ((cx + 1 - x) if dx > 0 else (x - cx)) * dt_x if dx else math.inf
        next_y = ((cy + 1 - y) if dy > 0 else (y - cy)) * dt_y if dy else math.inf
        while (cx, cy) != (tx, ty):
            if abs(next_x - next_y) < 1e-12:
                if not free(cx + sx, cy) or not free(cx, cy + sy):
                    return False
                cx, cy = cx + sx, cy + sy
                next_x, next_y = next_x + dt_x, next_y + dt_y
            elif next_x < next_y:
                cx, next_x = cx + sx, next_x + dt_x
            else:
                cy, next_y = cy + sy, next_y + dt_y
            if not free(cx, cy):
                return False
        return True

    def project_world(self, point):
        """Recover a start near a padded boundary without crossing a real wall."""
        if self.is_allowed_world(point):
            return [float(point[0]), float(point[1]), self.floor_z]
        if not self._inside(self.raw.allowed, *self.cell(point)):
            raise RuntimeError("Robot center is in occupied, unknown or out-of-map space")
        row, col = self.cell(point)
        limit = max(0.35, self.clearance_m + self.resolution)
        reach = math.ceil(limit / self.resolution) + 1
        r0, r1 = max(0, row - reach), min(self.mask.shape[0], row + reach + 1)
        c0, c1 = max(0, col - reach), min(self.mask.shape[1], col + reach + 1)
        candidates = np.argwhere(self.mask[r0:r1, c0:c1]) + [r0, c0]
        points = [self.world_point(int(r), int(c)) for r, c in candidates]
        points.sort(key=lambda p: math.dist(p[:2], point[:2]))
        for candidate in points:
            if math.dist(candidate[:2], point[:2]) > limit:
                break
            if self.allows_segment(point, candidate, raw=True):
                return candidate
        raise RuntimeError(
            "No nearby robot-clearance map cell is reachable without crossing a wall"
        )

    def _search(self, source, target):
        height, width = self.mask.shape
        if target != self._goal:
            self._goal = target
            self._distances = np.full(self.mask.shape, np.inf)
            self._next = np.full(self.mask.shape, -1, dtype=np.int32)
            self._settled = np.zeros(self.mask.shape, dtype=bool)
            self._distances[target] = 0
            self._heap = [(0.0, *target)]
        while self._heap and not self._settled[source]:
            cost, row, col = heapq.heappop(self._heap)
            if self._settled[row, col]:
                continue
            self._settled[row, col] = True
            for dr, dc in ((-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)):
                nr, nc = row + dr, col + dc
                if not self._inside(self.mask, nr, nc) or self._settled[nr, nc]:
                    continue
                if dr and dc and not (self.mask[row, nc] and self.mask[nr, col]):
                    continue
                proposed = cost + (math.sqrt(2) if dr and dc else 1)
                if proposed < self._distances[nr, nc]:
                    self._distances[nr, nc] = proposed
                    self._next[nr, nc] = row * width + col
                    heapq.heappush(self._heap, (proposed, nr, nc))
        if not self._settled[source]:
            raise RuntimeError("No connected static-map route to the goal")
        cells = [source]
        while cells[-1] != target:
            next_index = int(self._next[cells[-1]])
            if next_index < 0:
                raise RuntimeError("Static-map route reconstruction failed")
            cells.append(divmod(next_index, width))
        return cells

    def shortest_path(self, start, goal):
        if not self.is_allowed_world(start) or not self.is_allowed_world(goal):
            raise RuntimeError("Static-map path endpoints lack robot clearance")
        source, target = self.cell(start), self.cell(goal)
        if self.labels[source] != self.labels[target]:
            raise RuntimeError("Static-map start and goal are in disconnected regions")
        if self.allows_segment(start, goal):
            return [list(start), list(goal)]
        cells = self._search(source, target)
        points = [list(start), *[self.world_point(r, c) for r, c in cells], list(goal)]
        # Remove collinear grid runs before checking longer visible shortcuts.
        corners = [points[0]]
        for i in range(1, len(points) - 1):
            a, b, c = points[i - 1], points[i], points[i + 1]
            cross = (b[0] - a[0]) * (c[1] - b[1]) - (b[1] - a[1]) * (c[0] - b[0])
            if abs(cross) > 1e-12:
                corners.append(b)
        corners.append(points[-1])
        result, index = [corners[0]], 0
        while index < len(corners) - 1:
            next_index = index + 1
            for candidate in range(len(corners) - 1, index, -1):
                if self.allows_segment(corners[index], corners[candidate]):
                    next_index = candidate
                    break
            if not self.allows_segment(corners[index], corners[next_index]):
                raise RuntimeError("Static-map shortcut crosses a blocked cell")
            result.append(corners[next_index])
            index = next_index
        return result


def configured_static_map(bundle, scene):
    map_file = bundle["robot_task"].get("map_file")
    if not map_file:
        return None
    path = Path(map_file).expanduser()
    if not path.is_absolute():
        path = Path(bundle["config_root"]).parent / path
    generation = bundle["pedestrians"].get("generation") or {}
    region = generation.get("scene", {}).get("allowed_region_xy")
    return StaticMapPath(
        path, bundle["robot_task"]["clearance_m"], getattr(scene, "floor_z", 0.0) or 0.0, region
    )
