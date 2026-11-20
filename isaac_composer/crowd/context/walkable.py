"""Local density uses effective NavMesh area, not a bounding-box area."""

import math
import numpy as np
from scipy import ndimage


class WalkableContext:
    def __init__(self, scene, raster=None):
        self.scene = scene
        self.raster = raster if raster is not None else scene.rasterize(margin_m=0)
        self.mask = self.raster.mask
        b = self.raster.bounds
        self.dx = (b.x_max - b.x_min) / self.raster.width
        self.dy = (b.y_max - b.y_min) / self.raster.height
        self.cell_area = self.dx * self.dy
        self.area_m2 = float(self.mask.sum() * self.cell_area)
        self.clearance = ndimage.distance_transform_edt(
            np.pad(self.mask, 1), sampling=(self.dy, self.dx)
        )[1:-1, 1:-1]
        self._motion_cache = None
        self._projection_cache = None

    def begin_tick(self):
        """Reuse identical static-map checks only within one crowd observation."""
        self._motion_cache = {}
        self._projection_cache = {}

    def _project_world(self, point, radius):
        if self._projection_cache is None:
            return self.scene.project_world(point, agent_radius=radius)
        key = (*map(float, point), float(radius))
        if key not in self._projection_cache:
            self._projection_cache[key] = self.scene.project_world(point, agent_radius=radius)
        return self._projection_cache[key]

    def cell(self, point):
        b = self.raster.bounds
        return int(math.floor((b.y_max - point[1]) / self.dy)), int(
            math.floor((point[0] - b.x_min) / self.dx)
        )

    def allows(self, point):
        row, col = self.cell(point)
        return bool(
            0 <= row < self.raster.height and 0 <= col < self.raster.width and self.mask[row, col]
        )

    def segment_allowed(self, start, end):
        count = max(
            2,
            int(
                math.ceil(
                    np.linalg.norm(np.asarray(end)[:2] - np.asarray(start)[:2])
                    / (min(self.dx, self.dy) * 0.5)
                )
            )
            + 1,
        )
        return all(self.allows(point) for point in np.linspace(start, end, count))

    def motion_allowed(self, start, end, radius, margin):
        if self._motion_cache is None:
            return self._motion_allowed(start, end, radius, margin)
        key = (*map(float, start[:2]), *map(float, end[:2]), float(radius), float(margin))
        if key not in self._motion_cache:
            self._motion_cache[key] = self._motion_allowed(start, end, radius, margin)
        return self._motion_cache[key]

    def _motion_allowed(self, start, end, radius, margin):
        """Check native NavMesh near raster edges without relaxing body clearance.

        The baked center mesh is already eroded by scene.agent_radius. Permit
        at most one cell of tracking error within the difference to physical
        radius, and retain the scene's map/floor checks. The animation's path
        projection error must not increase along the proposed movement.
        """
        if not hasattr(self.scene, "project_world"):
            return self.segment_allowed(start, end)
        navigation_radius = getattr(self.scene, "routing_radius_m", self.scene.agent_radius)
        budget = max(0.0, min(min(self.dx, self.dy), navigation_radius - radius))
        count = max(
            2,
            int(
                np.ceil(
                    np.linalg.norm(np.asarray(end)[:2] - np.asarray(start)[:2])
                    / (min(self.dx, self.dy) * 0.5)
                )
            )
            + 1,
        )
        route_radius = getattr(self.scene, "routing_radius_m", None)
        initial_route_error = 0.0
        if route_radius is not None:
            origin = np.r_[np.asarray(start)[:2], getattr(self.scene, "floor_z", 0.0) or 0.0]
            try:
                projected = self._project_world(origin, route_radius)
                initial_route_error = float(np.linalg.norm(np.asarray(projected)[:2] - origin[:2]))
            except RuntimeError:
                return False
        for xy in np.linspace(np.asarray(start)[:2], np.asarray(end)[:2], count):
            xyz = np.r_[xy, getattr(self.scene, "floor_z", 0.0) or 0.0]
            if hasattr(self.scene, "is_allowed_world") and not self.scene.is_allowed_world(xyz):
                return False
            if route_radius is not None:
                try:
                    projected_route = self._project_world(xyz, route_radius)
                    # Native animation preview uses this radius too. Do not
                    # call a velocity executable if its target is clipped by
                    # that projection. Existing root error may recover inward.
                    error = np.linalg.norm(np.asarray(projected_route)[:2] - xy)
                    if error > max(initial_route_error, 0.001) + 1e-6:
                        return False
                except RuntimeError:
                    return False
            if self.allows(xy):
                continue
            try:
                projected = self._project_world(xyz, self.scene.agent_radius)
                if np.linalg.norm(np.asarray(projected)[:2] - xy) > budget + 1e-6:
                    return False
            except RuntimeError:
                return False
        return True

    def local(self, position, positions, radius):
        row, col = self.cell(position)
        rows = np.arange(
            max(0, row - math.ceil(radius / self.dy)),
            min(self.raster.height, row + math.ceil(radius / self.dy) + 1),
        )
        cols = np.arange(
            max(0, col - math.ceil(radius / self.dx)),
            min(self.raster.width, col + math.ceil(radius / self.dx) + 1),
        )
        b = self.raster.bounds
        xx = b.x_min + (cols + 0.5) * self.dx
        yy = b.y_max - (rows + 0.5) * self.dy
        circle = (xx[None, :] - position[0]) ** 2 + (yy[:, None] - position[1]) ** 2 <= radius**2
        area = float(np.sum(self.mask[np.ix_(rows, cols)] & circle) * self.cell_area)
        count = int(
            np.sum(np.linalg.norm(np.asarray(positions)[:, :2] - position[:2], axis=1) <= radius)
        )
        density = count / max(area, self.cell_area)
        clearance = (
            self.clearance[row, col]
            if 0 <= row < self.raster.height and 0 <= col < self.raster.width
            else 0
        )
        width = 2 * (clearance + getattr(self.scene, "agent_radius", 0.0))
        return density, area, float(width)
