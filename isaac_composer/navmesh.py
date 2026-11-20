from __future__ import annotations

import math
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Iterable

import numpy as np

@dataclass
class SceneBounds:
    x_min: float
    x_max: float
    y_min: float
    y_max: float
    z_min: float
    z_max: float

    def to_dict(self):
        return asdict(self)


def normalized_to_world_xy(uv, bounds):
    u, v = uv
    return (bounds.x_min + u * (bounds.x_max - bounds.x_min),
            bounds.y_max - v * (bounds.y_max - bounds.y_min))


def world_xy_to_normalized(xy, bounds):
    x, y = xy
    return ((x - bounds.x_min) / (bounds.x_max - bounds.x_min),
            (bounds.y_max - y) / (bounds.y_max - bounds.y_min))


@dataclass
class RasterizedNavMesh:
    mask: np.ndarray
    clearance_m: np.ndarray
    bounds: SceneBounds
    resolution_m: float

    @property
    def height(self) -> int:
        return int(self.mask.shape[0])

    @property
    def width(self) -> int:
        return int(self.mask.shape[1])

    def pixel_to_uv(self, row: int, col: int) -> tuple[float, float]:
        u = (float(col) + 0.5) / max(1, self.width)
        v = (float(row) + 0.5) / max(1, self.height)
        return u, v

    def pixel_to_world(self, row: int, col: int) -> tuple[float, float, float]:
        u, v = self.pixel_to_uv(row, col)
        x, y = normalized_to_world_xy((u, v), self.bounds)
        return x, y, self.bounds.z_min


class NavMeshScene:
    """Bake/query the pedestrian NavMesh and convert it to an exact top-down mask."""

    def __init__(self, navmesh_config: dict[str, Any], resolution_m: float = 0.12, floor_z: float | None = None, floor_tolerance_m: float = 0.8, environment_prim_path: str = "/World/Environment", allowed_region_xy: Iterable[float] | None = None, occupancy_region=None) -> None:
        self.config = navmesh_config
        self.resolution_m = float(resolution_m)
        self.floor_z = floor_z
        self.floor_tolerance_m = float(floor_tolerance_m)
        self.environment_prim_path = environment_prim_path
        self.occupancy_region = occupancy_region
        self.allowed_region_xy = tuple(map(float, allowed_region_xy)) if allowed_region_xy is not None else None
        if self.allowed_region_xy is not None:
            region = self.allowed_region_xy
            if (len(region) != 4 or not all(math.isfinite(v) for v in region)
                    or region[0] >= region[2] or region[1] >= region[3]):
                raise ValueError("allowed_region_xy must be [x_min, y_min, x_max, y_max]")
        self.navigation = None
        self.navmesh = None
        self.agent_radius = float(navmesh_config.get("agent_radius", 0.4))
        self.agent_height = float(navmesh_config.get("agent_height", 1.8))

    def is_allowed_world(self, xyz: Iterable[float]) -> bool:
        x, y, z = map(float, xyz)
        if not all(math.isfinite(value) for value in (x, y, z)):
            return False
        if self.floor_z is not None and abs(z - self.floor_z) > self.floor_tolerance_m:
            return False
        if self.allowed_region_xy is not None:
            x_min, y_min, x_max, y_max = self.allowed_region_xy
            if not (x_min <= x <= x_max and y_min <= y <= y_max):
                return False
        return self.occupancy_region is None or bool(self.occupancy_region.allows_xy(x, y))

    def _ensure_include_volume(self) -> None:
        import NavSchema
        import omni.kit.app
        import omni.usd
        from pxr import Gf, Usd, UsdGeom

        stage = omni.usd.get_context().get_stage()
        for prim in stage.Traverse():
            if prim.IsA(NavSchema.NavMeshVolume):
                if NavSchema.NavMeshVolume(prim).GetNavVolumeTypeAttr().Get() == "Include":
                    return
        if not self.config.get("auto_include_volume", True):
            raise RuntimeError("Environment has no NavMesh Include Volume")
        environment = stage.GetPrimAtPath(self.environment_prim_path)
        if not environment.IsValid():
            raise RuntimeError(f"Environment prim is missing: {self.environment_prim_path}")
        bounds = UsdGeom.BBoxCache(
            Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render]
        ).ComputeWorldBound(environment).ComputeAlignedRange()
        if bounds.IsEmpty():
            raise RuntimeError("Environment geometry is not loaded; cannot create NavMesh volume")
        margin = float(self.config.get("include_volume_margin_m", 0.5))
        minimum = bounds.GetMin() - Gf.Vec3d(margin)
        maximum = bounds.GetMax() + Gf.Vec3d(margin)
        volume = NavSchema.NavMeshVolume.Define(stage, "/World/PedestrianNavMeshVolume")
        volume.GetNavVolumeTypeAttr().Set("Include")
        UsdGeom.Boundable(volume.GetPrim()).CreateExtentAttr([
            Gf.Vec3f(*minimum), Gf.Vec3f(*maximum)
        ])
        omni.kit.app.get_app().update()

    def bake(self) -> None:
        import omni.anim.navigation.core as nav

        from isaac_composer.pedestrian.simulator.logic.people_manager import PeopleManager

        manager = PeopleManager.get_people_manager()
        manager.rebuild_nav_mesh(
            height=float(self.config.get("agent_height", 1.8)),
            radius=float(self.config.get("agent_radius", 0.4)),
            auto_rebake_on_changes=False,
            auto_rebake_delay_seconds=int(self.config.get("auto_rebake_delay_seconds", 4)),
            exclude_rigid_bodies=bool(self.config.get("exclude_rigid_bodies", True)),
            view_nav_mesh=bool(self.config.get("view", False)),
            dynamic_avoidance_enabled=bool(self.config.get("dynamic_avoidance", True)),
            navmesh_enabled=bool(self.config.get("enabled", True)),
        )
        self._ensure_include_volume()
        import carb.settings

        settings = carb.settings.get_settings()
        # Bake settings use centimeters, independently of the USD stage units.
        for path, value in (
            (nav.NavMeshSettings.AGENT_MIN_HEIGHT_SETTING_PATH, self.agent_height * 100.0),
            (nav.NavMeshSettings.AGENT_MIN_RADIUS_SETTING_PATH, self.agent_radius * 100.0),
            (nav.NavMeshSettings.AGENT_MAX_RADIUS_SETTING_PATH, max(self.agent_radius, float(self.config.get("agent_max_radius", 0.5))) * 100.0),
            (nav.NavMeshSettings.AGENT_MAX_STEP_HEIGHT_SETTING_PATH, float(self.config.get("agent_max_step_height", 0.2)) * 100.0),
            (nav.NavMeshSettings.AGENT_MAX_FLOOR_SLOPE_SETTING_PATH, float(self.config.get("agent_max_floor_slope", 45.0))),
            (nav.NavMeshSettings.AGENT_MIN_ISLAND_RADIUS_SETTING_PATH, float(self.config.get("agent_min_island_radius_m", 0.5)) * 100.0),
            (nav.NavMeshSettings.AUTO_REBAKE_SETTING_PATH, False),
            (nav.NavMeshSettings.EXCLUDE_RIGID_BODIES_PATH, bool(self.config.get("exclude_rigid_bodies", True))),
        ):
            settings.set(path, value)
        self.navigation = nav.acquire_interface()
        if not self.navigation.start_navmesh_baking_and_wait():
            raise RuntimeError("NavMesh baking failed")
        self.navmesh = self.navigation.get_navmesh()
        if self.navmesh is None:
            raise RuntimeError(
                "No baked NavMesh is available. Add/configure a NavMesh Include Volume "
                "for the environment, then run the pedestrian scenario again."
            )

    @staticmethod
    def _xyz(point: Any) -> tuple[float, float, float]:
        return float(point[0]), float(point[1]), float(point[2])

    def _all_triangles(self) -> list[tuple[tuple[float, float, float], ...]]:
        if self.navmesh is None:
            raise RuntimeError("NavMesh has not been baked")
        triangles: list[tuple[tuple[float, float, float], ...]] = []
        area_count = int(self.navmesh.get_area_count())
        for area_index in range(area_count):
            area_name = str(self.navmesh.get_area_name(area_index)).lower()
            if "notwalkable" in area_name or "not_walkable" in area_name:
                continue
            points = self.navmesh.get_draw_triangles(area_index)
            for index in range(0, len(points) - 2, 3):
                triangle = (
                    self._xyz(points[index]),
                    self._xyz(points[index + 1]),
                    self._xyz(points[index + 2]),
                )
                if self.floor_z is not None:
                    if any(abs(point[2] - float(self.floor_z)) > self.floor_tolerance_m for point in triangle):
                        continue
                if self.allowed_region_xy is not None:
                    x_min, y_min, x_max, y_max = self.allowed_region_xy
                    if (max(p[0] for p in triangle) < x_min or min(p[0] for p in triangle) > x_max
                            or max(p[1] for p in triangle) < y_min or min(p[1] for p in triangle) > y_max):
                        continue
                triangles.append(triangle)
        if not triangles:
            raise RuntimeError("Baked NavMesh contains no drawable triangles")
        return triangles

    def bounds(self, margin_m: float = 0.5) -> SceneBounds:
        triangles = self._all_triangles()
        xyz = np.asarray([point for tri in triangles for point in tri], dtype=np.float64)
        bounds = SceneBounds(
            x_min=float(xyz[:, 0].min() - margin_m),
            x_max=float(xyz[:, 0].max() + margin_m),
            y_min=float(xyz[:, 1].min() - margin_m),
            y_max=float(xyz[:, 1].max() + margin_m),
            z_min=float(xyz[:, 2].min()),
            z_max=float(xyz[:, 2].max()),
        )
        if self.allowed_region_xy is not None:
            x_min, y_min, x_max, y_max = self.allowed_region_xy
            bounds.x_min, bounds.y_min = max(bounds.x_min, x_min), max(bounds.y_min, y_min)
            bounds.x_max, bounds.y_max = min(bounds.x_max, x_max), min(bounds.y_max, y_max)
            if bounds.x_min >= bounds.x_max or bounds.y_min >= bounds.y_max:
                raise RuntimeError("Allowed interior region contains no NavMesh")
        return bounds

    def walkable_triangles(self):
        """Native baked floor triangles, before any image/grid conversion."""
        return self._all_triangles()

    @staticmethod
    def _fill_triangle(mask: np.ndarray, xy_px: np.ndarray) -> None:
        """Rasterize one triangle with a vectorized barycentric test."""
        height, width = mask.shape
        xs = xy_px[:, 0]
        ys = xy_px[:, 1]
        c0 = max(0, int(math.floor(xs.min())))
        c1 = min(width - 1, int(math.ceil(xs.max())))
        r0 = max(0, int(math.floor(ys.min())))
        r1 = min(height - 1, int(math.ceil(ys.max())))
        if c1 < c0 or r1 < r0:
            return

        cols = np.arange(c0, c1 + 1, dtype=np.float64) + 0.5
        rows = np.arange(r0, r1 + 1, dtype=np.float64) + 0.5
        xx, yy = np.meshgrid(cols, rows)

        x1, y1 = xy_px[0]
        x2, y2 = xy_px[1]
        x3, y3 = xy_px[2]
        denominator = (y2 - y3) * (x1 - x3) + (x3 - x2) * (y1 - y3)
        if abs(denominator) < 1e-12:
            return
        a = ((y2 - y3) * (xx - x3) + (x3 - x2) * (yy - y3)) / denominator
        b = ((y3 - y1) * (xx - x3) + (x1 - x3) * (yy - y3)) / denominator
        c = 1.0 - a - b
        inside = (a >= -1e-6) & (b >= -1e-6) & (c >= -1e-6)
        mask[r0 : r1 + 1, c0 : c1 + 1] |= inside

    def rasterize(self, margin_m: float = 0.5) -> RasterizedNavMesh:
        from scipy import ndimage

        triangles = self._all_triangles()
        bounds = self.bounds(margin_m=margin_m)
        width = max(2, int(math.ceil((bounds.x_max - bounds.x_min) / self.resolution_m)))
        height = max(2, int(math.ceil((bounds.y_max - bounds.y_min) / self.resolution_m)))
        mask = np.zeros((height, width), dtype=bool)

        def world_to_pixel_xy(x: float, y: float) -> tuple[float, float]:
            u, v = world_xy_to_normalized((x, y), bounds)
            return u * width, v * height

        for tri in triangles:
            px = np.asarray([world_to_pixel_xy(p[0], p[1]) for p in tri], dtype=np.float64)
            self._fill_triangle(mask, px)

        # Fill tiny rasterization gaps but do not bridge real obstacles.
        mask = ndimage.binary_closing(mask, structure=np.ones((3, 3), dtype=bool), iterations=1)
        if self.occupancy_region is not None:
            xs = bounds.x_min + (np.arange(width) + 0.5) / width * (bounds.x_max - bounds.x_min)
            ys = bounds.y_max - (np.arange(height) + 0.5) / height * (bounds.y_max - bounds.y_min)
            xx, yy = np.meshgrid(xs, ys)
            mask &= self.occupancy_region.allows_xy(xx, yy)
            if not mask.any():
                raise RuntimeError("Prebuilt map and NavMesh have no overlapping allowed free space")
        clearance = ndimage.distance_transform_edt(mask) * self.resolution_m
        return RasterizedNavMesh(
            mask=mask.astype(bool),
            clearance_m=clearance.astype(np.float32),
            bounds=bounds,
            resolution_m=self.resolution_m,
        )

    def project_world(self, xyz: Iterable[float], agent_radius: float | None = None, agent_height: float | None = None) -> tuple[float, float, float]:
        import carb
        if self.navmesh is None:
            raise RuntimeError("NavMesh has not been baked")
        values = list(xyz)
        target = carb.Float3(float(values[0]), float(values[1]), float(values[2]))
        point, _island = self.navmesh.query_closest_point(
            target=target,
            agent_radius=float(getattr(self,"routing_radius_m",self.agent_radius) if agent_radius is None else agent_radius),
            agent_height=float(self.agent_height if agent_height is None else agent_height),
        )
        if point is None:
            raise RuntimeError(f"Could not project point to NavMesh: {values}")
        projected = self._xyz(point)
        if not self.is_allowed_world(projected):
            raise RuntimeError(f"NavMesh projection is outside the allowed interior/floor: {projected}")
        return projected

    def project_normalized(self, uv: Iterable[float], bounds: SceneBounds) -> tuple[float, float, float]:
        x, y = normalized_to_world_xy(tuple(uv), bounds)
        z = 0.5 * (bounds.z_min + bounds.z_max)
        return self.project_world((x, y, z))

    def shortest_path(
        self,
        start_xyz: Iterable[float],
        end_xyz: Iterable[float],
        straighten: bool = True,
        agent_radius: float | None = None,
    ) -> list[tuple[float, float, float]]:
        import carb

        if self.navmesh is None:
            raise RuntimeError("NavMesh has not been baked")
        start = list(start_xyz)
        end = list(end_xyz)
        path = self.navmesh.query_shortest_path(
            start_pos=carb.Float3(*map(float, start)),
            end_pos=carb.Float3(*map(float, end)),
            agent_radius=float(getattr(self,"routing_radius_m",self.agent_radius) if agent_radius is None else agent_radius),
            agent_height=self.agent_height,
            straighten=bool(straighten),
        )
        if path is None or int(path.get_point_count()) < 2:
            raise RuntimeError(f"No NavMesh path between {start} and {end}")
        points = [self._xyz(point) for point in path.get_points()]
        if any(not self.is_allowed_world(point) for point in points):
            raise RuntimeError("NavMesh path leaves the allowed interior/floor")
        if self.occupancy_region is not None and any(
            not self.occupancy_region.allows_segment(a, b) for a, b in zip(points[:-1], points[1:])
        ):
            raise RuntimeError("NavMesh path crosses blocked/unknown space in the prebuilt map")
        return points
