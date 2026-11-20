"""Generate fixed or random pedestrian routes directly on Isaac Sim NavMesh."""

from __future__ import annotations

import copy
import json
import math
import random
from pathlib import Path

import numpy as np

from .navmesh import NavMeshScene, world_xy_to_normalized


def _distance(a, b):
    return math.dist(a[:2], b[:2])


class PedestrianRoutePlanner:
    def __init__(self, scene, config, seed=42, robot_positions=()):
        self.scene = scene
        self.config = config
        self.random = random.Random(seed)
        self.robot_positions = list(robot_positions)
        self.raster = None
        self.candidates = None

    def _project(self, point):
        projected = self.scene.project_world(point)
        if _distance(point, projected) > float(self.config.get("max_projection_m", 0.25)):
            raise RuntimeError(f"Point {point} is too far from walkable NavMesh")
        return list(projected)

    def _sample(self):
        if self.raster is None:
            self.raster = self.scene.rasterize(margin_m=0)
            self.candidates = np.argwhere(self.raster.mask)
        if not len(self.candidates):
            raise RuntimeError("No walkable points are available for pedestrians")
        row, col = self.candidates[self.random.randrange(len(self.candidates))]
        return self._project(self.raster.pixel_to_world(int(row), int(col)))

    def _route(self, start, goal):
        points = self.scene.shortest_path(start, goal)
        if (
            len(points) < 2
            or _distance(points[0], start) > 0.05
            or _distance(points[-1], goal) > 0.05
        ):
            raise RuntimeError("NavMesh path does not reach the requested endpoints")
        route = [list(points[0])]
        for point in points[1:]:
            if math.dist(route[-1], point) > 1e-4:
                route.append(list(point))
        if len(route) < 2:
            raise RuntimeError("Pedestrian start and goal must be distinct")
        return route

    def _separated(self, start, people):
        separation = float(self.config.get("min_spawn_separation_m", 1.0))
        separation = max(separation, 2 * float(self.config.get("radius", 0.3)) + 0.2)
        robot_separation = float(self.config.get("min_robot_separation_m", 1.5))
        return all(_distance(start, p["pose"]["position"]) >= separation for p in people) and all(
            _distance(start, robot) >= robot_separation for robot in self.robot_positions
        )

    def _person(self, index, route, speed):
        direction = next(
            (point for point in route[1:] if _distance(route[0], point) > 1e-4), route[0]
        )
        yaw = math.atan2(direction[1] - route[0][1], direction[0] - route[0][0])
        characters = self.config.get(
            "characters", ["F_Business_02", "male_adult_construction_01_new"]
        )
        return {
            "name": f"Character_{index}",
            "character": self.random.choice(characters),
            "pose": {"position": route[0], "yaw": yaw},
            "radius": float(self.config.get("radius", 0.3)),
            "height": float(self.config.get("height", 1.7)),
            "walk_speed": speed,
            "waypoints": route[1:],
            "repeat": bool(self.config.get("repeat", False)),
        }

    def plan(self):
        if self.config["mode"] == "fixed":
            start = self._project(self.config["start"])
            goal = self._project(self.config["goal"])
            if not self._separated(start, []):
                raise RuntimeError("Fixed pedestrian start is too close to the robot")
            return [
                self._person(0, self._route(start, goal), float(self.config.get("speed_mps", 1.0)))
            ]

        people = []
        count = int(self.config.get("count", 5))
        if "target_density_per_m2" in self.config:
            raster = self.scene.rasterize(margin_m=0)
            b = raster.bounds
            area = float(
                raster.mask.sum()
                * (b.x_max - b.x_min)
                / raster.width
                * (b.y_max - b.y_min)
                / raster.height
            )
            count = min(
                int(self.config.get("max_count", 30)),
                max(1, round(area * self.config["target_density_per_m2"])),
            )
        for index in range(count):
            people.append(self.random_person(index, people, count))
        return people

    def random_person(self, index, people, count):
        """Sample separated endpoints on the whole walkable mesh."""
        attempts = int(self.config.get("max_attempts_per_person", 500))
        minimum = float(self.config.get("min_route_length_m", 3.0))
        low, high = self.config.get("speed_range_mps", [0.6, 1.4])
        last_error = None
        for _ in range(attempts):
            try:
                start, goal = self._sample(), self._sample()
                if not self._separated(start, people) or _distance(start, goal) < minimum:
                    continue
                return self._person(index, self._route(start, goal), self.random.uniform(low, high))
            except RuntimeError as error:
                last_error = error
        raise RuntimeError(
            f"Could not generate pedestrian {index + 1}/{count} "
            f"after {attempts} attempts; last path error: {last_error}"
        )


def _save_artifacts(output, scene, people, seed, mode, robot_positions):
    from PIL import Image, ImageDraw

    output.mkdir(parents=True, exist_ok=True)
    (output / "pedestrian_routes.json").write_text(
        json.dumps(
            {"seed": seed, "mode": mode, "robot_positions": robot_positions, "people": people},
            indent=2,
        )
    )
    raster = scene.rasterize(margin_m=0)
    scale = 3
    image = Image.fromarray(np.where(raster.mask, 238, 28).astype(np.uint8)).convert("RGB")
    image = image.resize(
        (raster.width * scale, raster.height * scale), getattr(Image, "Resampling", Image).NEAREST
    )
    image.save(output / "00_free_space.png")
    draw = ImageDraw.Draw(image)

    def pixel(point):
        u, v = world_xy_to_normalized(point[:2], raster.bounds)
        return (round(u * image.width), round(v * image.height))

    colors = ["#e53935", "#1e88e5", "#43a047", "#fb8c00", "#8e24aa", "#00acc1"]
    for index, person in enumerate(people):
        points = [pixel(person["pose"]["position"]), *map(pixel, person["waypoints"])]
        color = colors[index % len(colors)]
        draw.line(points, fill=color, width=3)
        x, y = points[0]
        draw.ellipse((x - 5, y - 5, x + 5, y + 5), fill=color)
        draw.text((x + 7, y), person["name"], fill=color)
        x, y = points[-1]
        draw.rectangle((x - 4, y - 4, x + 4, y + 4), outline=color, width=2)
    for robot in robot_positions:
        x, y = pixel(robot)
        draw.text((x, y), "Robot", fill="#ff00ff")
    image.save(output / "01_pedestrian_routes.png")


def build_pedestrian_navmesh(bundle, package_root):
    """Build the shared pedestrian footprint without spawning or planning actors."""
    config = bundle["pedestrians"].get("generation") or {}
    scene_config = config.get("scene", {})
    region = None
    if scene_config.get("occupancy_map"):
        from .occupancy_region import OccupancyRegion

        path = Path(scene_config["occupancy_map"]).expanduser()
        if not path.is_absolute():
            path = Path(package_root) / path
        region = OccupancyRegion(path, clearance_m=float(scene_config.get("map_clearance_m", 0.4)))
    scene = NavMeshScene(
        bundle["pedestrians"]["navmesh"],
        resolution_m=float(scene_config.get("grid_resolution_m", 0.12)),
        floor_z=scene_config.get("floor_z", 0.0),
        floor_tolerance_m=float(scene_config.get("floor_tolerance_m", 0.2)),
        environment_prim_path=bundle["environment"]["asset"]["prim_path"],
        allowed_region_xy=scene_config.get("allowed_region_xy"),
        occupancy_region=region,
    )
    scene.bake()
    return scene


def prepare_pedestrian_routes(bundle, package_root):
    config = bundle["pedestrians"].get("generation")
    if not config:
        return None
    scene = build_pedestrian_navmesh(bundle, package_root)
    seed = int(bundle["pedestrians"].get("seed", 42))
    raw = config.get("output_dir", f'generated/pedestrians/{bundle["scenario"]["name"]}')
    output = Path(raw).expanduser()
    if not output.is_absolute():
        output = Path(package_root) / output
    robots = [copy.deepcopy(item["instance"]["pose"]["position"]) for item in bundle["robots"]]
    planner = PedestrianRoutePlanner(scene, config, seed, robots)
    people = planner.plan()
    bundle["pedestrians"]["people"] = people
    _save_artifacts(output, scene, people, seed, config["mode"], robots)
    return scene
