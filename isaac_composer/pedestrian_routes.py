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
    def __init__(self, scene, config, seed=42, robot_positions=(), placement=None):
        self.scene = scene
        self.config = config
        self.random = random.Random(seed)
        self.robot_positions = list(robot_positions)
        self.raster = None
        self.candidates = None
        self.placement = placement

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
        """Use the shared placement pool, or whole-mesh sampling without a robot task."""
        attempts = int(self.config.get("max_attempts_per_person", 500))
        minimum = float(self.config.get("min_route_length_m", 3.0))
        low, high = self.config.get("speed_range_mps", [0.6, 1.4])
        options = None
        if self.placement:
            assigned = {
                i: self.placement.cell_for_position(p["pose"]["position"])
                for i, p in enumerate(people)
            }
            assigned = {i: cid for i, cid in assigned.items() if cid is not None}
            available = [cid for cid in self.placement.cells if cid not in assigned.values()]
            options = self.placement.start_candidates(available, assigned, index, count)
            options = [cid for cid in options if self._separated(self.placement.cells[cid], people)]
        last_error = None
        for _ in range(attempts):
            try:
                if options is None:
                    start, goal = self._sample(), self._sample()
                else:
                    if not options:
                        raise RuntimeError(
                            "No separated start cells remain in the shared placement pool"
                        )
                    start = self.placement.cells[self.random.choice(options)]
                    goals = self.placement.goal_candidates(start, minimum, index, count)
                    if not goals:
                        raise RuntimeError("No shared placement goal is far enough from the start")
                    goal = self.placement.cells[self.random.choice(goals)]
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
    task_config = bundle.get("robot_task", {})
    if bundle["pedestrians"].get("social_navigation", {}).get("enabled"):
        bundle["pedestrians"]["navmesh"]["dynamic_avoidance"] = False
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
    social = bundle["pedestrians"].get("social_navigation", {})
    if social.get("enabled"):
        body_radius = max(
            [float(config.get("radius", 0.3))]
            + [float(p.get("radius", 0.3)) for p in bundle["pedestrians"]["people"]]
        )
        # Bake the pedestrian footprint; queries may request larger
        # robot radii. A robot task must not erode away pedestrian wall space.
        scene.agent_radius = body_radius
        scene.routing_radius_m = body_radius + social["animation_path_clearance_m"]
        native_config = getattr(scene, "config", bundle["pedestrians"]["navmesh"])
        native_config["agent_radius"] = body_radius
        native_config["agent_max_radius"] = max(
            float(native_config.get("agent_max_radius", 0.5)),
            scene.routing_radius_m,
            task_config.get("robot_radius_m", 0.0),
        )
        if region is not None:
            # Unknown cells remain blocked; allow each footprint to use its
            # own clearance from occupied cells instead of the shared 0.4m.
            scene.occupancy_region = OccupancyRegion(
                region.path,
                clearance_m=max(
                    scene.routing_radius_m,
                    float(scene_config.get("map_clearance_m", scene.routing_radius_m)),
                ),
            )
    elif task_config.get("enabled"):
        scene.agent_radius = max(scene.agent_radius, task_config["robot_radius_m"])
    scene.bake()
    return scene


def prepare_pedestrian_routes(bundle, package_root, update_app=None):
    config = bundle["pedestrians"].get("generation")
    task_config = bundle.get("robot_task", {})
    if not config and not task_config.get("enabled"):
        return None
    config = config or {}
    scene = build_pedestrian_navmesh(bundle, package_root)
    seed = int(bundle["pedestrians"].get("seed", 42))
    raw = task_config.get(
        "output_dir",
        config.get("output_dir", f'generated/pedestrians/{bundle["scenario"]["name"]}'),
    )
    output = Path(raw).expanduser()
    if not output.is_absolute():
        output = Path(package_root) / output
    semantic_dir = bundle["pedestrians"].get("semantic_plan_dir")
    compiled = None
    if semantic_dir:
        request_path = Path(semantic_dir) / "planning_request.json"
        approval_path = Path(semantic_dir) / "approval.json"
        pending = request_path.is_file() and (
            not approval_path.is_file()
            or json.loads(approval_path.read_text()).get("approved") is not True
        )
        if pending:
            from .robot_task import prepare_robot_task
            from .semantic_scenario.runtime.planning import finish_pending_plan

            task = prepare_robot_task(bundle, scene, output)
            if task is None:
                raise ValueError("Deferred semantic planning requires an enabled robot task")
            task["pedestrian_spawn_separation_m"] = float(config.get("min_robot_separation_m", 1.5))
            finish_pending_plan(semantic_dir, task, update_app, scene)
        from .semantic_scenario.runtime.compiler import load_reviewed_plan, compile_scene_plan

        context, participants, plan = load_reviewed_plan(semantic_dir)
        if context["environment"] != bundle["environment"]["name"]:
            raise RuntimeError("Semantic plan environment does not match the loaded world")
        compiled = compile_scene_plan(scene, context, participants, plan)
    if task_config.get("enabled") and not bundle.get("robot_task_instance"):
        from .robot_task import prepare_robot_task

        if compiled:
            prepare_robot_task(
                bundle,
                scene,
                output,
                excluded_starts=list(compiled["starts"].values()),
                planned_task=context.get("robot_task"),
            )
        else:
            prepare_robot_task(bundle, scene, output)
    robots = [copy.deepcopy(item["instance"]["pose"]["position"]) for item in bundle["robots"]]
    if compiled and bundle.get("robot_task_instance") and "spatial_guidance" in context:
        from .semantic_scenario.spatial import enrich_context

        robot_guidance = context["spatial_guidance"].get("robot", {})
        context = enrich_context(
            context,
            seed,
            bundle["robot_task_instance"],
            robot_guidance.get(
                "route_band_half_width_m",
                robot_guidance.get("straight_route_band_half_width_m", 3.0),
            ),
            robot_guidance.get("route_band_fraction", 1.0),
        )
    if config:
        placement = None
        if compiled and "spatial_guidance" in context:
            from .semantic_scenario.spatial import placement_from_context

            placement = placement_from_context(context)
        elif (
            not compiled
            and config["mode"] == "random"
            and bundle.get("robot_task_instance")
            and bundle["pedestrians"].get("placement")
        ):
            from .pedestrian_placement import PedestrianPlacement
            from .scene_grid import build_scene_grid

            settings = bundle["pedestrians"]["placement"]
            grid = build_scene_grid(scene, settings["cell_size_m"]).to_dict()
            task = bundle["robot_task_instance"]
            placement = PedestrianPlacement(
                grid,
                seed,
                {
                    "spawn_world": task["spawn_world"],
                    "goal_world": task["goal_world"],
                    "reference_path_world": task["reference_path_world"],
                    "reference_path_source": task.get("reference_path_source", "native_navmesh"),
                    "pedestrian_spawn_separation_m": float(
                        config.get("min_robot_separation_m", 1.5)
                    ),
                },
                settings["route_band_half_width_m"],
                settings.get("route_band_fraction", 0.7),
            )
        planner = PedestrianRoutePlanner(scene, config, seed, robots, placement=placement)
        if compiled:
            from .semantic_scenario.runtime.compiler import (
                build_mixed_people,
                save_execution_inputs,
            )

            people = build_mixed_people(compiled, planner)
            save_execution_inputs(output, semantic_dir, context, compiled)
            bundle["pedestrians"]["semantic_execution"] = compiled
        else:
            people = planner.plan()
        social = bundle["pedestrians"].get("social_navigation", {})
        from .crowd.config import MU_DEFINITION
        from .crowd.behavior.stiffness import SocialStiffness

        mu_random = random.Random(seed ^ 0x4D55)
        low, high = social.get("mu_range", [0.0, 1.0])
        for index, person in enumerate(people):
            person["id"] = index
            person["mu"] = mu_random.uniform(low, high)
        stiffness = SocialStiffness(people, social)
        bundle["pedestrians"]["people"] = people
        bundle["pedestrians"]["state_output_dir"] = str(output)
        if placement:
            output.mkdir(parents=True, exist_ok=True)
            details = placement.to_dict()
            details["seed"] = seed
            route_ids = placement.route_actor_ids(len(people))
            details["route_band_actor_ids"] = sorted(route_ids)
            details["global_random_actor_ids"] = sorted(set(range(len(people))) - route_ids)
            details["actors"] = [
                {
                    "actor_id": p["id"],
                    "start_cell": placement.cell_for_position(p["pose"]["position"]),
                    "start_world": p["pose"]["position"],
                    "start_route_distance_m": placement.route_distance(p["pose"]["position"]),
                    "planner_source": p.get("planner_source", "random"),
                    "placement_group": placement.group(p["id"], len(people)),
                }
                for p in people
            ]
            details["starts_within_route_band_count"] = sum(
                actor["start_route_distance_m"] <= placement.width for actor in details["actors"]
            )
            (output / "pedestrian_placement.json").write_text(
                json.dumps(details, indent=2, allow_nan=False)
            )
            print(
                f"PEDESTRIAN_PLACEMENT mode={'cosmos' if compiled else 'random'} "
                f"reference={details['reference_path_source']} "
                f"half_width_m={placement.width} cell_size_m={placement.grid['cell_size_m']} "
                f"route_band_count={len(route_ids)} global_random_count={len(people)-len(route_ids)} "
                f"priority_cells={len(placement.priority_cells)}",
                flush=True,
            )
        # Share this raster with runtime density/wall queries instead of rebaking.
        raster = scene.rasterize(margin_m=0)
        scene.social_raster = raster
        b = raster.bounds
        area = float(
            raster.mask.sum()
            * (b.x_max - b.x_min)
            / raster.width
            * (b.y_max - b.y_min)
            / raster.height
        )
        (output).mkdir(parents=True, exist_ok=True)
        (output / "crowd_config.json").write_text(
            json.dumps(
                {
                    "seed": seed,
                    "social_navigation": social,
                    "walkable_area_m2": area,
                    "pedestrian_count": len(people),
                    "actual_global_density_per_m2": len(people) / max(area, 1e-9),
                    "target_density_per_m2": config.get("target_density_per_m2"),
                    "max_count": config.get("max_count", 30),
                    "mu_definition": MU_DEFINITION,
                    "social_stiffness": stiffness.diagnostics(),
                    "deadlock_recovery_enabled": social["yielding_enabled"],
                    "people": [
                        {
                            "id": p["id"],
                            "name": p["name"],
                            "mu": p["mu"],
                            "radius": p.get("radius", 0.3),
                            "planner_source": p.get("planner_source", "random"),
                            "role": p.get("role", "background_pedestrian"),
                        }
                        for p in people
                    ],
                    "pedestrian_navigation_radius_m": getattr(
                        scene, "routing_radius_m", getattr(scene, "agent_radius", 0.0)
                    ),
                    "navmesh_bake_min_radius_m": getattr(scene, "agent_radius", 0.0),
                },
                indent=2,
            )
        )
        _save_artifacts(output, scene, people, seed, config["mode"], robots)
    return scene
