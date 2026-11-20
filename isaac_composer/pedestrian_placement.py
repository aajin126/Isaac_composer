"""Shared grid candidates and robot-route placement for random and semantic actors.

This module has no Cosmos dependency. Actual routes still use native NavMesh queries.
"""

import math
import random
from collections import Counter

DEFAULT_ROUTE_BAND_HALF_WIDTH_M = 3.0
DEFAULT_PLACEMENT_CELL_SIZE_M = 2.0
DEFAULT_ROUTE_BAND_FRACTION = 0.7


def segment_distance(point, start, end):
    dx, dy = end[0] - start[0], end[1] - start[1]
    length2 = dx * dx + dy * dy
    t = (
        0.0
        if length2 == 0
        else max(0.0, min(1.0, ((point[0] - start[0]) * dx + (point[1] - start[1]) * dy) / length2))
    )
    return math.hypot(point[0] - start[0] - t * dx, point[1] - start[1] - t * dy)


def segments_distance(a, b, c, d):
    def cross(p, q, r):
        return (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0])

    overlap = all(
        max(min(a[k], b[k]), min(c[k], d[k])) <= min(max(a[k], b[k]), max(c[k], d[k]))
        for k in (0, 1)
    )
    if overlap and cross(a, b, c) * cross(a, b, d) <= 0 and cross(c, d, a) * cross(c, d, b) <= 0:
        return 0.0
    return min(
        segment_distance(a, c, d),
        segment_distance(b, c, d),
        segment_distance(c, a, b),
        segment_distance(d, a, b),
    )


def robot_reference_path(task):
    """Use the task's validated reference path; never invent a straight fallback."""
    points = task.get("reference_path_world", [])
    if len(points) < 2:
        raise ValueError(
            "Robot placement requires reference_path_world from the robot path planner"
        )
    path = [list(map(float, point)) for point in points]
    if any(len(point) < 2 or not all(math.isfinite(v) for v in point) for point in path):
        raise ValueError("Robot reference path must contain finite world coordinates")
    if (
        math.dist(path[0][:2], task["spawn_world"][:2]) > 0.05
        or math.dist(path[-1][:2], task["goal_world"][:2]) > 0.05
    ):
        raise ValueError("Robot reference path does not match the task start and goal")
    return path


def polyline_distance(point, path):
    return min(segment_distance(point, a, b) for a, b in zip(path, path[1:]))


def segment_path_distance(start, end, path):
    return min(segments_distance(start, end, a, b) for a, b in zip(path, path[1:]))


class PedestrianPlacement:
    def __init__(
        self,
        grid,
        seed=42,
        robot_task=None,
        route_band_half_width_m=DEFAULT_ROUTE_BAND_HALF_WIDTH_M,
        route_band_fraction=DEFAULT_ROUTE_BAND_FRACTION,
    ):
        if not math.isfinite(route_band_half_width_m) or route_band_half_width_m <= 0:
            raise ValueError("Route band half-width must be finite and positive")
        if not math.isfinite(route_band_fraction) or not 0 <= route_band_fraction <= 1:
            raise ValueError("Route band fraction must be finite and within [0, 1]")
        self.grid, self.seed = grid, seed
        self.cells = {c["id"]: c["target_world"] for c in grid["cells"]}
        self.robot_task, self.width = robot_task, float(route_band_half_width_m)
        self.robot_path = robot_reference_path(robot_task) if robot_task else []
        self.fraction = float(route_band_fraction)
        self.regions = {}
        x0, y0, x1, y1 = grid["bounds_xy"]
        for cid, point in self.cells.items():
            col = min(2, max(0, int(3 * (point[0] - x0) / max(x1 - x0, 1e-6))))
            row = min(2, max(0, int(3 * (point[1] - y0) / max(y1 - y0, 1e-6))))
            self.regions[cid] = (
                ["south", "middle", "north"][row] + "_" + ["west", "center", "east"][col]
            )
        self.priority_cells = [
            cid
            for cid, point in self.cells.items()
            if robot_task and self.route_distance(point) <= self.width
        ]
        self._priority = set(self.priority_cells)

    def route_distance(self, point):
        if self.robot_task is None:
            return 0.0
        return polyline_distance(point, self.robot_path)

    def route_actor_ids(self, count):
        """Fixed-size assignment per episode, independent of planner/call order."""
        if self.robot_task is None:
            return set()
        number = min(count, math.floor(count * self.fraction + 0.5))
        return set(random.Random(self.seed ^ 0x504C4143).sample(range(count), number))

    def group(self, actor_id, count):
        return "route_band" if actor_id in self.route_actor_ids(count) else "global_random"

    def region_groups(self):
        groups = {}
        for cid, rid in self.regions.items():
            groups.setdefault(rid, []).append(cid)
        return [{"region_id": rid, "cells": ids} for rid, ids in groups.items()]

    def start_candidates(self, available, assigned, actor_id, count):
        near_route = self.group(actor_id, count) == "route_band"
        occupied = Counter(self.regions[cid] for cid in assigned.values())
        rng = random.Random(self.seed + actor_id * 3571)
        choices = list(available)
        rng.shuffle(choices)
        if self.robot_task:
            minimum = self.robot_task.get("pedestrian_spawn_separation_m", 1.5)
            choices = [
                cid
                for cid in choices
                if math.dist(self.cells[cid][:2], self.robot_task["spawn_world"][:2]) >= minimum
            ]
            if near_route:
                band = [cid for cid in choices if cid in self._priority]
                if band:
                    choices = band
        cap = max(2, math.ceil(count / 2))
        uncrowded = [cid for cid in choices if occupied[self.regions[cid]] < cap]
        if uncrowded:
            choices = uncrowded
        spacing = self.grid["cell_size_m"] * 1.5

        def rank(cid):
            distance = min(
                (
                    math.dist(self.cells[cid][:2], self.cells[other][:2])
                    for other in assigned.values()
                ),
                default=0.0,
            )
            band_distance = (
                self.route_distance(self.cells[cid])
                if near_route and cid not in self._priority
                else 0.0
            )
            return (band_distance, occupied[self.regions[cid]], -min(distance, spacing))

        choices.sort(key=rank)
        return choices[:32]

    def goal_candidates(self, start, minimum_distance_m, actor_id, count):
        choices = [
            cid
            for cid, point in self.cells.items()
            if math.dist(start[:2], point[:2]) >= minimum_distance_m
        ]
        if self.group(actor_id, count) == "global_random":
            return choices
        band = [cid for cid in choices if cid in self._priority]
        if band:
            return band
        choices.sort(key=lambda cid: self.route_distance(self.cells[cid]))
        return choices[:32]

    def cell_for_position(self, position):
        return next(
            (
                cid
                for cid, point in self.cells.items()
                if math.dist(position[:2], point[:2]) <= 0.001
            ),
            None,
        )

    def to_dict(self):
        return {
            "schema_version": 3,
            "frame_id": "world",
            "policy": "mix_robot_route_band_and_global_random",
            "reference_path_source": (self.robot_task or {}).get(
                "reference_path_source", "native_navmesh"
            ),
            "reference_path_world": self.robot_path,
            "route_band_half_width_m": self.width,
            "route_band_fraction": self.fraction,
            "robot_task": self.robot_task,
            "grid": self.grid,
            "priority_cells": self.priority_cells,
        }
