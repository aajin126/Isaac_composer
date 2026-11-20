"""Throttled static-route guidance for a teleoperated robot."""

from __future__ import annotations

import copy
import math
import numpy as np


def robot_navigation_scene(scene, config):
    """Share the baked mesh, but keep robot footprint queries independent."""
    result = copy.copy(scene)
    result.agent_radius = config["robot_radius_m"]
    result.routing_radius_m = config["robot_radius_m"]
    if hasattr(result, "social_raster"):
        del result.social_raster
    if getattr(scene, "occupancy_region", None) is not None:
        from .occupancy_region import OccupancyRegion

        result.occupancy_region = OccupancyRegion(
            scene.occupancy_region.path, clearance_m=config["clearance_m"]
        )
    return result


class GoalRoute:
    """Display guidance only: never commands the robot or avoids moving people."""

    def __init__(self, scene, task, interval_s=0.5, movement_m=0.25):
        self.static_map = getattr(scene, "path_source", None) == "static_occupancy_map"
        self.scene = scene if self.static_map else robot_navigation_scene(scene, task["config"])
        self.goal = list(task["goal_world"])
        self.points = [list(p) for p in task["reference_path_world"]]
        self.interval_s, self.movement_m = interval_s, movement_m
        self.last_attempt = None
        self.last_position = list(task["spawn_world"][:2])
        self.error = None
        self.start_adjustment_m = 0.0

    @property
    def length_m(self):
        return sum(math.dist(a, b) for a, b in zip(self.points, self.points[1:]))

    def guidance(self, robot_pose, look_ahead_m=0.75):
        """Remaining route length and a forward target, even on dense mesh paths."""
        if len(self.points) < 2:
            return math.dist(robot_pose[:2], self.goal[:2]), self.goal[:2]
        path = np.asarray(self.points)[:, :2]
        position = np.asarray(robot_pose[:2])
        delta = np.diff(path, axis=0)
        lengths = np.linalg.norm(delta, axis=1)
        alpha = np.clip(
            np.sum((position - path[:-1]) * delta, axis=1) / np.maximum(lengths**2, 1e-12), 0, 1
        )
        projections = path[:-1] + alpha[:, None] * delta
        index = int(np.argmin(np.linalg.norm(projections - position, axis=1)))
        remaining = (1 - alpha[index]) * lengths[index] + float(lengths[index + 1 :].sum())
        remaining += float(np.linalg.norm(projections[index] - position))
        target = projections[index].copy()
        budget = look_ahead_m
        for point in path[index + 1 :]:
            distance = float(np.linalg.norm(point - target))
            if distance >= budget:
                target += (point - target) * budget / max(distance, 1e-12)
                break
            budget -= distance
            target = point
        return float(remaining), target

    def update(self, robot_pose, simulation_time):
        """Keep the approved initial path; replan only after meaningful movement."""
        now = float(simulation_time)
        if self.last_attempt is None or now < self.last_attempt:
            self.last_attempt = now
        if now - self.last_attempt < self.interval_s - 1e-9:
            return False
        if self.error is None and math.dist(robot_pose[:2], self.last_position) < self.movement_m:
            return False
        self.last_attempt = now
        self.last_position = list(robot_pose[:2])
        try:
            start = self.scene.project_world([*robot_pose[:2], self.goal[2]])
            adjustment = math.dist(start[:2], robot_pose[:2])
            if not self.static_map and adjustment > 0.35:
                raise RuntimeError("Robot is outside navigable space for its footprint")
            points = self.scene.shortest_path(start, self.goal)
            if len(points) < 2 or math.dist(points[-1][:2], self.goal[:2]) > 0.05:
                raise RuntimeError("Route does not reach the goal")
            if self.static_map and adjustment > 1e-6:
                points = [[*robot_pose[:2], self.goal[2]], *points]
            self.points = [list(p) for p in points]
            self.start_adjustment_m = adjustment
            if self.error is not None:
                print("GOAL_ROUTE_RECOVERED", flush=True)
            self.error = None
        except RuntimeError as error:
            # Keep the last valid route, explicitly marked stale by the overlay.
            # Never invent a new direct line through a wall after a failed query.
            if str(error) != self.error:
                print(f"GOAL_ROUTE_UPDATE_PENDING reason={error}", flush=True)
            self.error = str(error)
        return True
