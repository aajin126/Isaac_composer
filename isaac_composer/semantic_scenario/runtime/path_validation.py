"""Main-thread native path checks and reachable alternatives for local repair."""

import math
from ..schema import NativeRouteError, PlanningRejected
from .compiler import compile_scene_plan, native_path


def inspect_native_paths(scene, context, people, plan):
    try:
        compile_scene_plan(scene, context, people, plan)
        return None
    except NativeRouteError as error:
        actor = next(
            a
            for s in plan["scenes"]
            if s["scene_id"] == error.scene_id
            for a in s["actor_plans"]
            if a["actor_id"] == error.actor_id
        )
        if error.path[0] == "subgoals" and actor["subgoals"][error.path[1]]["action"] == "Queue":
            raise PlanningRejected(
                "A native Queue approach failed; queue placement must be replanned"
            ) from error
        candidates = []
        lookup = {c["id"]: c for c in context["grid"]["cells"]}
        priority = set(
            context.get("spatial_guidance", {}).get("robot", {}).get("priority_cells", [])
        )
        order = sorted(context["valid_cells"], key=lambda cid: cid not in priority)
        for cid in order:
            cell = lookup[cid]
            try:
                if math.dist(error.start[:2], cell["target_world"][:2]) < 1.0:
                    continue
                if not scene.is_allowed_world(cell["target_world"]):
                    continue
                native_path(scene, error.start, cell["target_world"])
                candidates.append(cell["id"])
            except (RuntimeError, ValueError):
                continue
        if not candidates:
            raise PlanningRejected(
                f"No native-reachable replacement for actor {error.actor_id}"
            ) from error
        if error.path == ["goal_cell"]:
            origin = lookup[actor["start_cell"]]["target_world"]
            dwell = sum(s["duration_s"] for s in actor["subgoals"])
            required = max(3.0, context["grid"]["cell_size_m"] * 1.5, dwell * 2 / 3)
            far = [
                cid
                for cid in candidates
                if math.dist(origin[:2], lookup[cid]["target_world"][:2]) >= required
            ]
            if far:
                candidates = far
        return {
            "actor_id": error.actor_id,
            "scene_id": error.scene_id,
            "field_path": error.path,
            "available_cells": candidates,
            "reason": str(error),
        }
