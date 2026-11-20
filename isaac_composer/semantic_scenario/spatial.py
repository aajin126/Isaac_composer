"""Robot-relative planning addresses and placement guidance, without new pathfinding."""

import copy
import math
import random

from ..pedestrian_placement import (
    DEFAULT_ROUTE_BAND_HALF_WIDTH_M,
    DEFAULT_ROUTE_BAND_FRACTION,
    PedestrianPlacement,
    segment_distance,
    segments_distance,
)


def enrich_context(
    context,
    seed,
    robot_task=None,
    route_band_half_width_m=DEFAULT_ROUTE_BAND_HALF_WIDTH_M,
    route_band_fraction=DEFAULT_ROUTE_BAND_FRACTION,
):
    if not math.isfinite(route_band_half_width_m) or route_band_half_width_m <= 0:
        raise ValueError("Route band half-width must be finite and positive")
    context = copy.deepcopy(context)
    cells = context["grid"]["cells"]
    rng = random.Random(seed)
    context["valid_cells"] = [c["id"] for c in cells]
    rng.shuffle(context["valid_cells"])
    placement = PedestrianPlacement(
        context["grid"], seed, robot_task, route_band_half_width_m, route_band_fraction
    )
    guidance = {
        "regions": placement.region_groups(),
        "placement_goal": "Spread small groups and independent walkers; avoid one row along one boundary.",
        "activity_goal": "Most actors should walk meaningful distances, with only brief purpose-driven stops.",
        "queue_policy": "Queue is optional and needs a credible service purpose. Do not make everyone queue.",
        "after_story": "Return along the planned route and repeat the same movement and activities throughout the episode. Independent walkers repeat independently; joint activities retain their synchronization.",
        "minimum_moving_fraction": 0.6,
        "minimum_near_route_moving_fraction": min(0.5, placement.fraction),
        "maximum_queue_fraction": 0.3,
        "movement_note": "At least 60 percent of the planned cast should travel at least 3 meters. Assigned route_band actors should move through priority_cells; global_random actors may move anywhere in valid cells. Queue is a minority activity.",
        "ordinary_pause_budget_s_per_actor_scene": 6.0,
        "ordinary_pause_max_s": 4.0,
    }
    if robot_task:
        context["robot_task"] = copy.deepcopy(robot_task)
        start, goal = robot_task["spawn_world"], robot_task["goal_world"]
        width = float(route_band_half_width_m)
        near = placement.priority_cells
        guidance["robot"] = {
            "start_world": start,
            "goal_world": goal,
            "reference_path_world": placement.robot_path,
            "reference_path_source": robot_task.get("reference_path_source", "native_navmesh"),
            "route_band_half_width_m": width,
            "route_band_fraction": placement.fraction,
            "min_start_separation_m": robot_task.get("pedestrian_spawn_separation_m", 1.5),
            "priority_cells": near,
            "placement_goal": "A seeded fraction of actors starts near the static robot SHORTEST PATH in reference_path_world. The remaining actors may start anywhere in valid walkable cells, without route-distance preference. Respect each actor's allowed_start_cells and placement_group; distribute people across their permitted area.",
            "interaction_goal": "Plan plausible crossing, approaching, same-direction walking and short interactions near this band. Nobody deliberately collides with or pursues the robot.",
            "route_note": "reference_path_world is the robot-footprint-aware route through static structures; reference_path_source identifies its planner. This is the episode placement reference, not a prediction of teleoperation. Moving people are excluded. Pedestrians use their own native NavMesh paths and ORCA.",
        }
    context["spatial_guidance"] = guidance
    return context


def start_candidates(context, available, assigned, actor_id, count):
    """Expose common placement options; Cosmos makes the final selection."""
    if "spatial_guidance" not in context:
        return list(available)
    placement = placement_from_context(context)
    return placement.start_candidates(available, assigned, actor_id, count)


def placement_from_context(context):
    robot = context["spatial_guidance"].get("robot")
    task = None
    if robot:
        task = {
            "spawn_world": robot["start_world"],
            "goal_world": robot["goal_world"],
            "reference_path_world": robot.get("reference_path_world")
            or context.get("robot_task", {}).get("reference_path_world", []),
            "reference_path_source": robot.get(
                "reference_path_source",
                context.get("robot_task", {}).get("reference_path_source", "native_navmesh"),
            ),
            "pedestrian_spawn_separation_m": robot["min_start_separation_m"],
        }
    return PedestrianPlacement(
        context["grid"],
        context.get("episode", {}).get("seed", 0),
        task,
        (
            robot.get(
                "route_band_half_width_m",
                robot.get("straight_route_band_half_width_m", DEFAULT_ROUTE_BAND_HALF_WIDTH_M),
            )
            if robot
            else DEFAULT_ROUTE_BAND_HALF_WIDTH_M
        ),
        robot.get("route_band_fraction", 1.0) if robot else DEFAULT_ROUTE_BAND_FRACTION,
    )


def actor_example(schema, context, actor_id, start, kind):
    from .cosmos.output_examples import schema_example

    if "spatial_guidance" not in context:
        return schema_example(schema)
    cells = {c["id"]: c["target_world"] for c in context["grid"]["cells"]}
    choices = [cid for cid in context["valid_cells"] if cid != start]
    robot = context["spatial_guidance"].get("robot")
    total = context.get("placement_population_count", actor_id + 1)
    if robot and placement_from_context(context).group(actor_id, total) == "route_band":
        near = [cid for cid in choices if cid in robot["priority_cells"]]
        if near:
            choices = near
    rng = random.Random(context.get("episode", {}).get("seed", 0) + actor_id * 7919)
    rng.shuffle(choices)
    far = [cid for cid in choices if math.dist(cells[cid][:2], cells[start][:2]) >= 4]
    target = (far or choices or [start])[0]
    example = {"actor_id": actor_id, "subgoals": [], "goal_cell": target}
    if kind == "all_participants_arrived":
        example["subgoals"] = [{"cell_id": target, "action": "Idle", "duration_s": 3}]
    if "start_cell" in schema["properties"]:
        example["start_cell"] = start
    return example
