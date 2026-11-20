"""Validate exact grid centers and compile legs through official NavMesh queries."""

from __future__ import annotations

import copy
import hashlib
import json
import math
import shutil
from pathlib import Path

from ..schema import (
    PlanError,
    english_json,
    validate_global,
    validate_participants,
    validate_scene_plan,
    validate_review,
    save_json,
    NativeRouteError,
)


def plan_digest(story, participants, plan, grid, planning_inputs=None):
    values = [story, participants, plan, grid]
    if planning_inputs is not None:
        values.append(planning_inputs)
    data = json.dumps(values, sort_keys=True, allow_nan=False)
    return hashlib.sha256(data.encode("utf-8")).hexdigest()


def animation_query_scene(scene, tracking_margin_m):
    """Allow tiny root-motion drift across the artificial collection rectangle.

    Planning targets stay exact, strictly validated centers. Only runtime query
    bounds receive the existing tracking allowance; baked NavMesh, occupancy
    clearance, physical radii and ORCA obstacles are unchanged.
    """
    query_scene = copy.copy(scene)
    region = getattr(scene, "allowed_region_xy", None)
    if region is not None:
        padding = min(0.05, max(0.0, float(tracking_margin_m)))
        a, b, c, d = region
        query_scene.allowed_region_xy = (a - padding, b - padding, c + padding, d + padding)
    return query_scene


def save_execution_inputs(output, source_dir, context, compiled):
    """Keep each episode independent of the original planning directory."""
    output, source_dir = Path(output).resolve(), Path(source_dir).resolve()
    copied = output / "semantic_plan"
    if source_dir in copied.parents:
        raise PlanError("Episode output must be outside the source plan directory")
    output.mkdir(parents=True, exist_ok=True)
    save_json(output / "compiled_scene_plan.json", compiled)
    if source_dir != copied:
        shutil.copytree(source_dir, copied, dirs_exist_ok=True)
    context = copy.deepcopy(context)
    for key, name in (("scene_image", "grid_scene.png"), ("world_image", "world_scene.png")):
        if key in context:
            source = Path(context[key])
            target = copied / name
            if source.resolve() != target:
                shutil.copy2(source, target)
            context[key] = str(target)
    save_json(copied / "scene_context.json", context)


def load_reviewed_plan(directory):
    directory = Path(directory).resolve()
    read = lambda name: json.loads((directory / f"{name}.json").read_text())
    approval, review = read("approval"), read("review")
    story, participants, plan, context = (
        read(n) for n in ("global_scenario", "participant_plan", "scene_plan", "scene_context")
    )
    for value in (approval, review, story, participants, plan, context):
        english_json(value)
    if approval.get("approved") is not True or review.get("passed") is not True:
        raise PlanError("A passed Cosmos review is required before loading a semantic plan")
    inputs = (
        {key: context.get(key) for key in ("robot_task", "spatial_guidance")}
        if approval.get("digest_version") == 2
        else None
    )
    if approval.get("content_sha256") != plan_digest(
        story, participants, plan, context["grid"], inputs
    ):
        raise PlanError("Plan artifacts changed after Cosmos review; generate a new review")
    ids = [p["actor_id"] for p in participants["participants"]]
    validate_global(story)
    validate_participants(participants, participants["total_count"], ids)
    validate_review(review, participants)
    validate_scene_plan(plan, participants, context["grid"])
    return context, participants, plan


def native_path(scene, start, end):
    start, end = list(map(float, start)), list(map(float, end))
    if math.dist(start[:2], end[:2]) < 1e-5:
        return [start, end]
    path = [list(map(float, p)) for p in scene.shortest_path(start, end)]
    if (
        len(path) < 2
        or math.dist(path[0][:2], start[:2]) > 0.05
        or math.dist(path[-1][:2], end[:2]) > 0.05
    ):
        raise PlanError("Native path does not reach both requested endpoints")
    return path


def compile_scene_plan(scene, context, participants, plan):
    validate_scene_plan(plan, participants, context["grid"])
    cells = {c["id"]: c for c in context["grid"]["cells"]}
    used = set()
    for item in plan["scenes"]:
        for actor in item["actor_plans"]:
            used.update([actor["start_cell"], actor["goal_cell"]])
            used.update(s["cell_id"] for s in actor["subgoals"])
    for queue in plan.get("queues", []):
        used.update(queue["spots"])
    targets = {}
    for cid in used:
        cell = cells[cid]
        p = list(map(float, cell["target_world"]))
        a, b, c, d = cell["bounds_xy"]
        if abs(p[0] - (a + c) / 2) > 1e-6 or abs(p[1] - (b + d) / 2) > 1e-6:
            raise PlanError("Semantic targets must be exact cell centers")
        projected = scene.project_world(p)
        if (
            math.dist(p[:2], projected[:2]) > 0.001
            or abs(p[2] - projected[2]) > 0.05
            or not scene.is_allowed_world(p)
        ):
            raise PlanError(f"Cell {cid} is not a valid center in the current NavMesh")
        targets[cid] = p
    queues = []
    for queue in plan.get("queues", []):
        positions = [targets[cid] for cid in queue["spots"]]
        for a, b in zip(positions[:-1], positions[1:]):
            native_path(scene, a, b)
        queues.append({**copy.deepcopy(queue), "positions": positions})
    starts, previous, scenes = {}, {}, []

    def leg_path(start, end, aid, sid, path):
        try:
            return native_path(scene, start, end)
        except (RuntimeError, ValueError) as error:
            raise NativeRouteError(str(error), aid, sid, path, start) from error

    for item in plan["scenes"]:
        compiled = {
            k: copy.deepcopy(item[k]) for k in ("scene_id", "description", "start_condition")
        }
        compiled["actors"] = []
        for actor in item["actor_plans"]:
            aid = actor["actor_id"]
            start = targets[actor["start_cell"]]
            position = previous.get(aid, start)
            legs = []
            if aid not in starts:
                starts[aid] = start
            else:
                legs.append(
                    {
                        "cell_id": actor["start_cell"],
                        "target": start,
                        "path": leg_path(position, start, aid, item["scene_id"], ["start_cell"]),
                        "action": "GoTo",
                        "duration_s": 0,
                        "activity_index": -1,
                    }
                )
                position = start
            for index, step in enumerate(actor["subgoals"]):
                target = targets[step["cell_id"]]
                legs.append(
                    {
                        **copy.deepcopy(step),
                        "target": target,
                        "path": leg_path(
                            position, target, aid, item["scene_id"], ["subgoals", index, "cell_id"]
                        ),
                        "activity_index": index,
                    }
                )
                position = target
            goal = targets[actor["goal_cell"]]
            legs.append(
                {
                    "cell_id": actor["goal_cell"],
                    "target": goal,
                    "path": leg_path(position, goal, aid, item["scene_id"], ["goal_cell"]),
                    "action": "GoTo",
                    "duration_s": 0,
                    "activity_index": -1,
                }
            )
            compiled["actors"].append({"actor_id": aid, "legs": legs})
            previous[aid] = goal
        scenes.append(compiled)
    return {
        "schema_version": 1,
        "scenes": scenes,
        "starts": {str(k): v for k, v in starts.items()},
        "participant_plan": participants,
        "source": "cosmos",
        "queues": queues,
        "repeat_semantic_plan": True,
    }


def build_mixed_people(compiled, planner):
    """Keep one stable roster: semantic starts plus randomly generated remainder."""
    population = compiled["participant_plan"]
    assigned = {p["actor_id"]: p for p in population["participants"]}
    people = {}
    low, high = planner.config.get("speed_range_mps", [0.6, 1.4])
    for aid in sorted(assigned):
        start = compiled["starts"][str(aid)]
        if not planner._separated(start, list(people.values())):
            raise PlanError("Semantic initial positions violate actor/robot spawn separation")
        first = next(a for s in compiled["scenes"] for a in s["actors"] if a["actor_id"] == aid)
        route = first["legs"][0]["path"]
        # Orient an initially stationary actor toward its first later movement;
        # a wholly stationary actor keeps the route helper's default heading.
        if all(math.dist(start[:2], p[:2]) < 1e-5 for p in route):
            target = next(
                (
                    leg["target"]
                    for s in compiled["scenes"]
                    for a in s["actors"]
                    if a["actor_id"] == aid
                    for leg in a["legs"]
                    if math.dist(start[:2], leg["target"][:2]) > 1e-5
                ),
                None,
            )
            if target is not None:
                route = native_path(planner.scene, start, target)
        person = planner._person(aid, route, planner.random.uniform(low, high))
        person.update(
            repeat=False,
            planner_source="cosmos",
            role=assigned[aid]["role"],
            purpose=assigned[aid]["purpose"],
        )
        people[aid] = person
    for aid in population["random_actor_ids"]:
        person = planner.random_person(aid, list(people.values()), population["total_count"])
        person.update(
            planner_source="random",
            role="background_pedestrian",
            purpose="Random background walking",
        )
        people[aid] = person
    return [people[i] for i in range(population["total_count"])]
