"""Structural constraints only; semantic judgments belong to Cosmos review."""

from __future__ import annotations

import json
import math
from pathlib import Path


class PlanError(ValueError):
    pass


class PlanningRejected(PlanError):
    """A generated candidate failed bounded validation or semantic review."""


class LocalRepairFailed(PlanningRejected):
    """A local correction failed; do not regenerate valid peers."""


class ActorFieldError(PlanError):
    """A validation failure with an explicit editable path inside one actor."""

    def __init__(self, message, path, constraint=None):
        super().__init__(message)
        self.path = tuple(path)
        self.constraint = constraint


class NativeRouteError(PlanError):
    def __init__(self, message, actor_id, scene_id, path, start):
        super().__init__(message)
        self.actor_id, self.scene_id, self.path, self.start = (
            actor_id,
            scene_id,
            list(path),
            list(start),
        )


def english_json(value, path="$"):
    """Catch non-English scripts in artifacts; prompts require English prose too."""
    if isinstance(value, str) and not value.isascii():
        char = next(c for c in value if not c.isascii())
        raise PlanError(
            f"All JSON strings and keys must be English ASCII text; "
            f"found U+{ord(char):04X} at {path}. Rewrite this field entirely in English "
            "using plain ASCII punctuation; do not include words from other languages."
        )
    if isinstance(value, dict):
        for key, item in value.items():
            english_json(key, f"{path}[key]")
            english_json(item, f"{path}.{key}")
    elif isinstance(value, list):
        for index, item in enumerate(value):
            english_json(item, f"{path}[{index}]")


def save_json(path, value):
    english_json(value)
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def _text(value, name):
    if not isinstance(value, str) or not value.strip():
        raise PlanError(f"{name} must be nonempty English text")
    return value


def _integer(value, name, minimum=0):
    if type(value) is not int or value < minimum:
        raise PlanError(f"{name} must be an integer >= {minimum}")
    return value


def validate_global(document):
    english_json(document)
    if not isinstance(document, dict):
        raise PlanError("Global scenario must be an object")
    _text(document.get("scenario_id"), "scenario_id")
    _text(document.get("summary"), "summary")
    events = document.get("events")
    if not isinstance(events, list) or not events:
        raise PlanError("Global scenario needs an events list")
    for event in events:
        if not isinstance(event, dict):
            raise PlanError("Each event must be an object")
        _text(event.get("description"), "event.description")
    return document


def validate_staffing(document, total):
    english_json(document)
    if not isinstance(document, dict):
        raise PlanError("Staffing must be an object")
    _integer(total, "total_count", 1)
    for key in ("total_count", "minimum_cosmos_count", "random_count"):
        _integer(document.get(key), key)
    count = _integer(document.get("cosmos_count"), "cosmos_count", 1)
    if not math.ceil(total / 2) <= count <= total:
        raise PlanError(f"cosmos_count must be between {math.ceil(total / 2)} and {total}")
    if document.get("total_count") != total or document.get("random_count") != total - count:
        raise PlanError("Population counts must match the provided total_count")
    if document.get("minimum_cosmos_count") != math.ceil(total / 2):
        raise PlanError("minimum_cosmos_count must be ceil(total_count / 2)")
    _text(document.get("justification"), "justification")
    return document


def validate_participants(document, total, selected):
    english_json(document)
    validate_staffing(document, total)
    if document["cosmos_count"] != len(selected):
        raise PlanError("cosmos_count must equal the number of selected actors")
    participants = document.get("participants")
    if not isinstance(participants, list) or len(participants) != len(selected):
        raise PlanError("Exactly one role is required for each selected actor")
    ids = []
    for actor in participants:
        if not isinstance(actor, dict):
            raise PlanError("Participants must contain objects")
        ids.append(_integer(actor.get("actor_id"), "actor_id"))
        _text(actor.get("role"), "role")
        _text(actor.get("purpose"), "purpose")
    if len(set(ids)) != len(ids) or set(ids) != set(selected):
        raise PlanError("Role actor IDs must equal the selected actor IDs without duplicates")
    remaining = document.get("random_actor_ids")
    if isinstance(remaining, list):
        for actor_id in remaining:
            _integer(actor_id, "random_actor_id")
    expected = set(range(total)) - set(selected)
    if (
        not isinstance(remaining, list)
        or len(remaining) != len(expected)
        or set(remaining) != expected
    ):
        raise PlanError("random_actor_ids must contain exactly the remaining actors")
    return document


def validate_actor_route(plan, expected_id, cells, queues, kind):
    from .queues import validate_queue_step

    if not isinstance(plan, dict):
        raise PlanError(f"Expected route for actor {expected_id}")
    if type(plan.get("actor_id")) is not int or plan["actor_id"] != expected_id:
        raise ActorFieldError(
            f"Expected route for actor {expected_id}", ("actor_id",), {"const": expected_id}
        )
    cell_constraint = {"type": "string", "enum": sorted(cells)}
    for key in ("start_cell", "goal_cell"):
        if not isinstance(plan.get(key), str) or plan[key] not in cells:
            raise ActorFieldError(f"Unknown {key}: {plan.get(key)}", (key,), cell_constraint)
    steps = plan.get("subgoals")
    if not isinstance(steps, list):
        raise ActorFieldError("subgoals must be a list", ("subgoals",))
    if len(steps) > 4:
        raise ActorFieldError("Each actor supports at most four subgoals per scene", ("subgoals",))
    if kind == "all_participants_arrived" and not steps:
        raise ActorFieldError(
            "Synchronized scenes need a first activity subgoal for every actor", ("subgoals",)
        )
    for index, step in enumerate(steps):
        path = ("subgoals", index)
        if not isinstance(step, dict):
            raise ActorFieldError("subgoals must contain objects", path)
        if not isinstance(step.get("cell_id"), str) or step["cell_id"] not in cells:
            raise ActorFieldError(
                f"Unknown subgoal cell: {step.get('cell_id')}", path + ("cell_id",), cell_constraint
            )
        actions = ["Idle", "LookAround", "Queue", "GoTo"]
        if kind == "all_participants_arrived" and index == 0:
            actions.remove("GoTo")
        if not queues or kind == "all_participants_arrived":
            actions.remove("Queue")
        if step.get("action") not in actions:
            message = (
                "Subgoal action must be Idle, LookAround or Queue; Queue needs a defined queue_id"
            )
            if step.get("action") == "Queue" and kind == "all_participants_arrived":
                message = "Queue scenes must not wait for every actor to reach the front together"
            raise ActorFieldError(
                message,
                path + ("action",),
                {"type": "string", "enum": actions},
            )
        if step["action"] == "Queue":
            try:
                validate_queue_step(step, queues, kind)
            except ActorFieldError as error:
                raise ActorFieldError(str(error), path + error.path, error.constraint) from error
            following = (
                steps[index + 1].get("cell_id")
                if index + 1 < len(steps) and isinstance(steps[index + 1], dict)
                else plan["goal_cell"] if index + 1 == len(steps) else None
            )
            if following in queues[step["queue_id"]]["spots"]:
                following_path = (
                    ("subgoals", index + 1, "cell_id") if index + 1 < len(steps) else ("goal_cell",)
                )
                raise ActorFieldError(
                    "Queue service needs a following destination outside the queue spots for Dequeue",
                    following_path,
                    {
                        "type": "string",
                        "enum": sorted(cells - set(queues[step["queue_id"]]["spots"])),
                    },
                )
        duration = step.get("duration_s")
        if step["action"] == "GoTo":
            if type(duration) not in (int, float) or duration != 0:
                raise ActorFieldError(
                    "GoTo is a pass-through waypoint and needs duration_s:0",
                    path + ("duration_s",),
                    {"const": 0},
                )
        elif (
            isinstance(duration, bool)
            or not isinstance(duration, (int, float))
            or not math.isfinite(duration)
            or not 0 < duration <= 120
        ):
            raise ActorFieldError(
                f"Actor {expected_id}: EVERY subgoal needs numeric duration_s in (0,120]; missing or invalid value: {duration}",
                path + ("duration_s",),
                {"type": "number", "exclusiveMinimum": 0, "maximum": 120},
            )

    def check_text(value, path=()):
        if isinstance(value, str):
            try:
                english_json(value)
            except PlanError as error:
                raise ActorFieldError(str(error), path, {"type": "string"}) from error
        elif isinstance(value, dict):
            for key, item in value.items():
                if not key.isascii():
                    raise ActorFieldError("All JSON keys must be English ASCII text", path)
                check_text(item, path + (key,))
        elif isinstance(value, list):
            for index, item in enumerate(value):
                check_text(item, path + (index,))

    check_text(plan)
    return plan


def validate_scene_plan(document, participants, grid):
    english_json(document)
    allowed = {p["actor_id"] for p in participants["participants"]}
    cells = {c["id"] for c in grid["cells"]}
    if (
        not isinstance(document, dict)
        or not isinstance(document.get("scenes"), list)
        or not document["scenes"]
    ):
        raise PlanError("scene_plan must contain a nonempty scenes list")
    if len(document["scenes"]) > 6:
        raise PlanError("scene_plan supports at most six scenes")
    from .queues import queue_definitions

    queues = queue_definitions(document, cells)
    queue_users = {qid: set() for qid in queues}
    scene_ids, ancestors, last_scene, seen_actors = set(), {}, {}, set()
    first_starts = {}
    for scene in document["scenes"]:
        if not isinstance(scene, dict):
            raise PlanError("Scenes must contain objects")
        sid = _text(scene.get("scene_id"), "scene_id")
        if sid in scene_ids:
            raise PlanError("scene_id must be unique")
        _text(scene.get("description"), "scene.description")
        condition = scene.get("start_condition", {})
        if not isinstance(condition, dict):
            raise PlanError("start_condition must be an object")
        kind = condition.get("type")
        if kind not in {"episode_started", "after_scene", "all_participants_arrived"}:
            raise PlanError("Unknown scene start_condition type")
        dependency = (
            condition.get("scene_id") if kind == "after_scene" else condition.get("after_scene_id")
        )
        if dependency is not None and dependency not in scene_ids:
            raise PlanError("Scene dependencies must name an earlier scene")
        if kind == "after_scene" and dependency is None:
            raise PlanError("after_scene requires scene_id")
        ancestors[sid] = ({dependency} | ancestors[dependency]) if dependency else set()
        plans = scene.get("actor_plans")
        if not isinstance(plans, list) or not plans:
            raise PlanError("Each scene needs actor_plans")
        local_ids = set()
        for plan in plans:
            if not isinstance(plan, dict):
                raise PlanError("actor_plans must contain objects")
            actor = _integer(plan.get("actor_id"), "actor_id")
            if actor not in allowed or actor in local_ids:
                raise PlanError("Scene actor IDs must be unique and assigned to Cosmos")
            local_ids.add(actor)
            if actor in last_scene and last_scene[actor] not in ancestors[sid]:
                raise PlanError("Scenes sharing an actor must have an explicit dependency")
            validate_actor_route(plan, actor, cells, queues, kind)
            if actor not in seen_actors:
                if plan["start_cell"] in first_starts.values():
                    raise PlanError("Actors must have different initial start cells")
                first_starts[actor] = plan["start_cell"]
            for step in plan["subgoals"]:
                if step["action"] == "Queue":
                    queue_users[step["queue_id"]].add(actor)
            last_scene[actor] = sid
            seen_actors.add(actor)
        scene_ids.add(sid)
    if seen_actors != allowed:
        raise PlanError("Every Cosmos actor must have at least one scene plan")
    for qid, users in queue_users.items():
        if len(users) > len(queues[qid]["spots"]):
            raise PlanError("Queue needs enough spots for all assigned participants")
    return document


def validate_review(document, participants):
    english_json(document)
    if not isinstance(document, dict):
        raise PlanError("Review must be an object")
    if type(document.get("passed")) is not bool or not isinstance(document.get("issues"), list):
        raise PlanError("Review needs passed:boolean and issues:list")
    if document["passed"] and document["issues"]:
        raise PlanError("A passed review must have no issues")
    if not document["passed"] and not document["issues"]:
        raise PlanError("A failed review must explain at least one issue")
    allowed = {p["actor_id"] for p in participants["participants"]}
    for index, issue in enumerate(document["issues"]):
        if not isinstance(issue, dict):
            raise PlanError("Review issues must contain objects")
        if issue.get("actor_id") is not None and issue["actor_id"] not in allowed:
            raise PlanError("Review actor_id is not a Cosmos participant")
        _text(issue.get("problem"), "problem")
        correction = issue.get("requested_change")
        if not isinstance(correction, str) or not correction.strip():
            raise PlanError(
                f"issues[{index}].requested_change must be a nonempty English STRING, "
                "not an object or list; write the correction sentence directly"
            )
    return document
