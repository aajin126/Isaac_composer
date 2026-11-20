"""Repair explicit validation paths without rewriting accepted actor fields."""

import copy

from ..schema import (
    ActorFieldError,
    LocalRepairFailed,
    PlanError,
    english_json,
    validate_actor_route,
)
from .client import CosmosError
from .contracts import actor_route_contract, obj
from .route_repair import RouteCellRepair
from .output_examples import schema_example, with_examples


def normalize_actions(actor):
    """Normalize spelling case only; never substitute a different behavior."""
    steps = actor.get("subgoals")
    if isinstance(steps, list):
        for step in steps:
            if not isinstance(step, dict):
                continue
            for field in ("action", "service_action"):
                value = step.get(field)
                if isinstance(value, str):
                    canonical = {
                        name.casefold(): name for name in ("Idle", "LookAround", "Queue", "GoTo")
                    }
                    step[field] = canonical.get(value.strip().casefold(), value)


def replace_field(document, path, value):
    """Set an existing container's field; never invent intermediate containers."""
    if not path:
        raise PlanError("A local correction needs a field path")
    parent = document
    for key in path[:-1]:
        parent = parent[key]
    parent[path[-1]] = copy.deepcopy(value)


def field_schema(schema, path):
    if not path:
        return schema
    if "anyOf" in schema:
        return {"anyOf": [field_schema(option, path) for option in schema["anyOf"]]}
    key, *remaining = path
    child = schema["items"] if type(key) is int else schema["properties"][key]
    return field_schema(child, remaining)


class ActorPlanRepair:
    def __init__(self, client):
        self.client = client
        self.cells = RouteCellRepair(client)

    def repair(self, actor, actor_id, cells, queues, kind, start_cells, context):
        if not isinstance(actor, dict):
            raise PlanError("An actor plan must be an object")
        normalize_actions(actor)
        schema = actor_route_contract(cells, actor_id, start_cells, queues, kind)

        def check(candidate):
            validate_actor_route(candidate, actor_id, set(cells), queues, kind)
            guidance = context.get("spatial_guidance")
            if guidance:
                stops = [
                    (i, s)
                    for i, s in enumerate(candidate["subgoals"])
                    if s["action"] in ("Idle", "LookAround")
                ]
                limit = min(
                    guidance.get("ordinary_pause_max_s", 4.0),
                    guidance.get("ordinary_pause_budget_s_per_actor_scene", 6.0)
                    / max(1, len(stops)),
                )
                for index, step in stops:
                    if step["duration_s"] > limit:
                        raise ActorFieldError(
                            f"An ordinary stop in this moving navigation episode must be at most {limit:g} seconds. Keep people walking between brief purposeful stops.",
                            ("subgoals", index, "duration_s"),
                            {"type": "number", "exclusiveMinimum": 0, "maximum": limit},
                        )
            if candidate["start_cell"] not in start_cells:
                raise ActorFieldError(
                    "start_cell must belong to allowed_start_cells",
                    ("start_cell",),
                    {"type": "string", "enum": list(start_cells)},
                )

        # Cell selection is a specialized form of the same local repair: the
        # model only receives valid IDs and repeated bad IDs keep one choice.
        if type(actor.get("actor_id")) is int and actor["actor_id"] == actor_id:
            if isinstance(actor.get("subgoals", []), list):
                self.cells.repair(actor, cells)
        for _ in range(24):
            try:
                check(actor)
                return actor
            except ActorFieldError as issue:
                if not issue.path:
                    raise LocalRepairFailed(
                        f"Actor {actor_id} has no editable field: {issue}"
                    ) from issue
                constraint = issue.constraint or field_schema(schema, issue.path)
                self._correct(actor, actor_id, issue, constraint, context, check)
        raise LocalRepairFailed(f"Actor {actor_id} exceeded the local correction limit")

    def _correct(self, actor, actor_id, issue, constraint, context, check):
        error = None
        path = list(issue.path)
        retain_steps = (
            path == ["subgoals"]
            and isinstance(actor.get("subgoals"), list)
            and len(actor["subgoals"]) > 4
        )
        if retain_steps:
            constraint = {
                "type": "array",
                "items": {"type": "integer", "enum": list(range(len(actor["subgoals"])))},
                "minItems": 4,
                "maxItems": 4,
                "uniqueItems": True,
            }
        if path[-1] == "duration_s" and constraint.get("type") == "number":
            limit = constraint.get("maximum", 120)
            values = [
                n for n in (1, 2, 3, 4, 5, 8, 10, 15, 20, 30, 45, 60, 90, 120) if 0 < n <= limit
            ]
            if 0 < limit <= 120 and limit not in values:
                values.append(limit)
            constraint = {**constraint, "enum": values}
        example = {"value": [0, 1, 2, 3] if retain_steps else schema_example(constraint)}
        for attempt in range(3):
            payload = {
                "actor_id": actor_id,
                "field_path": path,
                "validation_error": str(issue),
                "current_actor_plan": actor,
                "allowed_value": constraint,
                "activity_context": context,
            }
            if path[-1] == "duration_s":
                payload.pop("current_actor_plan")
                payload["activity_context"] = {
                    key: context[key] for key in ("role",) if key in context
                }
            options = constraint.get("enum")
            if "const" in constraint:
                options = [constraint["const"]]
            if options is not None:
                # Just like start selection, do not repeatedly expose the
                # rejected choice in an error message or a rejected plan.
                payload = {
                    "actor_id": actor_id,
                    "field_path": path,
                    "available_values": options,
                    "activity_context": context,
                    "output_task": 'Choose ONLY one available_values entry and return {"value": the chosen entry}.',
                }
            if error:
                payload["repair_feedback"] = (
                    "The last choice was invalid. Copy only one entry from available_values."
                    if options is not None
                    else str(error)
                )
            if retain_steps:
                payload["output_task"] = (
                    'The list has too many subgoals. Return {"value":[four distinct existing '
                    "subgoal indexes in ascending order]} selecting which four to RETAIN. "
                    "Do not rewrite the subgoal objects. Their original details will be preserved."
                )
            if isinstance(error, CosmosError) and error.response_text:
                # Repair the malformed answer itself instead of asking the
                # model to copy the full actor plan a second time.
                payload = {
                    "rejected_response": error.response_text[:8000],
                    "validation_error": str(error),
                    "allowed_value": constraint,
                    "output_task": "Fix only this answer's JSON syntax. Return one object with only value. Remove extra closing characters and text; keep the intended value.",
                }
            try:
                response = self.client.complete(
                    "Correct ONLY field_path in this pedestrian plan. Preserve all other fields. "
                    "Choose a value that fixes validation_error and fits the activity. "
                    'Return ONLY {"value": the corrected field value}. All text must be English ASCII.',
                    with_examples(payload, example),
                    schema=obj({"value": constraint}),
                )
                english_json(response)
                if set(response) != {"value"}:
                    raise PlanError("Return only the corrected value")
                value = response["value"]
                if retain_steps:
                    if (
                        not isinstance(value, list)
                        or len(value) != 4
                        or any(
                            type(i) is not int or not 0 <= i < len(actor["subgoals"]) for i in value
                        )
                        or value != sorted(set(value))
                    ):
                        raise PlanError("Select four distinct subgoal indexes in ascending order")
                    value = [actor["subgoals"][i] for i in value]
                candidate = copy.deepcopy(actor)
                replace_field(candidate, path, value)
                normalize_actions(candidate)
                try:
                    check(candidate)
                except ActorFieldError as remaining:
                    if remaining.path == issue.path:
                        raise
                actor.clear()
                actor.update(candidate)
                print(f"SEMANTIC_FIELD_REPLACED actor={actor_id} path={path}", flush=True)
                return
            except (PlanError, CosmosError, KeyError, TypeError, IndexError, AttributeError) as exc:
                error = exc
                print(
                    f"SEMANTIC_FIELD_REPAIR actor={actor_id} path={path} reason={error}", flush=True
                )
        raise LocalRepairFailed(
            f"Actor {actor_id} field {path} replacement failed: {error}"
        ) from error
