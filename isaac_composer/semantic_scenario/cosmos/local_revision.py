"""Apply Cosmos's semantic corrections only to the actors named by review."""

import copy

from ..schema import (
    LocalRepairFailed,
    PlanError,
    english_json,
    validate_participants,
    validate_scene_plan,
)
from .actor_repair import replace_field
from .client import CosmosError
from .contracts import array, obj
from .output_examples import with_examples


class LocalScenarioRevision:
    def __init__(self, client):
        self.client = client

    def revise(self, context, story, participants, plan, review):
        people, revised = copy.deepcopy(participants), copy.deepcopy(plan)
        grouped = {}
        for issue in review["issues"]:
            aid = issue.get("actor_id")
            if aid is None or issue.get("scope") == "global_scenario":
                raise PlanError("Global issues require a global scenario revision")
            grouped.setdefault(aid, []).append(issue)
        for aid, issues in grouped.items():
            people, revised = self._actor(context, story, people, revised, aid, issues)
        return {"participant_plan": people, "scene_plan": revised}

    def _actor(self, context, story, people, plan, aid, issues):
        role = next(p for p in people["participants"] if p["actor_id"] == aid)
        scenes = [
            {
                "scene_id": s["scene_id"],
                "description": s["description"],
                "start_condition": s["start_condition"],
                "actor_plan": actor,
            }
            for s in plan["scenes"]
            for actor in s["actor_plans"]
            if actor["actor_id"] == aid
        ]
        scene_ids = [s["scene_id"] for s in scenes]
        if any(issue.get("scope") == "roles" for issue in issues):
            example = {"scene_id": None, "path": ["role"], "value": "activity coordinator"}
        elif scenes[0]["actor_plan"]["subgoals"]:
            example = {"scene_id": scene_ids[0], "path": ["subgoals", 0, "duration_s"], "value": 8}
        else:
            example = {
                "scene_id": scene_ids[0],
                "path": ["goal_cell"],
                "value": context["valid_cells"][0],
            }
        schema = obj(
            {
                "changes": array(
                    obj(
                        {
                            "scene_id": {"enum": [None, *scene_ids]},
                            "path": array(
                                {"anyOf": [{"type": "string"}, {"type": "integer", "minimum": 0}]},
                                1,
                                3,
                            ),
                            "value": {},
                        }
                    ),
                    1,
                    12,
                )
            }
        )
        error = None
        for attempt in range(3):
            payload = {
                "actor_id": aid,
                "global_scenario": story,
                "current_role": role,
                "actor_scene_plans": scenes,
                "review_issues": issues,
                "valid_cells": context["valid_cells"],
                "queues": plan.get("queues", []),
                "spatial_guidance": context.get("spatial_guidance"),
            }
            if error:
                payload["repair_feedback"] = str(error)
            try:
                response = self.client.complete(
                    "Correct ONLY the named actor and the problems in review_issues. "
                    'Return ONLY {"changes":[{"scene_id":"existing scene ID",'
                    '"path":["subgoals",0,"duration_s"],"value":8}]}. '
                    "Use the smallest necessary changes. Preserve every other valid choice. "
                    'For role or purpose corrections use scene_id:null and path:["role"] '
                    'or ["purpose"]. Scene paths may address start_cell, goal_cell, subgoals, '
                    "one subgoal, or a field inside one subgoal. Do not change actor_id, "
                    "other people, scene coordination, queues, or the global story. "
                    "Use only valid_cells and English ASCII text.",
                    with_examples(payload, {"changes": [example]}),
                    [context["scene_image"]],
                    schema=schema,
                )
                english_json(response)
                changes = response.get("changes")
                if (
                    set(response) != {"changes"}
                    or not isinstance(changes, list)
                    or not 1 <= len(changes) <= 12
                ):
                    raise PlanError(
                        "Return a nonempty changes list with at most twelve corrections"
                    )
                candidate_people, candidate = copy.deepcopy(people), copy.deepcopy(plan)
                candidate_role = next(
                    p for p in candidate_people["participants"] if p["actor_id"] == aid
                )
                for change in changes:
                    if not isinstance(change, dict) or set(change) != {"scene_id", "path", "value"}:
                        raise PlanError("Each correction needs scene_id, path and value")
                    path = change["path"]
                    if not isinstance(path, list) or not path:
                        raise PlanError("Correction path must be a nonempty list")
                    if path in (["role"], ["purpose"]) and change["scene_id"] is None:
                        replace_field(candidate_role, path, change["value"])
                        continue
                    self._check_scene_path(path)
                    scene = next(
                        (s for s in candidate["scenes"] if s["scene_id"] == change["scene_id"]),
                        None,
                    )
                    if scene is None:
                        raise PlanError("Correction must name one of the actor's scenes")
                    actor = next((p for p in scene["actor_plans"] if p["actor_id"] == aid), None)
                    if actor is None:
                        raise PlanError("The named actor does not participate in this scene")
                    replace_field(actor, path, change["value"])
                validate_participants(
                    candidate_people,
                    people["total_count"],
                    [p["actor_id"] for p in people["participants"]],
                )
                validate_scene_plan(candidate, candidate_people, context["grid"])
                if candidate_people == people and candidate == plan:
                    raise PlanError("The requested correction made no change")
                print(f"SEMANTIC_LOCAL_REVISION actor={aid} changes={len(changes)}", flush=True)
                return candidate_people, candidate
            except (PlanError, CosmosError, KeyError, TypeError, IndexError, AttributeError) as exc:
                error = exc
                print(f"SEMANTIC_LOCAL_REVISION_REPAIR actor={aid} reason={error}", flush=True)
        raise LocalRepairFailed(f"Actor {aid} semantic correction failed: {error}") from error

    @staticmethod
    def _check_scene_path(path):
        if path in (["start_cell"], ["goal_cell"], ["subgoals"]):
            return
        if (
            len(path) in (2, 3)
            and path[0] == "subgoals"
            and type(path[1]) is int
            and path[1] >= 0
            and (
                len(path) == 2
                or path[2] in ("cell_id", "action", "duration_s", "queue_id", "service_action")
            )
        ):
            return
        raise PlanError("Correction path must address an editable actor activity field")
