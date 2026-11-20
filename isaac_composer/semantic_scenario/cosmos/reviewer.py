"""Independent fresh-context Cosmos critique; no coded role-quality metrics."""

import json

from ..schema import PlanError, validate_scene_plan
from .client import CosmosError
from .contracts import review_contract
from .review_response import normalize_review_response
from .output_examples import with_examples

REVIEW = """Independently review the SEMANTIC meaning of this proposed pedestrian scenario.
The image shows the actual environment and cell labels. The plan has not been executed.
Code has already verified every actor ID, cell ID, initial spawn uniqueness and scene dependency.
Do NOT rejudge these verified checks or invent an out-of-bounds cell or insufficient access.
The cell routes are intentions; native NavMesh and ORCA handle navigation and collision avoidance.
Think WITHIN THIS SPECIFIC ENVIRONMENT. Verify the story against what is actually visible;
story text is not evidence that a facility exists. Reject reliance on invented facilities.
Only humans on foot move. Walking, Idle, LookAround and Queue/Dequeue are supported.
Inspection, supervision, conversations or service can use Idle/LookAround: do not demand
physical manipulation, tools, vehicle driving, new facilities or continuous movement.
Check meaningful roles, meeting locations, queue order, and consistency with the story.
Compare recent stories for meaningful novelty. Accept plausible plans; do not invent a
problem merely to provide criticism. Reject only a concrete semantic inconsistency.
Return ONLY English JSON with passed and issues. Acceptable: {"passed":true,"issues":[]}.
A rejection needs passed:false and nonempty issues. Each issue has actor_id (an assigned
integer or null), scope (global_scenario, roles or scenes), problem (a concrete sentence),
and requested_change (a nonempty STRING sentence, never an object or list).
Every correction must fit the same visible environment and supported pedestrian commands.
If format_repair_feedback exists, fix that output error while keeping this review contract."""


class ScenarioReviewer:
    def __init__(self, client):
        self.client = client

    def review(self, context, story, participants, plan, route_image, feedback=None):
        validate_scene_plan(plan, participants, context["grid"])
        roles = {p["actor_id"]: p for p in participants["participants"]}
        lines = ["Story: " + story["summary"]]
        lines.extend("Event: " + e["description"] for e in story["events"])
        for scene in plan["scenes"]:
            lines.append(
                f"Scene {scene['scene_id']}: {scene['description']}; start {json.dumps(scene['start_condition'])}"
            )
            for actor in scene["actor_plans"]:
                aid = actor["actor_id"]
                stops = (
                    ", ".join(
                        f"{s['cell_id']} {s['action']} {s['duration_s']}s"
                        + (
                            f" queue={s['queue_id']} service={s.get('service_action','Idle')}"
                            if s["action"] == "Queue"
                            else ""
                        )
                        for s in actor["subgoals"]
                    )
                    or "no stationary stops"
                )
                lines.append(
                    f"P{aid} ({roles[aid]['role']}; {roles[aid]['purpose']}): {actor['start_cell']} -> [{stops}] -> {actor['goal_cell']}"
                )
        if plan.get("queues"):
            lines.append("Queue definitions: " + json.dumps(plan["queues"]))
        response = self.client.complete(
            REVIEW,
            with_examples(
                {
                    "proposal_to_review": "\n".join(lines),
                    "verified_by_code": {
                        "all_actor_ids_valid": True,
                        "all_cell_ids_valid": True,
                        "initial_start_cells_unique": True,
                        "scene_dependencies_valid": True,
                        "native_paths_already_verified": context.get(
                            "native_paths_verified", False
                        ),
                    },
                    "navigation_validation_phase": (
                        "native_paths_verified"
                        if context.get("native_paths_verified")
                        else "simulator_load"
                    ),
                    # An echoed 10KB plan is a failed answer, not an output example.
                    # Keep raw diagnostics on disk; do not feed the echo back again.
                    "format_repair_feedback": (
                        {
                            "validation_error": feedback.get(
                                "validation_error", "Invalid review format"
                            )
                        }
                        if feedback
                        else None
                    ),
                    "recent_scenarios": context.get("recent_scenarios", []),
                    "robot_travel_band": context.get("spatial_guidance", {}).get("robot"),
                    "output_task": (
                        "Return ONLY passed and issues. Do not copy global_scenario, participant_plan "
                        "or scene_plan. Use at most THREE issues, each with one short problem sentence "
                        "and one short requested_change STRING. An acceptable plan has an empty issues list."
                        " Check that this episode differs from recent activities, uses varied destinations and movement directions, "
                        "and exposes the robot travel band to natural pedestrian traffic. Do not approve an entire cast standing along a boundary."
                    ),
                },
                {"passed": True, "issues": []},
                {
                    "passed": False,
                    "issues": [
                        {
                            "actor_id": participants["participants"][0]["actor_id"],
                            "scope": "scenes",
                            "problem": "The planned stop is too short for the stated waiting activity.",
                            "requested_change": "Extend that actor's waiting duration to match the activity.",
                        }
                    ],
                },
            ),
            [context.get("scene_image", route_image)],
            schema=review_contract([p["actor_id"] for p in participants["participants"]]),
        )
        try:
            return normalize_review_response(response)
        except PlanError as error:
            raise CosmosError(str(error), response_text=json.dumps(response)) from error
