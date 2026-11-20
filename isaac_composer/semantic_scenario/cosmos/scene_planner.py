"""Plan scene coordination first, then one short route request per participant."""

import json
from ..spatial import start_candidates, actor_example
from ..schema import PlanError, LocalRepairFailed
from .client import CosmosError
from .start_selector import StartSelector
from .actor_repair import ActorPlanRepair
from .output_examples import schema_example, with_examples
from .contracts import obj, array, TEXT, scenes_contract, queues_contract, actor_route_contract

OUTLINE = """Plan the scene timeline for the supplied story and participants.
Do not generate individual routes yet. Use separate simultaneous scenes for independent people
or small groups. Only genuinely shared activities need all_participants_arrived.
Use each assigned actor meaningfully. All props stay static; walking, Idle, LookAround and native Queue are available.
Most people should walk through the environment, including across or along the robot route band.
Keep pauses brief and purposeful. Queue is optional, for a credible visible service need;
do not turn routine walking, inspections or an entire cast into a queue.
Return JSON: {"scenes":[{"scene_id":"scene_01","description":"activity purpose",
"start_condition":{"type":"episode_started"},"actor_ids":SELECTED_ACTOR_IDS}]}.
Replace actor_ids with the actual assigned IDs. For a shared conversation use
start_condition {"type":"all_participants_arrived"}; everyone first walks to their
own first activity cell and waits until all arrive. For a later scene use
{"type":"after_scene","scene_id":"the preceding actual scene ID"}.
Scenes sharing an actor must depend on that actor's preceding scene.
For a queue, add top-level queues:[{"queue_id":"check_queue","spots":["front cell","next cell","tail cell"]}].
Choose actual distinct valid cells in line near a plausible service location, front first.
Provide at least as many spots as queue participants. For non-queue stories use queues:[] .
Native People Queue handles filling spots and advancing. Queue scenes use episode_started or after_scene;
never all_participants_arrived, because only one person can be at the front at a time.
Return only the scene coordination JSON, with no actor_plans or routes."""


ACTOR = """Plan ONLY the requested actor's route for the specified scene.
Return a JSON object with EXACTLY actor_id, start_cell, subgoals, goal_cell.
Choose every destination from valid_cells; NEVER invent a cell ID.
start_cell MUST belong to allowed_start_cells.
Walk meaningful distances through or across the robot travel band. Do not stay at the start.
For uninterrupted walking use subgoals:[] and a different goal_cell.
A subgoal string is a pass-through waypoint. An object has cell_id, action and duration_s:
GoTo uses duration_s:0; Idle and LookAround are brief stops. Keep ordinary stops within
spatial_guidance's pause budget. Do not stop at every neighboring grid cell.
Queue is available ONLY if queues defines it: use its queue_id and FRONT cell_id,
service_action Idle or LookAround, positive duration_s, then a goal outside its spots.
For all_participants_arrived, first walk to your own nearby activity cell and use Idle
or LookAround there, so the small group can meet. Other actors' destinations are supplied.
Use a different initial start cell from previous actor plans.
Only people move; props stay static. Make this actor's purpose visible in its movement.
If repair_feedback exists correct its errors, preserving valid choices.
Return only the actor object, no scenes wrapper, markdown or explanations."""


class ScenePlanner:
    def __init__(self, client):
        self.client = client
        self.start_selector = StartSelector(client)
        self.actor_repair = ActorPlanRepair(client)

    def generate(self, context, story, participants, feedback=None):
        ids = [p["actor_id"] for p in participants["participants"]]
        population = {
            "global_scenario": story,
            "participant_plan": participants,
            "repair_feedback": feedback,
            "valid_cells": context["valid_cells"],
            "cell_size_m": context["grid"]["cell_size_m"],
        }
        if "spatial_guidance" in context:
            population["spatial_guidance"] = context["spatial_guidance"]
        scene_schema = scenes_contract(context["valid_cells"], ids)["properties"]["scenes"]["items"]
        outline_schema = obj(
            {
                "scenes": array(
                    obj(
                        {
                            "scene_id": TEXT,
                            "description": TEXT,
                            "start_condition": scene_schema["properties"]["start_condition"],
                            "actor_ids": array({"type": "integer", "enum": ids}, 1, len(ids)),
                        }
                    ),
                    1,
                    6,
                ),
                "queues": queues_contract(context["valid_cells"]),
            }
        )
        instruction = OUTLINE.replace("SELECTED_ACTOR_IDS", json.dumps(ids))
        outline = self.client.complete(
            instruction,
            with_examples(
                population,
                {
                    "scenes": [
                        {
                            "scene_id": "scene_01",
                            "description": "Carry out the planned activities.",
                            "start_condition": {"type": "episode_started"},
                            "actor_ids": ids,
                        }
                    ],
                    "queues": [],
                },
            ),
            [context["scene_image"]],
            schema=outline_schema,
        )
        scenes = outline.get("scenes")
        if not isinstance(scenes, list) or not 1 <= len(scenes) <= 6:
            raise PlanError("Scene outline needs one to six scenes")
        mentioned = set()
        for scene in scenes:
            actor_ids = scene.get("actor_ids", [])
            if (
                not actor_ids
                or any(type(a) is not int or a not in ids for a in actor_ids)
                or len(set(actor_ids)) != len(actor_ids)
            ):
                raise PlanError("Scene outline actor_ids must be unique assigned IDs")
            mentioned.update(actor_ids)
        if mentioned != set(ids):
            raise PlanError("Scene outline must include every Cosmos actor")
        result = {"scenes": []}
        valid = {c.casefold(): c for c in context["valid_cells"]}
        definitions = outline.get("queues", [])
        for queue in definitions:
            queue["spots"] = [valid.get(cid.strip().casefold(), cid) for cid in queue["spots"]]
        if definitions:
            result["queues"] = definitions
        from ..queues import queue_definitions

        queues = queue_definitions({"queues": definitions}, set(context["valid_cells"]))
        initial_starts = {}
        roles = {p["actor_id"]: p for p in participants["participants"]}
        for scene in scenes:
            item = {k: scene[k] for k in ("scene_id", "description", "start_condition")}
            item["actor_plans"] = []
            for aid in scene["actor_ids"]:
                print(f"SEMANTIC_ACTOR scene={scene['scene_id']} actor={aid}", flush=True)
                # These exclude occupied INITIAL spawns, not later visits. A
                # person's later scenes may legitimately revisit another start.
                allowed_starts = [
                    cid
                    for cid in context["valid_cells"]
                    if aid in initial_starts or cid not in initial_starts.values()
                ]
                if not allowed_starts:
                    raise PlanError("Not enough distinct valid cells for the initial spawns")
                initial = aid not in initial_starts
                selected = None
                if initial:
                    allowed_starts = start_candidates(
                        context,
                        allowed_starts,
                        initial_starts,
                        aid,
                        participants.get("total_count", max(ids) + 1),
                    )
                    guidance = None
                    if "spatial_guidance" in context:
                        from ..spatial import placement_from_context

                        placement_group = placement_from_context(context).group(
                            aid, participants.get("total_count", max(ids) + 1)
                        )
                        allowed = set(allowed_starts)
                        guidance = {
                            "role": roles[aid],
                            "placement_group": placement_group,
                            "scene_purpose": scene["description"],
                            "regions": [
                                {
                                    "region_id": r["region_id"],
                                    "cells": [cid for cid in r["cells"] if cid in allowed],
                                }
                                for r in context["spatial_guidance"]["regions"]
                                if any(cid in allowed for cid in r["cells"])
                            ],
                            "goal": (
                                "Choose a purpose-consistent start near the robot travel band."
                                if placement_group == "route_band"
                                else "Choose a purpose-consistent start anywhere in the allowed walkable cells. Do not prefer the robot route for this actor."
                            ),
                        }
                    selected = self.start_selector.choose(aid, allowed_starts, guidance)
                    initial_starts[aid] = selected
                    allowed_starts = [selected]
                payload = {
                    **{
                        key: value for key, value in population.items() if key != "participant_plan"
                    },
                    "scene_timeline": outline,
                    "current_scene": scene,
                    "requested_actor": roles[aid],
                    "valid_cells": context["valid_cells"],
                    "allowed_start_cells": allowed_starts,
                    "assigned_initial_starts": [
                        {"actor_id": owner, "cell_id": cid} for owner, cid in initial_starts.items()
                    ],
                    "cell_size_m": context["grid"]["cell_size_m"],
                    "other_actor_destinations": [
                        {
                            "actor_id": a["actor_id"],
                            "scene_id": s["scene_id"],
                            "stops": [step["cell_id"] for step in a["subgoals"]],
                            "goal_cell": a["goal_cell"],
                        }
                        for s in result["scenes"] + [item]
                        for a in s["actor_plans"]
                    ],
                    "queues": definitions,
                }
                schema = actor_route_contract(
                    context["valid_cells"],
                    aid,
                    allowed_starts,
                    queues,
                    scene["start_condition"]["type"],
                )
                instruction = ACTOR
                if initial:
                    payload["selected_start_cell"] = selected
                    instruction = (
                        ACTOR.replace(
                            "EXACTLY actor_id, start_cell, subgoals, goal_cell.",
                            "EXACTLY actor_id, subgoals, goal_cell.\n"
                            "Cosmos already selected the initial start in a separate request. "
                            "Use selected_start_cell as the fixed origin; do not choose or output start_cell.",
                        )
                        .replace(
                            "start_cell MUST belong to allowed_start_cells.",
                            "The initial origin is fixed to selected_start_cell.",
                        )
                        .replace(
                            "Use a different initial start cell from previous actor plans.", ""
                        )
                    )
                    schema["properties"].pop("start_cell")
                    schema["required"].remove("start_cell")
                constraints = f"\nactor_id MUST equal {aid}.\n" + (
                    "The origin is fixed to selected_start_cell; do not output start_cell.\n"
                    if initial
                    else "Choose start_cell ONLY from allowed_start_cells in INPUT.\n"
                ) + "Choose goal_cell and ALL subgoal cell_id values ONLY from valid_cells in INPUT.\n" "Do not output IDs outside these lists.\n" + (
                    "Queue is forbidden: no queue is defined.\n" if not queues else ""
                )
                errors = []
                for attempt in range(3):
                    actor = None
                    try:
                        payload["output_task"] = (
                            f"Plan ONLY actor {aid}. Return a complete JSON object with "
                            f"these required keys: {json.dumps(schema['required'])}. "
                            "goal_cell is REQUIRED: choose its ID from valid_cells. "
                            "Use at most FOUR subgoals, preferably one or two. "
                            "Each subgoal needs cell_id, action and numeric duration_s. "
                            + (
                                f"The initial origin is already fixed to {selected}. "
                                if initial
                                else "Choose start_cell from allowed_start_cells. "
                            )
                            + "Do not omit the final goal_cell or return only subgoals."
                        )
                        actor = self.client.complete(
                            instruction + constraints,
                            with_examples(
                                payload,
                                actor_example(
                                    schema,
                                    context,
                                    aid,
                                    selected or allowed_starts[0],
                                    scene["start_condition"]["type"],
                                ),
                            ),
                            [context["scene_image"]],
                            schema=schema,
                        )
                        if initial:
                            # This is the earlier Cosmos decision, not a code
                            # fallback or a new selection by the route request.
                            actor["start_cell"] = selected
                        for key in ("start_cell", "goal_cell"):
                            value = actor.get(key)
                            if isinstance(value, str):
                                actor[key] = valid.get(value.strip().casefold(), value)
                        if isinstance(actor.get("subgoals"), list):
                            actor["subgoals"] = [
                                (
                                    {"cell_id": step, "action": "GoTo", "duration_s": 0}
                                    if isinstance(step, str)
                                    else step
                                )
                                for step in actor["subgoals"]
                            ]
                            for step in actor["subgoals"]:
                                if isinstance(step, dict):
                                    value = step.get("cell_id")
                                    if isinstance(value, str):
                                        step["cell_id"] = valid.get(value.strip().casefold(), value)
                        self.actor_repair.repair(
                            actor,
                            aid,
                            context["valid_cells"],
                            queues,
                            scene["start_condition"]["type"],
                            allowed_starts,
                            {
                                "global_scenario": story,
                                "role": roles[aid],
                                "scene": scene,
                                "queues": definitions,
                                "valid_cells": context["valid_cells"],
                                "spatial_guidance": context.get("spatial_guidance"),
                            },
                        )
                        break
                    except LocalRepairFailed:
                        raise
                    except (PlanError, CosmosError, KeyError, TypeError, AttributeError) as error:
                        if attempt == 2:
                            raise LocalRepairFailed(f"Actor {aid} route failed: {error}") from error
                        errors.append(str(error))
                        payload["repair_feedback"] = {
                            "errors": list(errors),
                            "rejected_response": actor,
                            "prior_feedback": feedback,
                        }
                        print(f"SEMANTIC_ACTOR_REPAIR actor={aid} reason={error}", flush=True)
                item["actor_plans"].append(actor)
            result["scenes"].append(item)
        return result
