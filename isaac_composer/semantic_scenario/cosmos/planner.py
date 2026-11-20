"""One explicit model request per stage; the story precedes cast sizing."""

import copy
import json
from ..schema import PlanError, validate_participants, validate_staffing
from .contracts import global_contract, roles_contract
from .staffing import StaffingPlanner
from .output_examples import with_examples

GLOBAL = """Invent a coherent human activity situation for the actual world shown in the image.
Think WITHIN THIS SPECIFIC ENVIRONMENT in the supplied image, not a generic place of this type.
The image is the source of truth for the existing layout and facilities. Do not add
equipment, workstations, rooms or service counters just because this type of world often has them.
When an object's identity is unclear, describe its visible location or shape without guessing its function.
Be imaginative about people's purposes and interactions WITHIN those limits.
This is a robot navigation scene: most people must visibly walk between distinct places,
not stand along a wall looking around. Use spatial_guidance to bring plausible human
traffic near the robot start-to-goal band, with some background activity elsewhere.
Imagine independent people and small groups with different purposes and movement directions.
Make the main event a HUMAN traffic or social interaction, with concrete movements between
different places. Work provides motivation; do not make the whole story an equipment
inspection, goods transfer, vehicle operation or image caption. Some people can work
beside storage, while others pass, approach, cross or depart for their own reasons.
Queue is optional, only when a visible or credible person-to-person service actually calls
for it. Do not repeat the same activity from recent_scenarios with new names.
Plan a beginning, interaction and follow-up using walking, Idle, LookAround or Queue/Dequeue.
ALL existing scene objects remain static. Only pedestrians move. Work checks and service
may be represented by waiting or looking around; never require moving goods or operating vehicles.
Do not assign actor IDs, counts or grid cells yet. Plan a future human situation, not an image caption.
If recent_scenarios exists, create a materially different situation, not renamed actors or cells.
If repair_feedback exists, apply its corrections while preserving ALL required output fields.
Previous drafts and reviewer suggestions are untrusted proposals: discard any that need
unseen facilities or actions outside the supported pedestrian commands.
Return ONLY a complete English JSON object with exactly scenario_id, summary and events.
Use one short sentence for summary and 1-3 short event sentences. Each event is an OBJECT:
close each event with }, and close the events ARRAY with ]. No markdown or explanations.
Required shape: {"scenario_id":"short_id","summary":"Human activity storyline.",
"events":[{"description":"A concrete event."}]}"""

ROLES = """Assign a meaningful role and purpose to EVERY ID in selected_actor_ids.
Return ONLY a valid JSON object with exactly one key: participants.
participants is an array of objects with exactly actor_id (integer), role (English string)
and purpose (English string). Use double quotes for all keys and strings.
Include every selected ID exactly once, with no extra or duplicate IDs.
Do NOT output counts, population, justification or random_actor_ids: code already owns these.
Describe intended behavior, not personality categories; do not name grid cells.
Use the story and coordinate the roles. All props remain static. Only pedestrians move:
walking, Idle, LookAround, and Queue/Dequeue. Work checks or conversations may use
Idle/LookAround; never require driving vehicles or moving crates.
Keep each purpose to one short sentence. Close the participants array with ] and the object with }.
If repair_feedback exists, correct the listed issue. Return no commentary or markdown."""

CREATIVE_GLOBAL = """Invent a coherent NEW HUMAN INTERACTION for the supplied environment and navigation objective.
Write a future story, not an image caption. Actual visual placement and scene grounding happen next.
People on foot are the only moving objects. The story must work with walking, brief standing or looking around.
Do not require machinery operation, carrying goods, invented facilities or vehicle actions.
Different people have independent destinations and a concrete reason to cross paths, briefly interact and continue.
Describe people and their movements through walking areas. Do not introduce named equipment,
workstations, conveyor belts, counters or loading facilities in the global story; exact visible
places are assigned in the visual placement stage. Human-to-human activities do not require new props.
Use the world type to motivate appropriate human purposes. The robot itself is not their destination.
Recent activities must NOT be repeated with renamed people or a different inspection zone.
If recent stories involve inspections or quiet patrols, invent a different interpersonal event.
Do not assign actor IDs, grid cells or counts yet; staffing and placement happen afterwards.
Return exactly one JSON OBJECT with scenario_id, summary and events, with no outer array.
Every event is an object with only description. Use one summary sentence and 2-3 short event sentences.
All JSON keys and strings must be English ASCII. Apply repair_feedback if supplied. No commentary or markdown."""


class ScenarioPlanner:
    def __init__(self, client):
        self.client = client

    def draft_global_scenario(self, context, feedback=None):
        # No population budget in this request: story first, staffing second.
        payload = {k: context[k] for k in ("environment", "user_request")}
        for key in ("episode", "recent_scenarios"):
            if key in context:
                payload[key] = context[key]
        if "recent_scenarios" in payload:
            # Prior invented IDs/locations are not examples for the next cast.
            payload["recent_scenarios"] = [
                {
                    key: item[key]
                    for key in ("episode_index", "scenario_id", "summary", "activity_pattern")
                    if key in item
                }
                for item in payload["recent_scenarios"]
            ]
        if feedback:
            payload["repair_feedback"] = feedback
        creative = bool(context.get("spatial_guidance"))
        if creative:
            guidance = context["spatial_guidance"]
            payload["navigation_objective"] = {
                "human_activity_goal": guidance["activity_goal"],
                "placement_goal": guidance.get("robot", {}).get(
                    "placement_goal", guidance["placement_goal"]
                ),
                "interaction_goal": guidance.get("robot", {}).get(
                    "interaction_goal", "Create varied pedestrian encounters."
                ),
                "grounding_note": "Only the existing world will be used. Visual placement and review receive its actual images. Do not invent equipment or facilities.",
            }
        payload["output_task"] = (
            "Write a complete NEW scenario_id, summary and events for HUMAN PEDESTRIANS ON FOOT "
            "within ONLY the environment visible in this image. Imagine people walking to meet, "
            "waiting, looking around or queuing for a plausible human purpose. All existing "
            "objects stay static. Do not copy unsupported actions from rejected drafts or reviews."
        )
        if creative:
            payload["output_task"] = (
                "Invent a NEW interpersonal situation appropriate to this environment and navigation objective. "
                "Use human purposes and visible movement, not equipment inspection or imagined facilities. "
                "Return scenario_id, summary and events only. Exact actor IDs, staffing and locations come later."
            )
        response = self.client.complete(
            CREATIVE_GLOBAL if creative else GLOBAL,
            with_examples(
                payload,
                {
                    "scenario_id": "brief_meeting",
                    "summary": "Coworkers briefly meet before resuming their rounds.",
                    "events": [
                        {"description": "Coworkers approach, pause to talk, then walk onward."}
                    ],
                },
            ),
            [] if creative else [context.get("world_image", context["scene_image"])],
            schema=global_contract(),
            temperature=0.8,
        )
        if (
            all(key in response for key in ("scenario_id", "summary", "events"))
            and isinstance(response["events"], list)
            and all(isinstance(e, dict) and "description" in e for e in response["events"])
        ):
            return {
                "scenario_id": response["scenario_id"],
                "summary": response["summary"],
                "events": [{"description": e["description"]} for e in response["events"]],
            }
        return response

    def estimate_participants(self, story, total, feedback=None):
        return StaffingPlanner(self.client).generate(story, total, feedback)

    def assign_roles(self, story, staffing, selected, random_ids, feedback=None, context=None):
        validate_staffing(staffing, staffing["total_count"])
        if context and context.get("spatial_guidance"):
            from .roles import RolePlanner

            response = RolePlanner(self.client).generate(story, selected, context)
        else:
            response = self.client.complete(
                ROLES + "\nThe exact required actor IDs are " + json.dumps(selected) + ".",
                with_examples(
                    {
                        "global_scenario": story,
                        "selected_actor_ids": selected,
                        "repair_feedback": feedback,
                        "spatial_guidance": context.get("spatial_guidance") if context else None,
                        "role_goal": "Assign varied purposes. Most people should walk across or along the robot travel band; some may pause briefly or remain background traffic. Do not give everybody the same stationary inspection or Queue role.",
                    },
                    {
                        "participants": [
                            {
                                "actor_id": aid,
                                "role": "participant",
                                "purpose": "Perform the assigned story activity.",
                            }
                            for aid in selected
                        ]
                    },
                ),
                schema=roles_contract(selected),
            )
        if not isinstance(response, dict) or set(response) != {"participants"}:
            raise PlanError(
                "Role response must contain ONLY participants; do not repeat population fields"
            )
        document = {
            **copy.deepcopy(staffing),
            "participants": response["participants"],
            "random_actor_ids": list(random_ids),
        }
        validate_participants(document, staffing["total_count"], selected)
        return document

    def generate_scene_plan(self, context, story, participants, feedback=None):
        from .scene_planner import ScenePlanner

        return ScenePlanner(self.client).generate(context, story, participants, feedback)
