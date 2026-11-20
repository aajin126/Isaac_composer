"""Short per-person role requests avoid regenerating an entire malformed cast."""

from ..schema import PlanError, LocalRepairFailed, english_json
from .client import CosmosError
from .contracts import obj, TEXT
from .output_examples import with_examples


class RolePlanner:
    def __init__(self, client):
        self.client = client

    def generate(self, story, selected, context):
        roles = []
        for aid in selected:
            error = None
            for attempt in range(3):
                payload = {
                    "requested_actor_id": aid,
                    "global_scenario": story,
                    "other_assigned_roles": roles,
                    "activity_goal": context["spatial_guidance"]["activity_goal"],
                    "output_task": "Assign ONE person a specific purpose in the human interaction. Return ONLY role and purpose as short English ASCII strings. Do not return participants, IDs, counts, routes or grid cells.",
                }
                if error:
                    payload["repair_feedback"] = str(error)
                try:
                    response = self.client.complete(
                        "Assign ONE pedestrian role. Write a role of 2-4 words and a purpose of one short sentence. "
                        "Give different people complementary moving activities instead of the same inspection. "
                        'Only humans move; props stay static. Return exactly {"role":"...","purpose":"..."}.',
                        with_examples(
                            payload,
                            {
                                "role": "passing coworker",
                                "purpose": "Walk past the activity area toward another work area.",
                            },
                        ),
                        schema=obj({"role": TEXT, "purpose": TEXT}),
                    )
                    english_json(response)
                    if set(response) != {"role", "purpose"} or any(
                        not isinstance(v, str) or not v.strip() for v in response.values()
                    ):
                        raise PlanError(
                            "Return only two nonempty English strings: role and purpose"
                        )
                    roles.append({"actor_id": aid, **response})
                    print(f"SEMANTIC_ROLE actor={aid} role={response['role']}", flush=True)
                    break
                except (PlanError, CosmosError, TypeError, AttributeError) as exc:
                    error = exc
                    print(f"SEMANTIC_ROLE_REPAIR actor={aid} reason={error}", flush=True)
            else:
                raise LocalRepairFailed(f"Actor {aid} role failed: {error}") from error
        return {"participants": roles}
