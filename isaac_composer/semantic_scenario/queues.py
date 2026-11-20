"""Structural contracts for native queue definitions and service actions."""

from .schema import PlanError, ActorFieldError


def queue_definitions(document, cells):
    result = {}
    definitions = document.get("queues", [])
    if not isinstance(definitions, list):
        raise PlanError("queues must be a list")
    for queue in definitions:
        qid = queue.get("queue_id")
        spots = queue.get("spots")
        if not isinstance(qid, str) or not qid.strip() or qid in result:
            raise PlanError("Each queue needs a unique queue_id")
        if (
            not isinstance(spots, list)
            or not spots
            or any(s not in cells for s in spots)
            or len(set(spots)) != len(spots)
        ):
            raise PlanError("Queue spots must be distinct valid cells, front first")
        result[qid] = queue
    return result


def validate_queue_step(step, definitions, start_kind):
    qid = step.get("queue_id")
    if not isinstance(qid, str) or qid not in definitions:
        raise ActorFieldError(
            "Queue action needs a defined queue_id",
            ("queue_id",),
            {"type": "string", "enum": list(definitions)},
        )
    if step["cell_id"] != definitions[qid]["spots"][0]:
        raise ActorFieldError(
            "Queue cell_id must be its front spot",
            ("cell_id",),
            {"const": definitions[qid]["spots"][0]},
        )
    if step.get("service_action", "Idle") not in ("Idle", "LookAround"):
        raise ActorFieldError(
            "Queue service_action must be Idle or LookAround",
            ("service_action",),
            {"type": "string", "enum": ["Idle", "LookAround"]},
        )
    if start_kind == "all_participants_arrived":
        raise ActorFieldError(
            "Queue scenes must not wait for every actor to reach the front together",
            ("action",),
            {"type": "string", "enum": ["Idle", "LookAround"]},
        )
