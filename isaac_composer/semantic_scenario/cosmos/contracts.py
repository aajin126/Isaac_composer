"""JSON decoding shapes; all activity choices remain the model's decisions."""

TEXT = {"type": "string", "minLength": 1}


def obj(properties):
    return {
        "type": "object",
        "properties": properties,
        "required": list(properties),
        "additionalProperties": False,
    }


def array(items, minimum=0, maximum=None):
    result = {"type": "array", "items": items, "minItems": minimum}
    if maximum is not None:
        result["maxItems"] = maximum
    return result


def global_contract():
    return obj(
        {"scenario_id": TEXT, "summary": TEXT, "events": array(obj({"description": TEXT}), 1, 3)}
    )


def staffing_contract(options):
    return obj({"option_id": {"type": "string", "enum": [o["option_id"] for o in options]}})


def roles_contract(selected):
    return obj(
        {
            "participants": array(
                obj(
                    {
                        "actor_id": {"type": "integer", "enum": selected},
                        "role": TEXT,
                        "purpose": TEXT,
                    }
                ),
                len(selected),
                len(selected),
            ),
        }
    )


def scenes_contract(cells, actors):
    cell = {"type": "string", "enum": list(cells)}
    condition = {
        "anyOf": [
            obj({"type": {"const": "episode_started"}}),
            obj({"type": {"const": "after_scene"}, "scene_id": TEXT}),
            obj({"type": {"const": "all_participants_arrived"}}),
            obj({"type": {"const": "all_participants_arrived"}, "after_scene_id": TEXT}),
        ]
    }
    stationary_step = obj(
        {
            "cell_id": cell,
            "action": {"type": "string", "enum": ["Idle", "LookAround"]},
            "duration_s": {"type": "number", "exclusiveMinimum": 0, "maximum": 120},
        }
    )
    step = {
        "anyOf": [
            stationary_step,
            obj({"cell_id": cell, "action": {"const": "GoTo"}, "duration_s": {"const": 0}}),
            obj(
                {
                    "cell_id": cell,
                    "action": {"const": "Queue"},
                    "queue_id": TEXT,
                    "duration_s": {"type": "number", "exclusiveMinimum": 0, "maximum": 120},
                    "service_action": {"type": "string", "enum": ["Idle", "LookAround"]},
                }
            ),
        ]
    }
    actor = obj(
        {
            "actor_id": {"type": "integer", "enum": actors},
            "start_cell": cell,
            "subgoals": array(step, 0, 4),
            "goal_cell": cell,
        }
    )
    scene = obj(
        {
            "scene_id": TEXT,
            "description": TEXT,
            "start_condition": condition,
            "actor_plans": array(actor, 1, len(actors)),
        }
    )
    return obj({"scenes": array(scene, 1, 6)})


def queues_contract(cells):
    return array(
        obj({"queue_id": TEXT, "spots": array({"type": "string", "enum": list(cells)}, 1)})
    )


def actor_route_contract(cells, actor_id, start_cells, queues, start_kind):
    """Restrict each actor's choices to the candidates in its current request."""
    actor = scenes_contract(cells, [actor_id])["properties"]["scenes"]["items"]["properties"][
        "actor_plans"
    ]["items"]
    actor["properties"]["actor_id"] = {"const": actor_id}
    actor["properties"]["start_cell"] = {"type": "string", "enum": list(start_cells)}
    steps = actor["properties"]["subgoals"]["items"]["anyOf"]
    stationary = steps[0]
    allowed = [stationary, steps[1], {"type": "string", "enum": list(cells)}]
    if start_kind != "all_participants_arrived":
        for qid, definition in queues.items():
            allowed.append(
                obj(
                    {
                        "cell_id": {"const": definition["spots"][0]},
                        "action": {"const": "Queue"},
                        "queue_id": {"const": qid},
                        "duration_s": {"type": "number", "exclusiveMinimum": 0, "maximum": 120},
                        "service_action": {"type": "string", "enum": ["Idle", "LookAround"]},
                    }
                )
            )
    actor["properties"]["subgoals"]["items"] = {"anyOf": allowed}
    return actor


def review_contract(actors):
    issue = obj(
        {
            "actor_id": {"enum": [None, *actors]},
            "problem": TEXT,
            "requested_change": TEXT,
            "scope": {"type": "string", "enum": ["global_scenario", "roles", "scenes"]},
        }
    )
    return obj({"passed": {"type": "boolean"}, "issues": array(issue, 0, 8)})
