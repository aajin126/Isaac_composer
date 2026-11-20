"""Measurable navigation intent; semantic meaning still requires Cosmos review."""

import math
from .spatial import placement_from_context
from ..pedestrian_placement import segment_path_distance


def inspect_navigation(context, participants, plan):
    if not context.get("spatial_guidance", {}).get("robot"):
        return {"passed": True, "issues": [], "status": "no_robot_context"}
    cells = {c["id"]: c["target_world"] for c in context["grid"]["cells"]}
    priority = set(context["spatial_guidance"]["robot"]["priority_cells"])
    ids = [p["actor_id"] for p in participants["participants"]]
    routes = {aid: [] for aid in ids}
    queue_users = set()
    dwell = {aid: 0.0 for aid in ids}
    for scene in plan["scenes"]:
        for actor in scene["actor_plans"]:
            route = [
                actor["start_cell"],
                *[s["cell_id"] for s in actor["subgoals"]],
                actor["goal_cell"],
            ]
            routes[actor["actor_id"]].extend(route)
            dwell[actor["actor_id"]] += sum(s["duration_s"] for s in actor["subgoals"])
            if any(s["action"] == "Queue" for s in actor["subgoals"]):
                queue_users.add(actor["actor_id"])
    minimum_m = max(3.0, context["grid"]["cell_size_m"] * 1.5)
    distances = {
        aid: sum(math.dist(cells[a][:2], cells[b][:2]) for a, b in zip(route[:-1], route[1:]))
        for aid, route in routes.items()
    }
    mobile = [
        aid
        for aid in ids
        if distances[aid] >= minimum_m
        and distances[aid] / max(1e-6, distances[aid] + dwell[aid]) >= 0.4
    ]
    placement = placement_from_context(context)
    total = participants.get(
        "total_count", context.get("placement_population_count", max(ids, default=-1) + 1)
    )
    route_actors = placement.route_actor_ids(total) & set(ids)
    exposed = [
        aid
        for aid in mobile
        if any(cid in priority for cid in routes[aid])
        or any(
            segment_path_distance(cells[a], cells[b], placement.robot_path) <= placement.width
            for a, b in zip(routes[aid][:-1], routes[aid][1:])
        )
    ]
    min_mobile = math.ceil(len(ids) * 0.6)
    min_exposed = min(math.ceil(len(ids) * 0.5), len(route_actors)) if priority else 0
    max_queue = max(2, math.ceil(len(ids) * 0.3))
    issues = []
    actors = {a["actor_id"]: (s["scene_id"], a) for s in plan["scenes"] for a in s["actor_plans"]}

    def movement_issue(aid):
        sid, actor = actors[aid]
        origin = cells[actor["start_cell"]]
        required = max(minimum_m, dwell[aid] * 2 / 3)
        near_route = aid in route_actors
        pool = [
            cid
            for cid in context["valid_cells"]
            if math.dist(origin[:2], cells[cid][:2]) >= required
            and (not near_route or cid in priority)
        ]
        pool.sort(key=lambda cid: sum(a["goal_cell"] == cid for _, a in actors.values()))
        return {
            "actor_id": aid,
            "scope": "scenes",
            "scene_id": sid,
            "field_path": ["goal_cell"],
            "available_cells": pool,
            "problem": (
                "This actor needs meaningful moving traffic near the robot reference path."
                if near_route
                else "This actor needs meaningful movement in its permitted walkable area."
            ),
            "requested_change": f"Choose a different goal from available_cells at least {required:g} meters from its start. Preserve the other actors and existing activities.",
        }

    for aid in [i for i in ids if i not in mobile][: max(0, min_mobile - len(mobile))]:
        issues.append(movement_issue(aid))
    already = {i["actor_id"] for i in issues}
    preferred = mobile + [a for a in ids if a not in mobile]
    for aid in [
        i for i in preferred if i in route_actors and i not in exposed and i not in already
    ][: max(0, min_exposed - len(exposed) - len(already & route_actors))]:
        issues.append(movement_issue(aid))
    for aid in sorted(queue_users)[max_queue:]:
        issues.append(
            {
                "actor_id": aid,
                "scope": "scenes",
                "problem": "The cast is dominated by Queue activities.",
                "requested_change": "Give this actor a plausible moving activity instead of Queue, preserving other actors and queue definitions.",
            }
        )
    return {
        "passed": not issues,
        "issues": issues,
        "metrics": {
            "planned_actor_count": len(ids),
            "mobile_actor_count": len(mobile),
            "minimum_mobile_actor_count": min_mobile,
            "near_route_mobile_actor_count": len(exposed),
            "minimum_near_route_mobile_actor_count": min_exposed,
            "queue_actor_count": len(queue_users),
            "maximum_queue_actor_count": max_queue,
            "total_dwell_s": {str(aid): seconds for aid, seconds in dwell.items()},
            "intended_travel_m": {str(aid): distance for aid, distance in distances.items()},
        },
        "measurement_note": "Subgoal intent only; actual collision-free paths and motion are checked in Isaac.",
    }
