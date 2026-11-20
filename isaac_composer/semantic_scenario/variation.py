"""Detect exact reuse; semantic diversity remains a Cosmos planning decision."""

import hashlib
import json
from collections import Counter


def story_key(story):
    # Renaming scenario_id alone does not create a new storyline.
    text = " ".join([story["summary"], *[e["description"] for e in story["events"]]])
    return hashlib.sha256(" ".join(text.casefold().split()).encode()).hexdigest()


def motion_key(plan):
    # Ignore actor renumbering and scene labels; compare actual requested actions.
    scenes = []
    queues = {q["queue_id"]: q["spots"] for q in plan.get("queues", [])}
    for scene in plan["scenes"]:
        routes = [
            {
                "start_cell": a["start_cell"],
                "goal_cell": a["goal_cell"],
                "subgoals": [
                    {
                        **{k: v for k, v in step.items() if k != "queue_id"},
                        **({"queue_spots": queues[step["queue_id"]]} if "queue_id" in step else {}),
                    }
                    for step in a["subgoals"]
                ],
            }
            for a in scene["actor_plans"]
        ]
        scenes.append(
            {
                "start_type": scene["start_condition"]["type"],
                "routes": sorted(routes, key=lambda a: json.dumps(a, sort_keys=True)),
            }
        )
    return hashlib.sha256(json.dumps(scenes, sort_keys=True).encode()).hexdigest()


def history_entry(index, story, plan):
    actors = [a for s in plan["scenes"] for a in s["actor_plans"]]
    actions = Counter(step["action"] for actor in actors for step in actor["subgoals"])
    return {
        "episode_index": index,
        "scenario_id": story["scenario_id"],
        "summary": story["summary"],
        "events": story["events"],
        "story_sha256": story_key(story),
        "motion_sha256": motion_key(plan),
        "activity_pattern": {
            "scene_count": len(plan["scenes"]),
            "stop_action_counts": dict(actions),
            "distinct_goal_count": len({a["goal_cell"] for a in actors}),
            "stationary_route_count": sum(
                all(
                    cid == a["start_cell"]
                    for cid in [a["goal_cell"], *[s["cell_id"] for s in a["subgoals"]]]
                )
                for a in actors
            ),
        },
    }
