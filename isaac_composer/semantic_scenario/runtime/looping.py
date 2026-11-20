"""Repeat approved activities; keep unrelated walkers on independent clocks."""

import copy
import math


def dependency(plan):
    condition = plan["start_condition"]
    return (
        condition.get("scene_id")
        if condition["type"] == "after_scene"
        else condition.get("after_scene_id")
    )


def execution_scenes(compiled):
    """Lower independent actors to separate scenes without changing source plans."""
    scenes = copy.deepcopy(compiled["scenes"])
    if not compiled.get("repeat_semantic_plan"):
        return scenes
    referenced = {dependency(s) for s in scenes}
    used_ids = {s["scene_id"] for s in scenes}
    result = []
    for scene in scenes:
        if (
            scene["start_condition"]["type"] == "episode_started"
            and scene["scene_id"] not in referenced
            and len(scene["actors"]) > 1
        ):
            for actor in scene["actors"]:
                sid = f'{scene["scene_id"]}__actor_{actor["actor_id"]}'
                while sid in used_ids:
                    sid += "_loop"
                used_ids.add(sid)
                result.append(
                    {
                        **scene,
                        "scene_id": sid,
                        "source_scene_id": scene["scene_id"],
                        "actors": [actor],
                    }
                )
        else:
            result.append(scene)
    return result


class SceneLoops:
    """Restart dependency/interaction groups, returning by native walking first."""

    def __init__(self, scenes, starts):
        self.plans = {sid: copy.deepcopy(item["plan"]) for sid, item in scenes.items()}
        self.groups = []
        remaining = set(scenes)
        # Scenes sharing an actor or a completion dependency belong to one loop.
        while remaining:
            members = {next(sid for sid in scenes if sid in remaining)}
            while True:
                actors = {a["actor_id"] for sid in members for a in self.plans[sid]["actors"]}
                linked = {
                    sid
                    for sid in remaining
                    if dependency(self.plans[sid]) in members
                    or sid in {dependency(self.plans[m]) for m in members}
                    or actors.intersection(a["actor_id"] for a in self.plans[sid]["actors"])
                }
                if linked <= members:
                    break
                members |= linked
            remaining -= members
            self.groups.append(
                {"scene_ids": [sid for sid in scenes if sid in members], "cycles": 0}
            )

        self.first_scene, targets = {}, {}
        for sid, plan in self.plans.items():
            for actor in plan["actors"]:
                aid = actor["actor_id"]
                self.first_scene.setdefault(aid, sid)
                targets.setdefault(aid, [("loop_start", starts[str(aid)])])
                targets[aid].extend((leg["cell_id"], leg["target"]) for leg in actor["legs"])
        self.returns = {}
        for aid, points in targets.items():
            legs, previous = [], points[-1][1]
            for cid, point in reversed(points[:-1]):
                if math.dist(previous[:2], point[:2]) <= 1e-5:
                    continue
                legs.append(
                    {
                        "cell_id": cid,
                        "target": point,
                        "action": "GoTo",
                        "duration_s": 0,
                        "activity_index": -1,
                    }
                )
                previous = point
            self.returns[aid] = legs

    def restart_completed(self, scenes, event):
        restarted = []
        for group in self.groups:
            ids = group["scene_ids"]
            if not all(scenes[sid]["status"] == "complete" for sid in ids):
                continue
            group["cycles"] += 1
            for sid in ids:
                plan = copy.deepcopy(self.plans[sid])
                for actor in plan["actors"]:
                    aid = actor["actor_id"]
                    if self.first_scene[aid] == sid:
                        actor["legs"] = copy.deepcopy(self.returns[aid]) + actor["legs"]
                scenes[sid].update(plan=plan, status="pending", actors={}, barrier_open=False)
                event("semantic_loop_restarted", sid, cycle=group["cycles"])
                if dependency(plan) is None:
                    restarted.append(sid)
        return restarted

    def actor_cycles(self):
        return {
            str(actor["actor_id"]): group["cycles"]
            for group in self.groups
            for sid in group["scene_ids"]
            for actor in self.plans[sid]["actors"]
        }
