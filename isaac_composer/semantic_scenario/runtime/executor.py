"""Activity state machine; locomotion still belongs to native NavMesh and ORCA."""

from __future__ import annotations

from collections import deque
import math
from pathlib import Path
import json

import numpy as np

from .compiler import native_path
from .looping import SceneLoops, dependency, execution_scenes


class SceneExecutor:
    def __init__(
        self,
        scene,
        people,
        compiled,
        output_dir=None,
        arrival_radius_m=0.3,
    ):
        self.scene, self.people, self.compiled = scene, people, compiled
        self.arrival_radius = arrival_radius_m
        self.elapsed = 0.0
        self.history = deque()
        self.events = []
        self.scenes = {
            s["scene_id"]: {"plan": s, "status": "pending", "actors": {}, "barrier_open": False}
            for s in execution_scenes(compiled)
        }
        self.actor_ids = {p["actor_id"] for p in compiled["participant_plan"]["participants"]}
        self.actions = {aid: "Idle" for aid in self.actor_ids}
        self.stream = None
        if output_dir:
            directory = Path(output_dir)
            directory.mkdir(parents=True, exist_ok=True)
            self.stream = (directory / "execution_events.jsonl").open("w", encoding="utf-8")
        self.nominal_speeds = {i: float(people[i]._target_speed) for i in self.actor_ids}
        self.loops = (
            SceneLoops(self.scenes, compiled["starts"])
            if compiled.get("repeat_semantic_plan")
            else None
        )
        self.queues = None
        if compiled.get("queues"):
            from .native_queue import NativeQueues

            self.queues = NativeQueues(
                scene, people, compiled["queues"], self.nominal_speeds, arrival_radius_m
            )
        for aid in self.actor_ids:
            people[aid]._target_speed = 0.0

    def _event(self, event, scene_id=None, actor_id=None, **fields):
        packet = {"time_s": self.elapsed, "event": event, **fields}
        if scene_id is not None:
            packet["scene_id"] = scene_id
            source = self.scenes[scene_id]["plan"].get("source_scene_id")
            if source:
                packet["source_scene_id"] = source
        if actor_id is not None:
            packet["actor_id"] = actor_id
        self.events.append(packet)
        if self.stream:
            self.stream.write(json.dumps(packet, allow_nan=False) + "\n")
            self.stream.flush()

    def _begin_leg(self, sid, state, positions):
        aid = state["actor_id"]
        leg = state["legs"][state["index"]]
        if leg["action"] == "Queue" and not state.get("queue_ready"):
            self.queues.begin(aid, "Queue", leg["queue_id"])
            state["phase"] = "queueing"
            self._event("queue_joined", sid, aid, queue_id=leg["queue_id"])
            return
        try:
            path = native_path(self.scene, positions[aid], leg["target"])
        except (RuntimeError, ValueError):
            self.people[aid]._target_speed = 0.0
            state["phase"], state["retry_at"] = "blocked", self.elapsed + 1.0
            self._event("path_blocked", sid, aid, cell_id=leg["cell_id"])
            return
        self.people[aid]._current_point_index = 0
        self.people[aid].update_target_position(path, walk_speed=self.nominal_speeds[aid])
        state["phase"] = "moving"
        self.actions[aid] = None
        self._event("movement_started", sid, aid, cell_id=leg["cell_id"])

    def _advance(self, sid, state, positions):
        leg = state["legs"][state["index"]]
        if leg["action"] == "Queue" and not state.get("dequeue_done"):
            target = state["legs"][state["index"] + 1]["target"]
            self.queues.begin(state["actor_id"], "Dequeue", leg["queue_id"], target)
            state["phase"] = "dequeueing"
            self.actions[state["actor_id"]] = None
            self._event("dequeue_started", sid, state["actor_id"], queue_id=leg["queue_id"])
            return
        state["index"] += 1
        state["queue_ready"], state["dequeue_done"] = False, False
        state["dwell_s"], state["started"] = 0.0, False
        if state["index"] >= len(state["legs"]):
            state["phase"] = "complete"
            self.people[state["actor_id"]]._target_speed = 0.0
            self.actions[state["actor_id"]] = "Idle"
            self._event("actor_scene_completed", sid, state["actor_id"])
        else:
            self._begin_leg(sid, state, positions)

    def _start_scene(self, sid, item, positions):
        item["status"] = "active"
        self._event("scene_started", sid)
        for actor in item["plan"]["actors"]:
            state = {**actor, "index": 0, "phase": "moving", "dwell_s": 0.0, "started": False}
            item["actors"][actor["actor_id"]] = state
            self._begin_leg(sid, state, positions)

    def step(self, dt, positions):
        self.elapsed += dt
        positions = np.asarray(positions, float)
        self.history.append((self.elapsed, positions.copy()))
        while len(self.history) > 1 and self.elapsed - self.history[1][0] >= 0.2:
            self.history.popleft()
        past_t, past_p = self.history[0]
        velocities = np.linalg.norm(positions[:, :2] - past_p[:, :2], axis=1) / max(
            self.elapsed - past_t, 1e-6
        )
        for aid in self.actor_ids:
            self.actions[aid] = "Idle"
        for sid, item in self.scenes.items():
            plan = item["plan"]
            condition = plan["start_condition"]
            parent = dependency(plan)
            if item["status"] == "pending":
                if parent and self.scenes[parent]["status"] != "complete":
                    continue
                self._start_scene(sid, item, positions)
            if item["status"] != "active":
                continue
            # First update arrivals for the full scene, then release its common
            # first activity. No actor starts a joint dwell before peers arrive.
            for aid, state in item["actors"].items():
                if state["phase"] == "complete":
                    continue
                if state["phase"] in {"queueing", "dequeueing"}:
                    phase = state["phase"]
                    done, waiting = self.queues.step(aid, dt, velocities[aid])
                    self.actions[aid] = "Idle" if waiting else None
                    if done:
                        if phase == "queueing":
                            state["queue_ready"] = True
                            state["phase"] = "arrived"
                            self._event("queue_front_reached", sid, aid)
                        else:
                            state["dequeue_done"] = True
                            self._event("dequeue_completed", sid, aid)
                            self._advance(sid, state, positions)
                    continue
                if state["phase"] == "blocked":
                    if self.elapsed >= state["retry_at"]:
                        self._begin_leg(sid, state, positions)
                    continue
                leg = state["legs"][state["index"]]
                near = math.dist(positions[aid, :2], leg["target"][:2]) <= self.arrival_radius
                if state["phase"] == "moving" and near:
                    self.people[aid]._target_speed = 0.0
                    state["phase"] = "arrived"
                    self._event("arrived", sid, aid, cell_id=leg["cell_id"])
                elif state["phase"] == "moving":
                    self.people[aid]._target_speed = self.nominal_speeds[aid]
                    self.actions[aid] = None
                elif not near:
                    # ORCA may move a waiting actor aside. Rejoin the target;
                    # paused dwell time never counts as completed work.
                    self._begin_leg(sid, state, positions)
            if condition["type"] == "all_participants_arrived" and not item["barrier_open"]:
                ready = all(
                    state["phase"] in {"arrived", "holding"}
                    and state["legs"][state["index"]]["activity_index"] == 0
                    and velocities[aid] <= 0.15
                    for aid, state in item["actors"].items()
                )
                if ready:
                    item["barrier_open"] = True
                    self._event("joint_activity_released", sid)
            for aid, state in item["actors"].items():
                if state["phase"] not in {"arrived", "holding"}:
                    continue
                leg = state["legs"][state["index"]]
                if leg["action"] == "GoTo":
                    self._advance(sid, state, positions)
                    continue
                if velocities[aid] > 0.15:
                    continue
                if (
                    condition["type"] == "all_participants_arrived"
                    and leg["activity_index"] == 0
                    and not item["barrier_open"]
                ):
                    continue
                if not state["started"]:
                    state["started"] = True
                    self._event(
                        "activity_started",
                        sid,
                        aid,
                        cell_id=leg["cell_id"],
                        action=(
                            leg.get("service_action", "Idle")
                            if leg["action"] == "Queue"
                            else leg["action"]
                        ),
                    )
                state["phase"] = "holding"
                self.actions[aid] = (
                    leg.get("service_action", "Idle") if leg["action"] == "Queue" else leg["action"]
                )
                state["dwell_s"] += dt
                if state["dwell_s"] + 1e-9 >= leg["duration_s"]:
                    self._event(
                        "activity_completed",
                        sid,
                        aid,
                        cell_id=leg["cell_id"],
                        action=(
                            leg.get("service_action", "Idle")
                            if leg["action"] == "Queue"
                            else leg["action"]
                        ),
                        duration_s=state["dwell_s"],
                    )
                    self._advance(sid, state, positions)
            if all(s["phase"] == "complete" for s in item["actors"].values()):
                item["status"] = "complete"
                self._event("scene_completed", sid)
        if self.loops:
            for sid in self.loops.restart_completed(self.scenes, self._event):
                # Issue the return movement in this same control tick, without
                # a final Idle frame or waiting for unrelated actors.
                self._start_scene(sid, self.scenes[sid], positions)

    def action(self, actor_id):
        return self.actions.get(actor_id)

    def diagnostics(self):
        return {
            "elapsed_s": self.elapsed,
            "scenes": {sid: s["status"] for sid, s in self.scenes.items()},
            "completed": all(s["status"] == "complete" for s in self.scenes.values()),
            "queue_backend": "omni.anim.people.QueueCmd/Dequeue" if self.queues else None,
            "repeat_semantic_plan": self.loops is not None,
            "actor_cycle_counts": self.loops.actor_cycles() if self.loops else {},
            "actors": {
                str(aid): {
                    "scene_id": sid,
                    "phase": state["phase"],
                    "leg_index": state["index"],
                    "dwell_s": state["dwell_s"],
                    "action": self.action(aid),
                }
                for sid, item in self.scenes.items()
                for aid, state in item["actors"].items()
            },
        }

    def close(self):
        self._event("execution_stopped", completed=self.diagnostics()["completed"])
        if self.queues:
            self.queues.close()
        if self.stream:
            self.stream.close()
            self.stream = None
