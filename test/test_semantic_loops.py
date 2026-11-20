"""Semantic loop transitions, native return targets, and interaction barriers."""

import copy

import numpy as np

from isaac_composer.semantic_scenario.runtime.executor import SceneExecutor


class Floor:
    def shortest_path(self, a, b):
        return [a, b]


class Walker:
    def __init__(self):
        self._target_speed = 1.0
        self._current_point_index = 0
        self.path = []

    def update_target_position(self, path, walk_speed):
        self.path = path
        self._target_speed = walk_speed


def leg(x, y=0, action="GoTo", duration=0, index=-1):
    return {
        "cell_id": f"C{x}_{y}",
        "target": [x, y, 0],
        "action": action,
        "duration_s": duration,
        "activity_index": index,
    }


def compiled(actors, kind="episode_started"):
    return {
        "repeat_semantic_plan": True,
        "participant_plan": {"participants": [{"actor_id": a["actor_id"]} for a in actors]},
        "starts": {str(a["actor_id"]): [0, a["actor_id"] * 2, 0] for a in actors},
        "scenes": [
            {
                "scene_id": "work",
                "description": "Walk and pause.",
                "start_condition": {"type": kind},
                "actors": actors,
            }
        ],
    }


def test_short_walk_loops_without_waiting_for_a_distant_independent_actor():
    plan = compiled([{"actor_id": 0, "legs": [leg(2)]}, {"actor_id": 1, "legs": [leg(20, 2)]}])
    original = copy.deepcopy(plan)
    people = [Walker(), Walker()]
    executor = SceneExecutor(Floor(), people, plan)
    executor.step(0.1, [[0, 0, 0], [0, 2, 0]])
    executor.step(0.1, [[2, 0, 0], [0, 2, 0]])
    assert executor.diagnostics()["actor_cycle_counts"] == {"0": 1, "1": 0}
    assert people[0].path[-1] == [0, 0, 0]
    assert people[0]._target_speed == 1 and executor.action(0) is None
    executor.step(0.1, [[0, 0, 0], [0, 2, 0]])
    assert people[0].path[-1] == [2, 0, 0]
    executor.step(0.1, [[2, 0, 0], [0, 2, 0]])
    assert executor.diagnostics()["actor_cycle_counts"]["0"] == 2
    assert not executor.diagnostics()["completed"]
    assert plan == original  # The reviewed source is immutable.


def test_return_retraces_subgoals_and_repeats_the_original_activity():
    plan = compiled(
        [{"actor_id": 0, "legs": [leg(2, action="LookAround", duration=0.2, index=0), leg(4)]}]
    )
    person = Walker()
    executor = SceneExecutor(Floor(), [person], plan)
    executor.step(0.1, [[0, 0, 0]])
    for _ in range(6):
        executor.step(0.1, [[2, 0, 0]])
    assert person.path[-1] == [4, 0, 0]
    executor.step(0.1, [[4, 0, 0]])
    assert person.path[-1] == [2, 0, 0]  # Reverse through the prior subgoal.
    executor.step(0.1, [[2, 0, 0]])
    assert person.path[-1] == [0, 0, 0]
    executor.step(0.1, [[0, 0, 0]])
    assert person.path[-1] == [2, 0, 0]
    for _ in range(6):
        executor.step(0.1, [[2, 0, 0]])
    activities = [e for e in executor.events if e["event"] == "activity_completed"]
    assert len(activities) == 2 and all(e["action"] == "LookAround" for e in activities)
    assert person.path[-1] == [4, 0, 0]


def test_joint_barrier_is_reset_and_requires_every_participant_on_each_cycle():
    plan = compiled(
        [
            {"actor_id": aid, "legs": [leg(2, aid * 2, "Idle", 0.2, 0), leg(4, aid * 2)]}
            for aid in range(2)
        ],
        "all_participants_arrived",
    )
    people = [Walker(), Walker()]
    executor = SceneExecutor(Floor(), people, plan)
    pos = np.array([[2, 0, 0], [2, 2, 0]], float)
    for _ in range(6):
        executor.step(0.1, pos)
    pos[:, 0] = 4
    executor.step(0.1, pos)
    assert executor.diagnostics()["actor_cycle_counts"] == {"0": 1, "1": 1}
    assert not executor.scenes["work"]["barrier_open"]
    pos[:, 0] = 2
    executor.step(0.1, pos)
    pos[:, 0] = 0
    executor.step(0.1, pos)
    pos[0, 0] = 2
    for _ in range(6):
        executor.step(0.1, pos)
    assert len([e for e in executor.events if e["event"] == "activity_started"]) == 2
    pos[1, 0] = 2
    for _ in range(6):
        executor.step(0.1, pos)
    assert len([e for e in executor.events if e["event"] == "activity_started"]) == 4


def test_scene_dependency_is_preserved_when_the_group_restarts():
    plan = compiled([{"actor_id": 0, "legs": [leg(2)]}])
    plan["scenes"].append(
        {
            "scene_id": "next",
            "description": "Continue after work.",
            "start_condition": {"type": "after_scene", "scene_id": "work"},
            "actors": [{"actor_id": 0, "legs": [leg(4)]}],
        }
    )
    executor = SceneExecutor(Floor(), [Walker()], plan)
    executor.step(0.1, [[0, 0, 0]])
    assert executor.scenes["next"]["status"] == "pending"
    executor.step(0.1, [[2, 0, 0]])
    assert executor.scenes["next"]["status"] == "active"
    executor.step(0.1, [[4, 0, 0]])
    assert executor.scenes["work"]["status"] == "active"
    assert executor.scenes["next"]["status"] == "pending"
    assert executor.diagnostics()["actor_cycle_counts"] == {"0": 1}


def test_repeated_queue_activity_rejoins_and_dequeues_on_every_cycle(monkeypatch):
    from isaac_composer.semantic_scenario.runtime import native_queue

    commands = []

    class QueueBackend:
        def __init__(self, *args):
            pass

        def begin(self, aid, action, queue_id, target=None):
            commands.append(action)

        def step(self, aid, dt, speed):
            return True, False

    monkeypatch.setattr(native_queue, "NativeQueues", QueueBackend)
    service = {**leg(0, action="Queue", duration=0.1, index=0), "queue_id": "service"}
    plan = compiled([{"actor_id": 0, "legs": [service, leg(2)]}])
    plan["queues"] = [{"queue_id": "service"}]
    person = Walker()
    executor = SceneExecutor(Floor(), [person], plan)
    position = [0, 0, 0]
    for _ in range(40):
        executor.step(0.1, [position])
        state = executor.scenes["work"]["actors"][0]
        if state["phase"] == "moving":
            position = person.path[-1]
        elif state["phase"] == "queueing":
            position = [0, 0, 0]
        elif state["phase"] == "dequeueing":
            position = [2, 0, 0]
    assert executor.diagnostics()["actor_cycle_counts"]["0"] >= 2
    assert commands[:4] == ["Queue", "Dequeue", "Queue", "Dequeue"]
