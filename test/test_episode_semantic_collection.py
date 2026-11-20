"""Every episode plans afresh, keeps world context and avoids prior story reuse."""

import copy
import json
from pathlib import Path
from PIL import Image
import pytest

from isaac_composer.semantic_scenario import collection
from isaac_composer.semantic_scenario.collection import EpisodeScenarioPlanner
from isaac_composer.semantic_scenario.runtime.compiler import load_reviewed_plan
from isaac_composer.semantic_scenario.variation import story_key, motion_key


class EpisodeClient:
    instances = []

    def __init__(self, *args, seed=None, **kwargs):
        self.model, self.seed = "fixture_cosmos", seed
        self.calls = []
        self.instances.append(self)

    def complete(self, instruction, payload, images=(), schema=None, temperature=0.3):
        self.calls.append((instruction, copy.deepcopy(payload), list(images)))
        if instruction.startswith("Invent a coherent"):
            self.index = payload["episode"]["episode_index"]
            return {
                "scenario_id": f"event_{self.index}",
                "summary": f"A distinct work situation {self.index}.",
                "events": [{"description": f"Work coordination sequence {self.index}."}],
            }
        if instruction.startswith("Choose one staffing option"):
            return {"option_id": "cast_001"}
        if instruction.startswith("Assign ONE"):
            return {"role": "coworker", "purpose": "Coordinate work."}
        if instruction.startswith("Assign"):
            return {
                "participants": [
                    {"actor_id": aid, "role": "coworker", "purpose": "Coordinate work."}
                    for aid in payload["selected_actor_ids"]
                ],
            }
        if instruction.startswith("Plan the scene timeline"):
            return {
                "scenes": [
                    {
                        "scene_id": "scene_01",
                        "description": "Coordinate work.",
                        "start_condition": {"type": "episode_started"},
                        "actor_ids": [
                            p["actor_id"] for p in payload["participant_plan"]["participants"]
                        ],
                    }
                ]
            }
        if instruction.startswith("Choose an initial"):
            return {"start_cell": ["A1", "B1", "C1"][payload["actor_id"]]}
        if instruction.startswith("Plan ONLY"):
            aid = payload["requested_actor"]["actor_id"]
            cells = ["A1", "B1", "C1"]
            return {
                "actor_id": aid,
                "start_cell": cells[aid],
                "subgoals": [
                    {
                        "cell_id": cells[(aid + self.index + 1) % 3],
                        "action": "Idle",
                        "duration_s": 1,
                    }
                ],
                "goal_cell": cells[(aid + self.index + 1) % 3],
            }
        return {"passed": True, "issues": []}


@pytest.fixture
def environment(tmp_path, monkeypatch):
    package = tmp_path / "package"
    (package / "config").mkdir(parents=True)
    usd = package / "warehouse.usd"
    usd.write_text("warehouse geometry")
    bundle = {
        "config_root": str(package / "config"),
        "environment": {"name": "warehouse", "asset": {"usd_path": str(usd)}},
        "pedestrians": {
            "navmesh": {"agent_radius": 0.3},
            "generation": {"count": 3, "scene": {}},
            "social_navigation": {"enabled": True},
        },
    }
    monkeypatch.setattr(collection, "load_scenario_bundle", lambda _: copy.deepcopy(bundle))
    EpisodeClient.instances = []
    monkeypatch.setattr(collection, "CosmosClient", EpisodeClient)
    exports = []

    def export(self, current, key):
        exports.append(current["environment"]["name"])
        directory = tmp_path / f"context_{len(exports)}"
        directory.mkdir()
        for name in ("world_scene.png", "grid_scene.png"):
            Image.new("RGB", (600, 200), "gray").save(directory / name)
        grid = {
            "environment": current["environment"]["name"],
            "source_usd": current["environment"]["asset"]["usd_path"],
            "cell_size_m": 2,
            "bounds_xy": [0, 0, 6, 2],
            "cells": [
                {"id": c, "bounds_xy": [x - 1, 0, x + 1, 2], "target_world": [x, 1, 0]}
                for c, x in zip(["A1", "B1", "C1"], [1, 3, 5])
            ],
        }
        (directory / "grid_cells.json").write_text(json.dumps(grid))
        return directory

    monkeypatch.setattr(EpisodeScenarioPlanner, "_export_context", export)
    return bundle, exports


def test_two_episodes_have_fresh_requests_history_seeds_and_separate_plans(
    environment, tmp_path, capsys
):
    _, exports = environment
    root = tmp_path / "data"
    planner = EpisodeScenarioPlanner("warehouse_a2_random", root)
    metadata = [planner.prepare(i, root / f"episode_{i:06d}", 43 + i) for i in range(2)]
    assert exports == ["warehouse"]
    assert len(EpisodeClient.instances) == 2
    assert [c.seed for c in EpisodeClient.instances] == [43, 44]
    assert metadata[0]["plan_dir"] != metadata[1]["plan_dir"]
    first, second = [c.calls[0][1] for c in EpisodeClient.instances]
    assert not first["recent_scenarios"]
    assert second["recent_scenarios"][0]["scenario_id"] == "event_0"
    assert "total_count" not in first and "total_count" not in second
    assert second["environment"] == "warehouse"
    assert metadata[1]["cosmos_count"] == 2 and metadata[1]["random_count"] == 1
    for item in metadata:
        load_reviewed_plan(item["plan_dir"])
    output = capsys.readouterr().out
    assert "SEMANTIC_SCENARIO_DRAFT id=event_0" in output
    assert "SEMANTIC_EPISODE_READY episode=000001" in output
    assert "Story: A distinct work situation 1." in output
    assert "Population: 2 Cosmos + 1 random = 3" in output
    assert "P0: coworker - Coordinate work." in output
    assert "P0: A1 -> C1 (Idle, 1s) -> C1" in output


def test_world_change_refreshes_image_and_drops_other_world_history(environment, tmp_path):
    bundle, exports = environment
    root = tmp_path / "data"
    planner = EpisodeScenarioPlanner("a_world", root)
    planner.prepare(0, root / "episode_000000", 43)
    bundle["environment"]["name"] = "hospital"
    bundle["environment"]["asset"]["usd_path"] = "/world/hospital.usd"
    planner.prepare(1, root / "episode_000001", 44)
    assert exports == ["warehouse", "hospital"]
    request = EpisodeClient.instances[-1].calls[0][1]
    assert request["environment"] == "hospital" and not request["recent_scenarios"]


def test_restart_uses_saved_history_but_refreshes_world_input(environment, tmp_path):
    _, exports = environment
    root = tmp_path / "data"
    EpisodeScenarioPlanner("world", root).prepare(0, root / "episode_000000", 43)
    EpisodeScenarioPlanner("world", root).prepare(1, root / "episode_000001", 44)
    assert exports == ["warehouse", "warehouse"]
    assert EpisodeClient.instances[-1].calls[0][1]["recent_scenarios"][0]["episode_index"] == 0


def test_exact_reuse_ignores_actor_and_scenario_renaming():
    story = {
        "scenario_id": "old",
        "summary": "Wait for a check.",
        "events": [{"description": "Advance after inspection."}],
    }
    assert story_key(story) == story_key({**story, "scenario_id": "new"})
    plan = {
        "scenes": [
            {
                "scene_id": "one",
                "start_condition": {"type": "episode_started"},
                "actor_plans": [
                    {"actor_id": 0, "start_cell": "A1", "subgoals": [], "goal_cell": "B1"}
                ],
            }
        ]
    }
    renamed = copy.deepcopy(plan)
    renamed["scenes"][0]["scene_id"] = "two"
    renamed["scenes"][0]["actor_plans"][0]["actor_id"] = 5
    assert motion_key(plan) == motion_key(renamed)


def test_robot_episodes_defer_model_requests_until_the_actual_native_task_exists(
    environment, tmp_path
):
    root = tmp_path / "data"
    metadata = EpisodeScenarioPlanner(
        "warehouse_a2_random",
        root,
        robot_task=True,
        route_band_half_width_m=4.5,
        route_band_fraction=0.5,
    ).prepare(0, root / "episode_000000", 43)
    assert metadata["planning_pending"] is True
    directory = Path(metadata["plan_dir"])
    assert json.loads((directory / "approval.json").read_text())["approved"] is False
    request = json.loads((directory / "planning_request.json").read_text())
    assert request["seed"] == 43 and request["total_count"] == 3
    assert request["route_band_half_width_m"] == 4.5
    assert request["route_band_fraction"] == 0.5
    assert not EpisodeClient.instances
