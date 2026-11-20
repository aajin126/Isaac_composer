"""Robot-conditioned placement, navigation intent and ongoing traffic."""

import copy
import math

from isaac_composer.semantic_scenario.spatial import (
    enrich_context,
    start_candidates,
    segment_distance,
)
from isaac_composer.semantic_scenario.navigation_quality import inspect_navigation


def context():
    cells = [
        {"id": f"C{i}_{j}", "target_world": [i, j, 0], "bounds_xy": [i - 1, j - 1, i + 1, j + 1]}
        for j in range(-8, 9, 2)
        for i in range(-8, 9, 2)
    ]
    original = {
        "grid": {"bounds_xy": [-10, -10, 10, 10], "cell_size_m": 2, "cells": cells},
        "episode": {"seed": 17},
        "valid_cells": [c["id"] for c in cells],
    }
    return enrich_context(
        original,
        17,
        {
            "spawn_world": [-8, 0, 0.65],
            "goal_world": [8, 0, 0],
            "reference_path_world": [[-8, 0, 0], [8, 0, 0]],
        },
        route_band_half_width_m=1.5,
        route_band_fraction=1.0,
    )


def test_default_band_expands_to_three_meters_and_custom_width_changes_candidates():
    narrow = context()
    wider = enrich_context(narrow, 17, narrow["robot_task"])
    custom = enrich_context(narrow, 17, narrow["robot_task"], route_band_half_width_m=5.0)
    narrow_cells = set(narrow["spatial_guidance"]["robot"]["priority_cells"])
    wider_cells = set(wider["spatial_guidance"]["robot"]["priority_cells"])
    custom_cells = set(custom["spatial_guidance"]["robot"]["priority_cells"])
    assert narrow_cells < wider_cells < custom_cells
    assert wider["spatial_guidance"]["robot"]["route_band_half_width_m"] == 3.0
    assert "C0_2" in wider_cells and "C0_4" not in wider_cells
    assert "C0_4" in custom_cells


def test_first_option_bias_still_produces_spread_near_the_actual_robot_line():
    scene = context()
    assigned = {}
    for aid in range(10):
        available = [c for c in scene["valid_cells"] if c not in assigned.values()]
        options = start_candidates(scene, available, assigned, aid, 10)
        assert options and set(options) <= set(available)
        assigned[aid] = options[0]  # Deliberately mimic the observed model bias.
    lookup = {c["id"]: c["target_world"] for c in scene["grid"]["cells"]}
    points = [lookup[c] for c in assigned.values()]
    assert sum(segment_distance(p, [-8, 0], [8, 0]) <= 2.5 for p in points) >= 7
    assert max(p[0] for p in points) - min(p[0] for p in points) >= 12
    assert all(p[1] == 0 for p in points[:8])
    assert max(abs(p[1]) for p in points) <= 2
    assert all(math.dist(p[:2], [-8, 0]) >= 1.5 for p in points)
    assert len(set(assigned.values())) == 10


def test_same_seed_is_reproducible_and_new_seed_changes_candidate_order():
    scene = context()
    options = start_candidates(scene, scene["valid_cells"], {}, 0, 10)
    assert options == start_candidates(scene, scene["valid_cells"], {}, 0, 10)
    other = copy.deepcopy(scene)
    other["episode"]["seed"] += 1
    assert options != start_candidates(other, other["valid_cells"], {}, 0, 10)


def test_standing_at_a_distant_boundary_cannot_pass_navigation_quality():
    scene = context()
    people = {"participants": [{"actor_id": i} for i in range(10)]}
    plans = [
        {
            "actor_id": i,
            "start_cell": f"C{-8+2*(i%9)}_8",
            "subgoals": [{"cell_id": f"C{-8+2*(i%9)}_8", "action": "LookAround", "duration_s": 5}],
            "goal_cell": f"C{-8+2*(i%9)}_8",
        }
        for i in range(10)
    ]
    report = inspect_navigation(
        scene, people, {"scenes": [{"scene_id": "traffic", "actor_plans": plans}]}
    )
    assert not report["passed"]
    assert report["metrics"]["mobile_actor_count"] == 0
    assert report["metrics"]["near_route_mobile_actor_count"] == 0
    assert len({i["actor_id"] for i in report["issues"]}) >= 6


def test_crossing_and_along_line_traffic_passes_without_forcing_queues():
    scene = context()
    people = {"participants": [{"actor_id": i} for i in range(10)]}
    plans = [
        {
            "actor_id": i,
            "start_cell": f"C{-8+2*(i%9)}_-2",
            "subgoals": [],
            "goal_cell": f"C{-8+2*(i%9)}_2",
        }
        for i in range(10)
    ]
    report = inspect_navigation(
        scene, people, {"scenes": [{"scene_id": "traffic", "actor_plans": plans}]}
    )
    assert report["passed"]
    assert report["metrics"]["near_route_mobile_actor_count"] == 10
    assert report["metrics"]["queue_actor_count"] == 0


def test_actor_three_and_seven_do_not_get_remote_background_slots():
    scene = context()
    lookup = {c["id"]: c["target_world"] for c in scene["grid"]["cells"]}
    for seed in range(20):
        scene["episode"]["seed"] = seed
        assigned = {}
        for aid in range(8):
            available = [c for c in scene["valid_cells"] if c not in assigned.values()]
            options = start_candidates(scene, available, assigned, aid, 10)
            assert all(lookup[c][1] == 0 for c in options)
            assigned[aid] = options[0]


def test_long_lookaround_waits_do_not_count_as_a_moving_navigation_cast():
    scene = context()
    people = {"participants": [{"actor_id": i} for i in range(10)]}
    plans = [
        {
            "actor_id": i,
            "start_cell": f"C{-8+2*(i%9)}_-2",
            "subgoals": [{"cell_id": f"C{-8+2*(i%9)}_2", "action": "LookAround", "duration_s": 90}],
            "goal_cell": f"C{-8+2*(i%9)}_2",
        }
        for i in range(10)
    ]
    report = inspect_navigation(
        scene, people, {"scenes": [{"scene_id": "traffic", "actor_plans": plans}]}
    )
    assert not report["passed"]
    assert report["metrics"]["mobile_actor_count"] == 0


def test_pending_planning_gives_actual_robot_task_to_model_and_keeps_native_calls_on_main_thread(
    tmp_path, monkeypatch
):
    import json
    import threading
    from PIL import Image
    from isaac_composer.semantic_scenario.runtime import planning, path_validation
    from isaac_composer.semantic_scenario.schema import save_json

    base = context()
    image = tmp_path / "source.png"
    Image.new("RGB", (400, 400), "gray").save(image)
    base["scene_image"] = str(image)
    request = {
        "context": base,
        "seed": 17,
        "base_url": "http://test.invalid",
        "model": "test_model",
        "total_count": 1,
        "max_revisions": 2,
        "episode_index": 3,
        "route_band_half_width_m": 4.5,
    }
    save_json(tmp_path / "planning_request.json", request)
    actual = {
        "spawn_world": [-6, 0, 0.65],
        "goal_world": [6, 0, 0],
        "reference_path_world": [[-6, 0, 0], [6, 0, 0]],
    }
    main_id = threading.get_ident()
    calls, updates = [], []

    def inspect(scene, ctx, people, plan):
        calls.append((threading.get_ident(), ctx["robot_task"]))
        return None

    monkeypatch.setattr(path_validation, "inspect_native_paths", inspect)

    class Pipeline:
        def __init__(self, *args, native_validator=None, **kwargs):
            self.validator = native_validator

        def run(self, ctx, total, directory, seed):
            assert threading.get_ident() != main_id
            assert ctx["spatial_guidance"]["robot"]["route_band_half_width_m"] == 4.5
            assert "C0_4" in ctx["spatial_guidance"]["robot"]["priority_cells"]
            self.validator(ctx, {}, {})
            save_json(
                tmp_path / "participant_plan.json",
                {"cosmos_count": 1, "random_count": 0, "total_count": 1, "participants": []},
            )
            save_json(
                tmp_path / "global_scenario.json",
                {"scenario_id": "traffic", "summary": "Walk across the aisle.", "events": []},
            )
            return {"scenes": []}

    monkeypatch.setattr(planning, "ScenarioPipeline", Pipeline)
    planning.finish_pending_plan(tmp_path, actual, lambda: updates.append(1), scene=object())
    assert calls == [(main_id, actual)] and updates
    assert json.loads((tmp_path / "episode_planning.json").read_text())["planning_pending"] is False


def detour_context():
    scene = context()
    return enrich_context(
        scene,
        17,
        {
            "spawn_world": [-8, -4, 0.65],
            "goal_world": [8, -4, 0],
            "reference_path_world": [[-8, -4, 0], [-8, 6, 0], [8, 6, 0], [8, -4, 0]],
        },
        1.5,
        1.0,
    )


def test_navigation_quality_checks_the_curved_route_instead_of_the_start_goal_chord():
    scene = detour_context()
    participants = {"participants": [{"actor_id": 0}]}
    actor = {"actor_id": 0, "start_cell": "C-2_-6", "goal_cell": "C2_-2", "subgoals": []}
    plan = {"scenes": [{"scene_id": "crossing", "actor_plans": [actor]}]}
    report = inspect_navigation(scene, participants, plan)
    assert report["metrics"]["mobile_actor_count"] == 1
    assert report["metrics"]["near_route_mobile_actor_count"] == 0
    assert not report["passed"]
    actor.update(start_cell="C-2_6", goal_cell="C2_6")
    report = inspect_navigation(scene, participants, plan)
    assert report["passed"]
    assert report["metrics"]["near_route_mobile_actor_count"] == 1


def test_cosmos_input_image_highlights_the_detour_without_painting_a_direct_line(tmp_path):
    from PIL import Image
    from isaac_composer.semantic_scenario.scene_input import render_robot_context

    scene = detour_context()
    source = tmp_path / "source.png"
    Image.new("RGB", (200, 200), "black").save(source)
    scene["scene_image"] = str(source)
    output = tmp_path / "route.png"
    render_robot_context(scene, output)
    with Image.open(output) as image:
        r, g, b = image.getpixel((100, 40))  # World (0, 6): actual route.
        assert r > 150 and g > 150 and b < 20
        assert image.getpixel((100, 140)) == (0, 0, 0)  # World (0, -4): only the chord.


def test_global_random_semantic_actor_is_not_forced_back_toward_the_robot_route():
    scene = context()
    scene["spatial_guidance"]["robot"]["route_band_fraction"] = 0
    participants = {"total_count": 10, "participants": [{"actor_id": 9}]}
    actor = {"actor_id": 9, "start_cell": "C-2_8", "goal_cell": "C2_8", "subgoals": []}
    report = inspect_navigation(
        scene, participants, {"scenes": [{"scene_id": "background", "actor_plans": [actor]}]}
    )
    assert report["passed"]
    assert report["metrics"]["near_route_mobile_actor_count"] == 0
    assert report["metrics"]["minimum_near_route_mobile_actor_count"] == 0
