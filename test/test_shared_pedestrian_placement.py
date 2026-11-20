"""Random and Cosmos use one placement policy, with native routes and no model fallback."""

import copy
import json
from pathlib import Path
from unittest.mock import patch
import pytest

from isaac_composer.config import load_scenario_bundle
from isaac_composer.pedestrian_placement import (
    PedestrianPlacement,
    segment_distance,
    polyline_distance,
)
from isaac_composer.pedestrian_routes import PedestrianRoutePlanner, prepare_pedestrian_routes
from isaac_composer.scene_grid import build_scene_grid
from isaac_composer.semantic_scenario.spatial import (
    enrich_context,
    start_candidates,
    placement_from_context,
)
from isaac_composer.semantic_scenario.runtime.compiler import build_mixed_people
from test_pedestrian_routes import GridScene


class NativeEndpoints(GridScene):
    """Grid path-query fixture accepting exact coarse-grid centers as endpoints."""

    agent_radius = 0.3
    floor_z = 0.0

    def bounds(self, margin_m=0):
        return self.raster.bounds

    def walkable_triangles(self):
        yield [(0, 0, 0), (20, 0, 0), (20, 20, 0)]
        yield [(0, 0, 0), (20, 20, 0), (0, 20, 0)]

    def is_allowed_world(self, point):
        try:
            self.project_world(point)
            return True
        except RuntimeError:
            return False

    def shortest_path(self, start, goal):
        path = super().shortest_path(start, goal)
        return [list(start), *path[1:-1], list(goal)]


TASK = {
    "spawn_world": [1, 10, 0.65],
    "goal_world": [19, 10, 0],
    "reference_path_world": [[1, 10, 0], [19, 10, 0]],
    "reference_path_source": "native_navmesh",
    "pedestrian_spawn_separation_m": 1.5,
}


def placement(scene, width=3):
    return PedestrianPlacement(build_scene_grid(scene, 2).to_dict(), 43, TASK, width, 1.0)


def test_random_starts_use_exactly_the_candidates_exposed_to_cosmos():
    scene = NativeEndpoints()
    shared = placement(scene)
    context = enrich_context(
        {"grid": shared.grid, "episode": {"seed": 43}}, 43, TASK, route_band_fraction=1.0
    )
    config = {"mode": "random", "count": 10, "repeat": True}
    planner = PedestrianRoutePlanner(scene, config, 43, [TASK["spawn_world"]], shared)
    # A configured common pool must never silently fall back to whole-map sampling.
    planner._sample = lambda: (_ for _ in ()).throw(AssertionError("Unexpected whole-map sample"))
    people = planner.plan()
    assert people == PedestrianRoutePlanner(scene, config, 43, [TASK["spawn_world"]], shared).plan()
    assigned = {}
    for aid, person in enumerate(people):
        available = [cid for cid in shared.cells if cid not in assigned.values()]
        options = start_candidates(context, available, assigned, aid, len(people))
        assert options == shared.start_candidates(available, assigned, aid, len(people))
        cid = shared.cell_for_position(person["pose"]["position"])
        assert cid in options
        assigned[aid] = cid
        assert shared.route_distance(person["pose"]["position"]) <= 3
        assert shared.route_distance(person["waypoints"][-1]) <= 3
        assert person["repeat"]
        assert all(scene.is_allowed_world(p) for p in person["waypoints"])
    # NavMesh detours can leave the placement band to pass the doorway safely.
    assert any(
        segment_distance(p, TASK["spawn_world"], TASK["goal_world"]) > 3
        for person in people
        for p in person["waypoints"]
    )


def test_cosmos_random_remainder_uses_the_same_pool_without_actor_id_exceptions():
    scene = NativeEndpoints()
    shared = placement(scene)
    start = shared.cells[next(cid for cid, p in shared.cells.items() if tuple(p[:2]) == (3.0, 9.0))]
    compiled = {
        "participant_plan": {
            "total_count": 5,
            "participants": [
                {"actor_id": 0, "role": "worker", "purpose": "Walk through the aisle."}
            ],
            "random_actor_ids": [1, 2, 3, 4],
        },
        "starts": {"0": start},
        "scenes": [
            {
                "actors": [
                    {"actor_id": 0, "legs": [{"path": [start, [7, 9, 0]], "target": [7, 9, 0]}]}
                ]
            }
        ],
    }
    planner = PedestrianRoutePlanner(scene, {"repeat": True}, 43, [TASK["spawn_world"]], shared)
    planner._sample = lambda: (_ for _ in ()).throw(AssertionError("Unexpected whole-map sample"))
    people = build_mixed_people(compiled, planner)
    assert len(people) == 5 and people[0]["planner_source"] == "cosmos"
    assert all(shared.route_distance(p["pose"]["position"]) <= 3 for p in people)
    assert all(p["repeat"] and p["planner_source"] == "random" for p in people[1:])


def test_random_preparation_samples_real_task_first_and_saves_common_inputs(tmp_path):
    scene = NativeEndpoints()
    bundle = load_scenario_bundle("warehouse_a2_random")
    bundle["pedestrians"]["generation"].update(count=5, scene={}, output_dir=str(tmp_path))
    bundle["pedestrians"]["placement"] = {
        "route_band_half_width_m": 3,
        "cell_size_m": 2,
        "route_band_fraction": 1.0,
    }
    bundle["robot_task"].update(enabled=True, output_dir=str(tmp_path))

    def task_first(bundle, native, output):
        bundle["robot_task_instance"] = copy.deepcopy(TASK)
        bundle["robots"][0]["instance"]["pose"]["position"] = TASK["spawn_world"]

    with patch("isaac_composer.pedestrian_routes.NavMeshScene", return_value=scene), patch(
        "isaac_composer.robot_task.prepare_robot_task", side_effect=task_first
    ), patch(
        "isaac_composer.semantic_scenario.cosmos.client.CosmosClient.complete",
        side_effect=AssertionError("Random mode must not call Cosmos"),
    ):
        prepare_pedestrian_routes(bundle, Path("."))
    saved = json.loads((tmp_path / "pedestrian_placement.json").read_text())
    assert saved["robot_task"] == TASK
    assert saved["route_band_half_width_m"] == 3
    assert len(saved["actors"]) == 5
    assert all(p["start_route_distance_m"] <= 3 for p in saved["actors"])
    assert "semantic_execution" not in bundle["pedestrians"]
    assert not (tmp_path / "semantic_plan").exists()


def test_random_and_cosmos_candidates_follow_native_doorway_detour_instead_of_the_chord():
    scene = NativeEndpoints()
    task = copy.deepcopy(TASK)
    task["reference_path_world"] = scene.shortest_path([1, 10, 0], task["goal_world"])
    grid = build_scene_grid(scene, 2).to_dict()
    shared = PedestrianPlacement(grid, 43, task, 2, 1.0)
    context = enrich_context({"grid": grid, "episode": {"seed": 43}}, 43, task, 2, 1.0)
    restored = placement_from_context(context)
    assert restored.robot_path == shared.robot_path
    assert restored.priority_cells == shared.priority_cells
    # The doorway is well away from the direct start-goal line, but on the native route.
    detour_cells = [
        cid
        for cid in shared.priority_cells
        if segment_distance(shared.cells[cid], task["spawn_world"], task["goal_world"]) > 3
    ]
    assert detour_cells
    people = PedestrianRoutePlanner(
        scene, {"mode": "random", "count": 8, "repeat": True}, 43, [task["spawn_world"]], shared
    ).plan()
    assigned = {}
    for aid, person in enumerate(people):
        available = [cid for cid in shared.cells if cid not in assigned.values()]
        choices = start_candidates(context, available, assigned, aid, len(people))
        assert choices == shared.start_candidates(available, assigned, aid, len(people))
        cid = shared.cell_for_position(person["pose"]["position"])
        assert cid in choices
        assigned[aid] = cid
        assert shared.route_distance(person["pose"]["position"]) <= 2
        assert shared.route_distance(person["waypoints"][-1]) <= 2
        assert all(scene.is_allowed_world(p) for p in person["waypoints"])
    assert any(
        segment_distance(p["pose"]["position"], task["spawn_world"], task["goal_world"]) > 3
        for p in people
    )


def test_u_shaped_path_prioritizes_its_far_corner_and_excludes_the_direct_chord():
    grid = {
        "bounds_xy": [-10, -10, 10, 10],
        "cell_size_m": 2,
        "cells": [
            {"id": cid, "target_world": point}
            for cid, point in [("corner", [0, 6, 0]), ("chord", [0, -4, 0]), ("side", [8, 0, 0])]
        ],
    }
    task = {
        "spawn_world": [-8, -4, 0.65],
        "goal_world": [8, -4, 0],
        "reference_path_world": [[-8, -4, 0], [-8, 6, 0], [8, 6, 0], [8, -4, 0]],
    }
    shared = PedestrianPlacement(grid, 43, task, 1.5, 1.0)
    assert set(shared.priority_cells) == {"corner", "side"}
    assert shared.route_distance([0, -4, 0]) == 8
    assert polyline_distance([0, 6, 10], shared.robot_path) == 0  # XY placement, independent of z.
    assert set(shared.start_candidates(shared.cells, {}, 0, 2)) == {"corner", "side"}


@pytest.mark.parametrize(
    "path",
    [[], [[1, 10, 0]], [[1, 10, 0], [18, 10, 0]], [[1, 10, 0], [float("nan"), 15, 0], [19, 10, 0]]],
)
def test_missing_or_invalid_native_reference_does_not_silently_fall_back_to_a_straight_line(path):
    task = {**TASK, "reference_path_world": path}
    with pytest.raises(ValueError, match="reference|Reference"):
        PedestrianPlacement(
            {"cells": [], "bounds_xy": [0, 0, 20, 20], "cell_size_m": 2}, robot_task=task
        )


def test_old_semantic_context_reuses_its_saved_native_task_path():
    scene = NativeEndpoints()
    shared = placement(scene)
    context = enrich_context(
        {"grid": shared.grid, "episode": {"seed": 43}}, 43, TASK, route_band_fraction=1.0
    )
    robot = context["spatial_guidance"]["robot"]
    del robot["reference_path_world"]
    robot["straight_route_band_half_width_m"] = robot.pop("route_band_half_width_m")
    restored = placement_from_context(context)
    assert restored.robot_path == shared.robot_path and restored.width == 3


@pytest.mark.parametrize("fraction,near_count", [(0, 0), (0.5, 5), (0.7, 7), (1, 10)])
def test_mixed_placement_has_exact_seeded_groups_and_unbiased_global_candidates(
    fraction, near_count
):
    scene = NativeEndpoints()
    grid = build_scene_grid(scene, 2).to_dict()
    shared = PedestrianPlacement(grid, 43, TASK, 3, fraction)
    context = enrich_context({"grid": grid, "episode": {"seed": 43}}, 43, TASK, 3, fraction)
    context["placement_population_count"] = 10
    near = shared.route_actor_ids(10)
    assert len(near) == near_count
    assert near == placement_from_context(context).route_actor_ids(10)
    for aid in range(10):
        choices = shared.start_candidates(shared.cells, {}, aid, 10)
        assert choices == start_candidates(context, list(shared.cells), {}, aid, 10)
        goals = shared.goal_candidates([1, 10, 0], 3, aid, 10)
        if aid in near:
            assert all(shared.route_distance(shared.cells[c]) <= 3 for c in choices + goals)
        else:
            assert any(shared.route_distance(shared.cells[c]) > 3 for c in choices)
            assert any(shared.route_distance(shared.cells[c]) > 3 for c in goals)
            assert len(goals) > len(shared.priority_cells)
    people = PedestrianRoutePlanner(
        scene, {"mode": "random", "count": 10, "repeat": True}, 43, [TASK["spawn_world"]], shared
    ).plan()
    assert all(shared.route_distance(people[aid]["pose"]["position"]) <= 3 for aid in near)
    assert all(person["repeat"] for person in people)


def test_group_assignment_changes_with_seed_and_not_with_actor_processing_order():
    scene = NativeEndpoints()
    grid = build_scene_grid(scene, 2).to_dict()
    first = PedestrianPlacement(grid, 43, TASK)
    assert len(first.route_actor_ids(10)) == 7
    assert first.route_actor_ids(20) == first.route_actor_ids(20)
    assert any(
        first.route_actor_ids(10) != PedestrianPlacement(grid, seed, TASK).route_actor_ids(10)
        for seed in range(44, 50)
    )
    before = {aid: first.group(aid, 10) for aid in range(10)}
    after = {aid: first.group(aid, 10) for aid in reversed(range(10))}
    assert before == after


@pytest.mark.parametrize("fraction", [-0.01, 1.01, float("nan"), float("inf")])
def test_invalid_mixture_fraction_is_rejected(fraction):
    with pytest.raises(ValueError, match="fraction"):
        PedestrianPlacement(
            {"cells": [], "bounds_xy": [0, 0, 20, 20]},
            robot_task=TASK,
            route_band_fraction=fraction,
        )
