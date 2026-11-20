"""Map-only robot paths, cached search, boundary recovery and visible stale guidance."""

import copy
import math
from pathlib import Path
from unittest.mock import patch

import numpy as np
from PIL import Image
import pytest
import yaml

from isaac_composer.static_map_path import StaticMapPath, configured_static_map
from isaac_composer.goal_route import GoalRoute
from isaac_composer.robot_task import RobotTaskPlanner, prepare_robot_task
from isaac_composer.robot_task_config import TASK_DEFAULTS
from isaac_composer.config import load_scenario_bundle
from test_pedestrian_routes import GridScene


def map_scene(tmp_path, pixels=None, clearance=0, origin=(0, 0, 0), resolution=1):
    if pixels is None:
        pixels = np.full((20, 20), 255, np.uint8)
        pixels[:, 10] = 0
        pixels[2:5, 10] = 255
    Image.fromarray(pixels).save(tmp_path / "map.png")
    path = tmp_path / "map.yaml"
    path.write_text(
        yaml.safe_dump(
            {
                "image": "map.png",
                "resolution": resolution,
                "origin": list(origin),
                "negate": 0,
                "free_thresh": 0.196,
                "occupied_thresh": 0.65,
            }
        )
    )
    return StaticMapPath(path, clearance)


def test_map_detour_avoids_the_wall_and_reuses_the_goal_search(tmp_path):
    scene = map_scene(tmp_path)
    start, goal = [4.5, 4.5, 0], [15.5, 4.5, 0]
    path = scene.shortest_path(start, goal)
    assert path[0] == start and path[-1] == goal
    assert max(p[1] for p in path) >= 15
    assert all(scene.allows_segment(a, b) for a, b in zip(path, path[1:]))
    assert sum(math.dist(a, b) for a, b in zip(path, path[1:])) > math.dist(start, goal)
    distances = scene._distances
    next_path = scene.shortest_path([4.5, 5.5, 0], goal)
    assert scene._distances is distances
    assert next_path[-1] == goal
    scene.shortest_path(start, [15.5, 3.5, 0])
    assert scene._distances is not distances


def test_diagonal_corner_cutting_and_unknown_space_are_blocked(tmp_path):
    pixels = np.array([[255, 0], [0, 255]], np.uint8)
    scene = map_scene(tmp_path, pixels)
    a, b = scene.world_point(0, 0), scene.world_point(1, 1)
    assert not scene.allows_segment(a, b)
    with pytest.raises(RuntimeError, match="disconnected"):
        scene.shortest_path(a, b)
    pixels[0, 1] = 205  # ROS unknown gray is still blocked.
    scene = map_scene(tmp_path, pixels)
    assert not scene.is_allowed_world(scene.world_point(0, 1))


def test_rotated_ros_map_origin_and_clearance_are_respected(tmp_path):
    scene = map_scene(tmp_path, origin=(10, -10, math.pi / 2), clearance=1)
    start, goal = scene.world_point(15, 3), scene.world_point(15, 15)
    assert scene.cell(start) == (15, 3)
    path = scene.shortest_path(start, goal)
    assert all(scene.allows_segment(a, b) for a, b in zip(path, path[1:]))


def test_near_wall_start_can_recover_into_padded_map_without_crossing_obstacles(tmp_path):
    pixels = np.full((20, 20), 255, np.uint8)
    pixels[:, 0] = 0
    scene = map_scene(tmp_path, pixels, clearance=0.65, resolution=0.1)
    start = [0.15, 1.0, 0]
    assert not scene.is_allowed_world(start)
    recovered = scene.project_world(start)
    assert scene.is_allowed_world(recovered)
    assert scene.allows_segment(start, recovered, raw=True)
    with pytest.raises(RuntimeError, match="occupied"):
        scene.project_world([0.05, 1.0, 0])


def test_static_map_route_and_near_wall_updates_do_not_call_any_navmesh_query(tmp_path):
    pixels = np.full((20, 20), 255, np.uint8)
    pixels[:, 0] = 0
    scene = map_scene(tmp_path, pixels, clearance=0.65, resolution=0.1)
    initial, goal = [0.85, 1.0, 0], [1.25, 1.0, 0]
    task = {
        "config": dict(TASK_DEFAULTS),
        "spawn_world": [*initial[:2], 0.65],
        "goal_world": goal,
        "reference_path_world": scene.shortest_path(initial, goal),
    }
    route = GoalRoute(scene, task)
    route.update(initial, 0)
    assert route.update([0.15, 1.0, 0], 0.5)
    assert route.error is None
    assert route.start_adjustment_m > 0.35  # Old NavMesh projection gate would reject this.
    assert route.points[0][:2] == [0.15, 1.0]
    assert scene.allows_segment(route.points[0], route.points[1], raw=True)
    previous = copy.deepcopy(route.points)
    assert route.update([0.05, 0.6, 0], 1.0)
    assert route.error and route.points == previous
    assert route.update(initial, 1.5)
    assert route.error is None


def test_task_uses_map_path_even_when_native_path_queries_always_fail(tmp_path):
    static = map_scene(tmp_path)
    native = GridScene()
    native.shortest_path = lambda *_: (_ for _ in ()).throw(
        AssertionError("Native path was called")
    )
    planner = RobotTaskPlanner(native, {**TASK_DEFAULTS, "max_attempts": 50}, 43, path_scene=static)
    task = planner.plan()
    assert task["reference_path_source"] == "static_occupancy_map"
    assert all(
        static.allows_segment(a, b)
        for a, b in zip(task["reference_path_world"], task["reference_path_world"][1:])
    )


@pytest.mark.parametrize("preset", ["warehouse_a2_random", "hospital_a2_random"])
def test_default_presets_select_their_actual_static_map(preset):
    bundle = load_scenario_bundle(preset)
    scene = configured_static_map(bundle, GridScene())
    assert scene.path.is_file()
    assert scene.path_source == "static_occupancy_map"
    assert scene.clearance_m == bundle["robot_task"]["clearance_m"]


def test_prepared_robot_task_saves_the_map_source_and_shares_the_same_runtime_planner(tmp_path):
    static = map_scene(tmp_path)
    bundle = load_scenario_bundle("warehouse_a2_random")
    with patch("isaac_composer.static_map_path.configured_static_map", return_value=static):
        task = prepare_robot_task(bundle, GridScene(), tmp_path / "output")
    assert bundle["robot_navigation_scene"] is static
    assert task["path_map_file"] == str(static.path)
    assert task["reference_path_source"] == "static_occupancy_map"
