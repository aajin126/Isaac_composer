"""Verify sampling, frame calibration and arrival semantics without Isaac Sim."""

import copy
import math
from types import SimpleNamespace

import numpy as np
import pytest

from isaac_composer.config import ConfigError, load_scenario_bundle, validate_scenario_config
from isaac_composer.robot_task import (
    TASK_DEFAULTS,
    ArrivalMonitor,
    RobotTaskPlanner,
    calibrate_world_from_odom,
    prepare_robot_task,
    rotation_2d,
    world_point_to_odom,
)
from test_pedestrian_routes import GridScene
from isaac_composer.runtime import SimulationRunner
from isaac_composer.navmesh import NavMeshScene
from unittest.mock import patch


def test_random_task_has_reachable_separated_endpoints_and_reproducible_heading():
    config = {**TASK_DEFAULTS, "enabled": True}
    first = RobotTaskPlanner(GridScene(), config, 43).plan()
    assert first == RobotTaskPlanner(GridScene(), config, 43).plan()
    assert first != RobotTaskPlanner(GridScene(), config, 44).plan()
    assert 5 <= first["straight_distance_m"] <= 15
    assert -math.pi <= first["spawn_yaw"] <= math.pi
    assert first["spawn_world"][2] == 0.65
    assert first["reference_path_world"][-1] == first["goal_world"]
    assert first["path_length_m"] >= first["straight_distance_m"]


def test_robot_queries_do_not_inherit_pedestrian_navigation_radius():
    scene = NavMeshScene({"agent_radius": 0.3, "agent_height": 1.8})
    scene.routing_radius_m = 0.35
    scene.rasterize = lambda margin_m: GridScene().raster
    calls = []

    class Mesh:
        def query_closest_point(self, *, target, agent_radius, agent_height):
            calls.append(agent_radius)
            return target, 0

    scene.navmesh = Mesh()
    planner = RobotTaskPlanner(scene, {**TASK_DEFAULTS, "enabled": True}, 43)
    with patch.dict("sys.modules", {"carb": SimpleNamespace(Float3=lambda *p: p)}):
        scene.project_world([1.0, 2.0, 0.0])
        planner.scene.project_world([1.0, 2.0, 0.0])
    assert calls == [0.35, 0.55]
    assert scene.routing_radius_m == 0.35


def test_unreachable_pairs_fail_with_bounded_retries():
    scene = GridScene()
    scene.shortest_path = lambda *args: (_ for _ in ()).throw(RuntimeError("disconnected"))
    config = {**TASK_DEFAULTS, "max_attempts": 10}
    with pytest.raises(RuntimeError, match="after 10 attempts"):
        RobotTaskPlanner(scene, config, 43).plan()


def test_task_updates_robot_before_pedestrian_sampling_and_preserves_rest_state(tmp_path):
    bundle = load_scenario_bundle("warehouse_a2_random")
    bundle["robot_task"].pop("map_file", None)  # This fixture supplies its own synthetic scene.
    task = prepare_robot_task(bundle, GridScene(), tmp_path)
    pose = bundle["robots"][0]["instance"]["pose"]
    assert pose["position"] == task["spawn_world"]
    assert pose["orientation_quat_wxyz"] == [
        math.cos(task["spawn_yaw"] / 2),
        0,
        0,
        math.sin(task["spawn_yaw"] / 2),
    ]
    assert (tmp_path / "robot_task.json").is_file()


def test_calibration_handles_random_origin_heading_and_nonzero_initial_odometry():
    origin = [12.0, -4.0, math.pi / 2]
    odom = [1.5, -2.0, 0.3]
    world_xy = np.asarray(origin[:2]) + rotation_2d(origin[2]) @ odom[:2]
    world = [*world_xy, origin[2] + odom[2]]
    inferred = calibrate_world_from_odom(world, odom)
    np.testing.assert_allclose(inferred, origin)
    np.testing.assert_allclose(world_point_to_odom([12.0, 6.0, 0], inferred), [10.0, 0], atol=1e-9)


def test_arrival_requires_proximity_and_sustained_low_actual_speed():
    monitor = ArrivalMonitor([5, 0], TASK_DEFAULTS)
    assert not monitor.update(0, [4.8, 0], [0.6, 0], 0)
    assert not monitor.update(0.1, [4.8, 0], [0, 0], 0)
    assert not monitor.update(0.5, [4.8, 0], [0, 0], 0)
    assert not monitor.update(0.6, [4.8, 0], [0, 0.3], 0)
    assert not monitor.update(0.7, [4.8, 0], [0, 0], 0)
    assert not monitor.update(1.0, [4.8, 0], [0, 0], 0.4)
    assert not monitor.update(1.1, [4.8, 0], [0, 0], 0)
    assert monitor.update(1.7, [4.8, 0], [0, 0], 0)


def test_goal_episode_can_run_past_sixty_seconds_and_finishes_only_on_arrival():
    fake = SimpleNamespace(
        simulation_app=SimpleNamespace(is_running=lambda: True),
        world=SimpleNamespace(current_time=0),
        frame_count=0,
        episode_start_sim_time=0,
        robot_task=SimpleNamespace(reached=False),
        setup=lambda: None,
    )

    def step():
        fake.world.current_time += 35
        fake.frame_count += 1
        fake.robot_task.reached = fake.frame_count == 3

    fake.step = step
    assert SimulationRunner.run(fake) == "goal_reached"
    assert fake.world.current_time == 105


@pytest.mark.parametrize(
    "change",
    [
        {"min_goal_distance_m": float("nan")},
        {"max_goal_distance_m": 1},
        {"clearance_m": 0.1},
        {"max_attempts": 0},
        {"enabled": "true"},
        {"map_file": 123},
    ],
)
def test_invalid_task_configuration_is_rejected_before_app_start(change):
    scenario = load_scenario_bundle("warehouse_a2_random")["scenario"]
    scenario = copy.deepcopy(scenario)
    scenario["robot_task"].update(change)
    with pytest.raises(ConfigError):
        validate_scenario_config(scenario)


def test_reviewed_robot_task_is_reused_instead_of_sampling_another_goal(tmp_path):
    bundle = load_scenario_bundle("warehouse_a2_random")
    bundle["robot_task"].pop("map_file", None)
    task = prepare_robot_task(bundle, GridScene(), tmp_path / "first")
    original = copy.deepcopy(task)
    bundle["pedestrians"]["seed"] = 999
    replay = prepare_robot_task(bundle, GridScene(), tmp_path / "replay", planned_task=task)
    assert replay["spawn_world"] == original["spawn_world"]
    assert replay["spawn_yaw"] == original["spawn_yaw"]
    assert replay["goal_world"] == original["goal_world"]
    assert task == original
