"""Native detours, robot footprint isolation, and bounded guidance updates."""

import copy
from types import SimpleNamespace
from unittest.mock import patch

import pytest
import numpy as np

from isaac_composer.goal_route import GoalRoute
from isaac_composer.navmesh import NavMeshScene
from isaac_composer.robot_task_config import TASK_DEFAULTS


def fixture():
    calls = []

    class Path:
        def get_point_count(self):
            return 4

        def get_points(self):
            # The direct diagonal crosses an obstacle; native path uses corners.
            return [(0, 0, 0), (0, 3, 0), (4, 3, 0), (4, 4, 0)]

    class Mesh:
        def query_closest_point(self, **kwargs):
            calls.append(("project", kwargs))
            return kwargs["target"], 0

        def query_shortest_path(self, **kwargs):
            calls.append(("path", kwargs))
            return Path()

    scene = NavMeshScene({"agent_radius": 0.3, "agent_height": 1.8})
    scene.routing_radius_m = 0.35
    scene.navmesh = Mesh()
    task = {
        "config": dict(TASK_DEFAULTS),
        "spawn_world": [0, 0, 0.65],
        "goal_world": [4, 4, 0],
        "reference_path_world": [[0, 0, 0], [0, 3, 0], [4, 3, 0], [4, 4, 0]],
    }
    return scene, task, calls


def test_native_corner_path_is_preserved_and_robot_radius_does_not_change_pedestrians():
    scene, task, calls = fixture()
    before = copy.deepcopy(task)
    route = GoalRoute(scene, task)
    assert not route.update([0, 0, 1.0], 0)
    with patch.dict("sys.modules", {"carb": SimpleNamespace(Float3=lambda *p: p)}):
        assert route.update([0, 0.5, 1.0], 0.5)
    assert len(route.points) == 4
    assert route.length_m == 8
    assert route.length_m > 2**0.5 * 4
    assert [c[0] for c in calls] == ["project", "path"]
    assert all(c[1]["agent_radius"] == 0.55 for c in calls)
    assert scene.agent_radius == 0.3 and scene.routing_radius_m == 0.35
    assert task == before


def test_stationary_robot_and_render_frames_do_not_requery_the_mesh():
    scene, task, calls = fixture()
    route = GoalRoute(scene, task)
    route.update([0, 0, 0], 0)
    for frame in range(1, 501):
        route.update([0.1, 0.0, 0], frame * 0.02)
    assert calls == []
    with patch.dict("sys.modules", {"carb": SimpleNamespace(Float3=lambda *p: p)}):
        assert route.update([0, 0.5, 0], 10.1)
        assert not route.update([0, 1.0, 0], 10.2)
        assert route.update([0, 1.0, 0], 10.6)
    assert sum(name == "path" for name, _ in calls) == 2


def test_unreachable_update_keeps_last_route_and_retry_is_throttled_even_without_movement():
    scene, task, _ = fixture()
    route = GoalRoute(scene, task)
    original = copy.deepcopy(route.points)
    route.update([0, 0, 0], 0)
    route.scene.project_world = lambda point: point
    attempts = []

    def fail(*args):
        attempts.append(args)
        raise RuntimeError("Disconnected islands")

    route.scene.shortest_path = fail
    assert route.update([0, 1, 0], 0.5)
    assert route.points == original and route.error == "Disconnected islands"
    assert not route.update([0, 1, 0], 0.8)
    assert route.update([0, 1, 0], 1.0)
    assert len(attempts) == 2
    route.scene.shortest_path = lambda a, b: [a, b]
    assert route.update([0, 1, 0], 1.5)
    assert route.error is None and len(route.points) == 2


def test_remote_projection_does_not_draw_a_path_from_the_wrong_corridor():
    scene, task, _ = fixture()
    route = GoalRoute(scene, task)
    route.update([0, 0, 0], 0)
    route.scene.project_world = lambda point: [2, 2, 0]
    assert route.update([0, 1, 0], 0.5)
    assert route.points == task["reference_path_world"]
    assert "outside navigable space" in route.error


def test_guidance_stays_ahead_on_dense_paths_and_counts_remaining_detours():
    scene, task, _ = fixture()
    route = GoalRoute(scene, task)
    route.points = [[0, float(y), 0] for y in np.linspace(0, 3, 301)] + [[4, 3, 0], [4, 4, 0]]
    distance, target = route.guidance([0, 1, 0])
    assert distance == pytest.approx(7)
    np.testing.assert_allclose(target, [0, 1.75])
    distance, target = route.guidance([0, 2.8, 0])
    assert distance == pytest.approx(5.2)
    np.testing.assert_allclose(target, [0.55, 3])
