"""Locomotion conditioning against the real RVO2 velocity constraints."""

import math
import numpy as np

from isaac_composer.crowd.config import validate_social_config
from isaac_composer.crowd.runtime.animation import AnimationCommands
from isaac_composer.crowd.avoidance.rvo2 import RVO2Solver
from isaac_composer.crowd.context.walkable import WalkableContext
from test_social_navigation import BoxScene


def test_fast_walk_reaches_requested_speed_and_advances_animation_instead_of_clipping():
    scene = BoxScene()
    cfg = validate_social_config({})
    adapter = AnimationCommands([{"pose": {"yaw": 0}}], cfg)
    solver = RVO2Solver(WalkableContext(scene), cfg)
    p, v = np.zeros((1, 3)), np.zeros((1, 2))
    request = np.array([[1.6, 0.0]])
    for _ in range(80):
        solver.compute(0.02, p, v, request, [1.6], [0.3], [])
        v, _, ok = adapter.update(0.02, p, request, scene, solver)
        assert all(ok)
        p[:, :2] += v * 0.02
    np.testing.assert_allclose(v[0], [1.6, 0], atol=1e-5)
    assert adapter.walk[0] == 1.0
    np.testing.assert_allclose(adapter.playback_rate, [1.6])
    # Deceleration and the original low-speed Walk blend still apply on stopping.
    for _ in range(40):
        solver.compute(0.02, p, v, np.zeros((1, 2)), [1.6], [0.3], [])
        v, _, _ = adapter.update(0.02, p, np.zeros((1, 2)), scene, solver)
        p[:, :2] += v * 0.02
    assert adapter.walk[0] == 0.0
    assert adapter.playback_rate[0] == 1.0
    solver.close()


def test_straight_walk_accelerates_gradually_and_reuses_world_space_targets():
    scene = BoxScene()
    cfg = validate_social_config({})
    adapter = AnimationCommands([{"pose": {"yaw": 0}}], cfg)
    solver = RVO2Solver(WalkableContext(scene), cfg)
    p, v = np.zeros((1, 3)), np.zeros((1, 2))
    previous_speed = 0.0
    for _ in range(200):
        request = np.array([[0.8, 0.0]])
        solver.compute(0.02, p, v, request, [0.8], [0.3], [])
        v, _, feasible = adapter.update(0.02, p, request, scene, solver)
        assert all(feasible)
        speed = float(np.linalg.norm(v[0]))
        assert speed - previous_speed <= cfg["animation_max_acceleration_mps2"] * 0.02 + 1e-6
        assert 0 <= adapter.walk[0] <= 1
        previous_speed = speed
        p[:, :2] += v * 0.02
    assert p[0, 0] > 2.8
    assert adapter.path_updates < 25  # Previously 200 replacements.
    solver.close()


def test_reversal_brakes_and_turns_without_a_one_tick_180_degree_target_flip():
    scene = BoxScene()
    cfg = validate_social_config({})
    adapter = AnimationCommands([{"pose": {"yaw": 0}}], cfg)
    solver = RVO2Solver(WalkableContext(scene), cfg)
    p, v = np.zeros((1, 3)), np.zeros((1, 2))
    headings = []
    ticks = []
    for tick in range(180):
        request = np.array([[0.8 if tick < 60 else -0.8, 0.0]])
        solver.compute(0.02, p, v, request, [0.8], [0.3], [])
        v, _, _ = adapter.update(0.02, p, request, scene, solver)
        if np.linalg.norm(v[0]) > 0.08:
            headings.append(math.atan2(v[0, 1], v[0, 0]))
            ticks.append(tick)
        p[:, :2] += v * 0.02
    changes = np.abs(np.arctan2(np.sin(np.diff(headings)), np.cos(np.diff(headings))))
    # Rotation may continue while stopped; do not count a stopped interval as
    # a single 20ms heading jump. While walking, retargets are small steps.
    adjacent = np.diff(ticks) == 1
    assert np.max(changes[adjacent]) < math.radians(30)
    assert np.max(changes) < math.radians(90)
    assert v[0, 0] < -0.7
    solver.close()


def test_conditioned_velocity_is_rechecked_against_robot_constraints():
    scene = BoxScene()
    cfg = validate_social_config({})
    adapter = AnimationCommands([{}], cfg)
    solver = RVO2Solver(WalkableContext(scene), cfg)
    p = np.array([[-1.4, 0.0, 0.0]])
    request = np.array([[0.8, 0.0]])
    solver.compute(
        0.02, p, np.zeros((1, 2)), request, [0.8], [0.3], [(np.zeros(3), np.zeros(2), 0.55)]
    )
    v, _, ok = adapter.update(0.02, p, request, scene, solver)
    if ok[0]:
        projected = solver.project_velocity(0, p[0], v[0])
        np.testing.assert_allclose(v[0], projected, atol=0.02)
    solver.close()


def test_unreachable_animation_path_stops_instead_of_executing_wrong_direction():
    class Blocked(BoxScene):
        def shortest_path(self, a, b):
            raise RuntimeError("No path")

    class Solver:
        def project_velocity(self, i, p, v):
            return v

    adapter = AnimationCommands([{}], validate_social_config({}))
    v, _, ok = adapter.update(0.1, np.zeros((1, 3)), np.array([[0.8, 0.0]]), Blocked(), Solver())
    np.testing.assert_array_equal(v, np.zeros((1, 2)))
    assert not ok[0] and adapter.walk[0] == 0


def test_preview_endpoint_does_not_use_the_route_goal_arrival_radius():
    # Native projection can shorten a preview to below 30cm. This is not the
    # pedestrian's goal and must not manufacture a stop command.
    tangent = AnimationCommands._path_velocity(np.zeros(3), np.array([[0, 0, 0], [0.2, 0, 0]]), 0.6)
    np.testing.assert_allclose(tangent, [0.6, 0])


def test_unsafe_long_preview_tries_a_safe_shorter_path():
    class DetourScene(BoxScene):
        def shortest_path(self, a, b):
            if np.linalg.norm(np.asarray(b) - a) > 0.6:
                return [a, np.asarray(a) + [0, 0.5, 0], b]
            return [a, b]

    class HorizontalSolver:
        def project_velocity(self, i, p, v):
            return np.array([v[0], 0.0])

    adapter = AnimationCommands([{}], validate_social_config({}))
    v, paths, ok = adapter.update(
        0.1, np.zeros((1, 3)), np.array([[0.8, 0]]), DetourScene(), HorizontalSolver()
    )
    assert ok[0] and v[0, 0] > 0.1
    assert np.linalg.norm(paths[0][-1]) <= 0.6
    assert adapter.recovered_paths == 1 and adapter.rejected == 0


def test_short_stop_keeps_action_but_clears_walk_and_forward_path_immediately():
    class Solver:
        blocked = False

        def project_velocity(self, i, p, v):
            return None if self.blocked else v

    cfg = validate_social_config({})
    adapter = AnimationCommands([{}], cfg)
    solver = Solver()
    scene = BoxScene()
    pos = np.zeros((1, 3))
    request = np.array([[0.8, 0]])
    for _ in range(10):
        adapter.update(0.02, pos, request, scene, solver)
    assert adapter.action_walking[0]
    before = adapter.action_switches
    solver.blocked = True
    for tick in range(4):
        v, paths, ok = adapter.update(0.02, pos, request, scene, solver)
        np.testing.assert_array_equal(v, [[0, 0]])
        assert not ok[0] and adapter.walk[0] == 0
        assert adapter.action_walking[0] and not adapter.action_changed[0]
        if tick == 0:
            assert adapter.path_changed[0]
            np.testing.assert_array_equal(paths[0], pos)
    solver.blocked = False
    for _ in range(3):
        adapter.update(0.02, pos, request, scene, solver)
    assert adapter.walking[0] and adapter.action_walking[0]
    assert adapter.action_switches == before


def test_persistent_stop_transitions_to_idle_after_delay():
    class Solver:
        blocked = False

        def project_velocity(self, i, p, v):
            return None if self.blocked else v

    cfg = validate_social_config({})
    adapter = AnimationCommands([{}], cfg)
    solver = Solver()
    for _ in range(10):
        adapter.update(0.02, np.zeros((1, 3)), np.array([[0.8, 0]]), BoxScene(), solver)
    solver.blocked = True
    for _ in range(9):
        adapter.update(0.02, np.zeros((1, 3)), np.array([[0.8, 0]]), BoxScene(), solver)
        assert adapter.action_walking[0]
    adapter.update(0.02, np.zeros((1, 3)), np.array([[0.8, 0]]), BoxScene(), solver)
    assert not adapter.action_walking[0] and adapter.action_changed[0]
