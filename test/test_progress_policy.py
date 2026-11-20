"""Progress queries and policy transitions against the actual official RVO2 core."""

import numpy as np

from test_social_navigation import BoxScene
from isaac_composer.crowd.avoidance.rvo2 import RVO2Solver
from isaac_composer.crowd.behavior.progress import ProgressMonitor
from isaac_composer.crowd.behavior.yielding import YieldCoordinator
from isaac_composer.crowd.context.walkable import WalkableContext
from isaac_composer.crowd.config import validate_social_config


def setup_region(robots=()):
    config = validate_social_config({"enabled": True})
    context = WalkableContext(BoxScene())
    solver = RVO2Solver(context, config)
    positions = np.array([[0.0, 0.0, 0.0]])
    preferred = np.array([[0.8, 0.0]])
    output, feasible = solver.compute(
        0.1, positions, np.zeros((1, 2)), preferred, [0.8], [0.3], robots
    )
    monitor = ProgressMonitor(1, config)
    routes = [np.array([[0.0, 0.0, 0.0], [3.0, 0.0, 0.0]])]
    return solver, monitor, positions, preferred, output, feasible, routes


def test_slow_chosen_velocity_does_not_trigger_yield_when_other_safe_directions_exist():
    solver, monitor, p, pref, _, _, routes = setup_region()
    for _ in range(10):
        monitor.update(0.1, p, pref, [0.8], routes, np.zeros((1, 2)), [True], solver)
    assert monitor.available[0] and not monitor.blocked[0]
    assert monitor.elapsed[0] == 0
    solver.close()


def test_lp_queries_keep_current_observation_and_solution_unchanged():
    solver, _, p, pref, first, feasible, _ = setup_region()
    alternatives = solver.alternatives(0, p[0], pref[0])
    assert max(v[0] for v in alternatives) >= 0.79
    second, ok = solver.compute(0.1, p, np.zeros((1, 2)), pref, [0.8], [0.3], [])
    np.testing.assert_array_equal(first, second)
    np.testing.assert_array_equal(feasible, ok)
    solver.close()


def test_crowded_side_space_has_no_useful_orca_velocity_and_requires_persistence():
    # All eight stationary agents are outside physical overlap, but restrict
    # the central pedestrian's forward and lateral velocities under ORCA.
    robots = [
        (np.r_[np.cos(a), np.sin(a), 0.0], np.zeros(2), 0.3) for a in np.arange(8) * np.pi / 4
    ]
    solver, monitor, p, pref, output, feasible, routes = setup_region(robots)
    for _ in range(4):
        monitor.update(0.1, p, pref, [0.8], routes, output, feasible, solver)
        assert not monitor.available[0] and not monitor.blocked[0]
    monitor.update(0.1, p, pref, [0.8], routes, output, feasible, solver)
    assert monitor.blocked[0]
    # Clearing the surrounding space cancels the evidence immediately.
    monitor.update(0.1, p, pref, [0.8], routes, pref, [True], solver)
    assert monitor.available[0] and monitor.elapsed[0] == 0
    solver.close()


def test_goal_change_and_idle_do_not_carry_stall_evidence_into_next_route():
    solver, monitor, p, pref, output, feasible, routes = setup_region()

    class NoEscape:
        def alternatives(self, *args):
            return []

    for _ in range(6):
        monitor.update(0.1, p, pref, [0.8], routes, np.zeros((1, 2)), [True], NoEscape())
    assert monitor.blocked[0]
    routes[0][-1] = [0, 3, 0]
    monitor.update(0.1, p, pref, [0.8], routes, np.zeros((1, 2)), [True], NoEscape())
    assert not monitor.blocked[0] and monitor.elapsed[0] < 0.2
    monitor.update(0.1, p, np.zeros((1, 2)), [0], routes, output, feasible, solver)
    assert monitor.elapsed[0] == 0 and monitor.available[0]
    solver.close()


def test_policy_ignores_density_and_does_not_yield_on_brief_slowdown():
    scene = BoxScene()
    behavior = YieldCoordinator(
        scene,
        WalkableContext(scene),
        [{"mu": 0}, {"mu": 1}],
        validate_social_config({"enabled": True}),
        43,
    )
    positions = np.array([[-1.0, 0.0, 0.0], [1.0, 0.0, 0.0]])
    preferred = np.array([[0.8, 0], [-0.8, 0]])
    routes = [np.array([positions[0], [3.0, 0, 0]]), np.array([positions[1], [-3.0, 0, 0]])]

    def update(available, context):
        behavior.update(
            0.1, positions, preferred, routes, [0.8, 0.8], [0.3, 0.3], [], context, available
        )

    for _ in range(10):
        update([True, True], [(100, 0.01, 0.1)] * 2)
    assert not any(e["action"] == "yield" for e in behavior.events)
    for _ in range(4):
        update([False, False], [(0, 100, 10)] * 2)
    assert not any(e["action"] == "yield" for e in behavior.events)
    update([True, True], [(0, 100, 10)] * 2)
    for _ in range(4):
        update([False, False], [(0, 100, 10)] * 2)
    assert not any(e["action"] == "yield" for e in behavior.events)
    update([False, False], [(0, 100, 10)] * 2)
    yields = [e for e in behavior.events if e["action"] == "yield"]
    assert len(yields) == 1 and yields[0]["yielder"] == 1
    assert yields[0]["blocked_for_s"] >= 0.5
    for _ in range(10):
        update([False, False], [(100, 0.01, 0.1)] * 2)
    assert len([e for e in behavior.events if e["action"] == "yield"]) == 1


def test_waiting_point_is_selected_by_walkable_path_length_not_straight_distance():
    class DetourScene(BoxScene):
        def shortest_path(self, a, b):
            if b[1] < 0:
                return [a, a + [0, -5, 0], b + [0, -5, 0], b]
            return [a, b]

    scene = DetourScene()
    behavior = YieldCoordinator(
        scene,
        WalkableContext(scene),
        [{"mu": 1}, {"mu": 0}],
        validate_social_config({"enabled": True}),
        43,
    )
    positions = np.array([[-0.5, 0, 0], [-2, 0, 0]])
    route = np.array([positions[1], [3.0, 0, 0]])
    path = behavior._waiting_path(0, positions[1], route, positions, [0.3, 0.3])
    assert path[-1, 1] > 0
    assert np.linalg.norm(np.diff(path[:, :2], axis=0), axis=1).sum() < 3


def test_native_motion_check_resolves_raster_aliasing_without_crossing_blocked_map():
    class NativeScene(BoxScene):
        agent_radius = 0.45
        floor_z = 0

        def __init__(self):
            super().__init__()
            self.raster.mask[80, 80] = False  # Cell center missed a valid thin polygon.
            self.projection_offset = 0.0
            self.allowed = True

        def project_world(self, p, agent_radius=None):
            return np.asarray(p) + [self.projection_offset, 0, 0]

        def is_allowed_world(self, p):
            return self.allowed

    scene = NativeScene()
    context = WalkableContext(scene)
    assert not context.segment_allowed([0, 0], [0.02, 0.02])
    assert context.motion_allowed([0, 0], [0.02, 0.02], 0.3, 0.18)
    scene.projection_offset = 0.2
    assert not context.motion_allowed([0, 0], [0.02, 0.02], 0.3, 0.18)
    scene.projection_offset = 0
    scene.allowed = False
    assert not context.motion_allowed([0, 0], [0.02, 0.02], 0.3, 0.18)


def test_animation_preview_projects_drifted_start_and_preserves_native_corners():
    from isaac_composer.crowd.runtime.animation import locomotion_path

    class Scene:
        def project_world(self, p):
            result = np.asarray(p).copy()
            result[1] = max(result[1], 0.0)
            return result

        def shortest_path(self, a, b):
            assert a[1] >= 0 and b[1] >= 0
            return [a, np.array([a[0], 0.4, 0]), b]

    position = np.array([0.0, -0.02, 0.0])
    path = locomotion_path(Scene(), position, np.array([0.8, 0.0]))
    np.testing.assert_array_equal(path[0], position)
    assert np.any(np.all(path == [0.0, 0.4, 0.0], axis=1))
    assert path[-1, 0] >= 0.69


def test_animation_preview_stops_when_navmesh_cannot_find_any_target():
    from isaac_composer.crowd.runtime.animation import locomotion_path

    class Scene:
        def project_world(self, p):
            raise RuntimeError("Blocked or unknown")

    position = np.array([0.0, 0.0, 0.0])
    np.testing.assert_array_equal(
        locomotion_path(Scene(), position, np.array([0.8, 0.0])), [position]
    )
