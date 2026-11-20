import math
import numpy as np
import pytest

from isaac_composer.crowd.state import MeasuredVelocities, body_velocity_to_world
from isaac_composer.crowd.context.walkable import WalkableContext
from isaac_composer.navmesh import RasterizedNavMesh, SceneBounds
from isaac_composer.crowd.avoidance.rvo2 import RVO2Solver
from isaac_composer.crowd.navigation.controller import SocialNavigator
from isaac_composer.crowd.behavior.yielding import yielding_probability
from isaac_composer.crowd.config import validate_social_config


class BoxScene:
    agent_radius = 0.3

    def __init__(self, corridor=False):
        mask = np.ones((160, 160), bool)
        if corridor:
            mask[:] = False
            mask[79:82, :] = True
            mask[60:100, 90:130] = True
        self.raster = RasterizedNavMesh(
            mask, np.ones(mask.shape), SceneBounds(-8, 8, -8, 8, 0, 0), 0.1
        )

    def rasterize(self, margin_m=0):
        return self.raster

    def shortest_path(self, a, b):
        context = WalkableContext(self, self.raster)
        if context.segment_allowed(a, b):
            return [a, b]
        # A corridor with a passing bay: retreat along its center before turning.
        via = np.array([2.5, 0, 0])
        if context.segment_allowed(a, via) and context.segment_allowed(via, b):
            return [a, via, b]
        raise RuntimeError("No test path")


def simulation(
    routes, config=None, scene=None, robot_fn=None, mus=None, seconds=30, nominal_speeds=None
):
    scene = scene or BoxScene()
    context = WalkableContext(scene)
    people = [{"mu": m} for m in (mus or [0.5] * len(routes))]
    nav = SocialNavigator(
        scene,
        context,
        people,
        {"enabled": True, "tracking_margin_m": 0.0, "obstacle_clearance_m": 0.0, **(config or {})},
        43,
    )
    p = np.asarray([r[0] for r in routes], float)
    v = np.zeros((len(p), 2))
    minimum = math.inf
    history = []
    for tick in range(int(seconds / 0.04)):
        speeds = [
            speed if np.linalg.norm(q[:2] - np.asarray(r)[-1, :2]) > 0.3 else 0
            for q, r, speed in zip(p, routes, nominal_speeds or [0.8] * len(p))
        ]
        robots = [] if robot_fn is None else robot_fn(tick * 0.04)
        v, _ = nav.step(0.04, p, v, routes, speeds, [0.3] * len(p), robots)
        p[:, :2] += v * 0.04
        assert all(context.allows(q) for q in p)
        for i in range(len(p)):
            for j in range(i):
                minimum = min(minimum, np.linalg.norm(p[i, :2] - p[j, :2]))
            for q, _, radius in robots:
                minimum = min(minimum, np.linalg.norm(p[i, :2] - np.asarray(q)[:2]))
        history.append(p.copy())
    return p, minimum, nav, np.array(history)


def test_stage1_actual_velocity_and_body_world_rotation():
    measured = MeasuredVelocities()
    np.testing.assert_equal(measured.update(0, [[0, 0, 0]]), [[0, 0]])
    np.testing.assert_allclose(measured.update(0.1, [[0.2, -0.1, 0]]), [[2, -1]])
    np.testing.assert_allclose(body_velocity_to_world([1, 0], math.pi / 2), [0, 1], atol=1e-10)
    np.testing.assert_equal(measured.update(0, [[9, 9, 0]]), [[0, 0]])


def test_stage2_official_rvo2_follows_preference_without_neighbors():
    solver = RVO2Solver(WalkableContext(BoxScene()), validate_social_config({}))
    v, ok = solver.compute(
        0.02, np.array([[0, 0, 0]]), np.array([[0, 0]]), np.array([[0.7, 0.2]]), [1], [0.3], []
    )
    assert ok.all()
    np.testing.assert_allclose(v, [[0.7, 0.2]], atol=1e-6)
    solver.close()


def test_close_opposing_lanes_keep_progress_with_default_pedestrian_clearance():
    """Two bodies fit with a few cm gap; padded radii must not block them."""
    cfg = validate_social_config({"enabled": True})
    scene = BoxScene()
    scene.raster.mask[:] = False
    scene.raster.mask[76:84, :] = True
    context = WalkableContext(scene)
    solver = RVO2Solver(context, cfg)
    p = np.array([[-1.5, -0.34, 0.0], [1.5, 0.34, 0.0]])
    velocity = np.array([[0.8, 0.0], [-0.8, 0.0]])
    minimum = float("inf")
    for _ in range(150):
        result, feasible = solver.compute(0.02, p, velocity, velocity, [0.8, 0.8], [0.3, 0.3], [])
        assert feasible.all()
        np.testing.assert_allclose(result, velocity, atol=1e-6)
        p[:, :2] += result * 0.02
        minimum = min(minimum, float(np.linalg.norm(p[0, :2] - p[1, :2])))
    assert 0.66 <= minimum < 0.69
    assert p[0, 0] > 0.8 and p[1, 0] < -0.8
    solver.close()


def test_reduced_pedestrian_clearance_preserves_teleop_robot_constraint():
    context = WalkableContext(BoxScene())
    p = np.array([[-1.5, 0.25, 0.0]])
    v = np.array([[0.5, 0.0]])
    preferred = np.array([[0.8, 0.0]])
    robots = [(np.zeros(3), np.array([0.0, 0.4]), 0.55)]
    outputs = []
    for values in [{}, {"safety_margin_m": 0.18, "tracking_margin_m": 0.15}]:
        solver = RVO2Solver(context, validate_social_config(values))
        outputs.append(solver.compute(0.02, p, v, preferred, [0.8], [0.3], robots))
        solver.close()
    np.testing.assert_allclose(outputs[0][0], outputs[1][0], atol=1e-6)
    np.testing.assert_array_equal(outputs[0][1], outputs[1][1])


@pytest.mark.parametrize(
    "routes",
    [
        [[[-3, 0, 0], [3, 0, 0]], [[3, 0, 0], [-3, 0, 0]]],
        [[[-3, 0, 0], [3, 0, 0]], [[0, -3, 0], [0, 3, 0]]],
        [[[-3, 0, 0], [3, 0, 0]], [[-1, 0, 0], [4, 0, 0]]],
    ],
)
def test_stage3_low_density_mutual_orca_reaches_goals(routes):
    p, minimum, nav, _ = simulation(routes, {"yielding_enabled": False})
    assert minimum >= 0.6 - 1e-5
    assert all(np.linalg.norm(q[:2] - np.asarray(r)[-1, :2]) <= 0.31 for q, r in zip(p, routes)), p
    assert nav.infeasible_count == 0
    assert not any(e["action"] == "yield" for e in nav.events)


def test_stage4_robot_constraint_takes_full_responsibility():
    routes = [[[-3, 0, 0], [3, 0, 0]]]
    p, minimum, nav, _ = simulation(
        routes, {"yielding_enabled": False}, robot_fn=lambda t: [([0, 0, 0], [0, 0], 0.55)]
    )
    assert minimum >= 0.85 - 1e-5
    assert np.linalg.norm(p[0, :2] - [3, 0]) <= 0.31, p


def test_stage5_density_uses_local_walkable_area_and_includes_self():
    wide = WalkableContext(BoxScene())
    narrow = WalkableContext(BoxScene(True))
    positions = np.array([[0, 0, 0], [1, 0, 0]])
    a = wide.local(positions[0], positions, 3)
    b = narrow.local(positions[0], positions, 3)
    assert a[1] > b[1] and a[0] < b[0] and b[2] < a[2]
    assert math.isclose(a[0], 2 / a[1])


def test_dense_context_does_not_force_yield_while_orca_can_pass():
    routes = [[[-3, 0, 0], [3, 0, 0]], [[0, -3, 0], [0, 3, 0]]]
    p, minimum, nav, _ = simulation(routes, mus=[0, 1])
    assert minimum >= 0.6 - 1e-5
    yields = [e for e in nav.events if e["action"] == "yield"]
    assert not yields
    assert all(np.linalg.norm(q[:2] - np.asarray(r)[-1, :2]) <= 0.31 for q, r in zip(p, routes)), p


def test_stage7_probability_ensemble_and_seed_reproducibility():
    import random

    rng = random.Random(43)
    count = sum(rng.random() < yielding_probability(0.8, 0.2) for _ in range(5000))
    assert 0.78 < count / 5000 < 0.82
    assert yielding_probability(0, 0) == 0.5
    route = [[[-3, 0, 0], [3, 0, 0]], [[3, 0, 0], [-3, 0, 0]]]
    first = simulation(route, scene=BoxScene(True), mus=[0.2, 0.8], seconds=45)
    second = simulation(route, scene=BoxScene(True), mus=[0.2, 0.8], seconds=45)
    assert first[2].events == second[2].events
    np.testing.assert_equal(first[3], second[3])


@pytest.mark.parametrize(
    "value",
    [
        {"progress_min_fraction": 1},
        {"mu_range": [0, 2]},
        {"time_horizon_s": 0},
        {"enabled": "true"},
        {"unknown": 1},
    ],
)
def test_social_config_rejects_invalid_settings(value):
    with pytest.raises(ValueError):
        validate_social_config(value)


def test_moving_teleop_robot_is_not_expected_to_reciprocate():
    routes = [[[-3, 0, 0], [3, 0, 0]]]

    def robot(t):
        return [([0, min(3, -3 + 0.6 * t), 0], [0, 0.6 if t < 10 else 0], 0.55)]

    p, minimum, nav, _ = simulation(routes, {"yielding_enabled": False}, robot_fn=robot)
    assert minimum >= 0.85
    assert np.linalg.norm(p[0, :2] - [3, 0]) <= 0.31
    assert nav.infeasible_count == 0


@pytest.mark.parametrize("mus", [[0, 1], [1, 0]])
def test_narrow_passing_bay_latches_role_until_winner_passes_and_both_resume(mus):
    routes = [[[-3, 0, 0], [3, 0, 0]], [[3, 0, 0], [-3, 0, 0]]]
    p, minimum, nav, _ = simulation(routes, scene=BoxScene(True), mus=mus, seconds=45)
    yields = [event for event in nav.events if event["action"] == "yield"]
    assert len(yields) == 1 and yields[0]["yielder"] == 1
    assert yields[0]["blocked_for_s"] >= 0.5
    assert any(event["action"] == "resume" for event in nav.events)
    assert minimum >= 0.6
    assert all(np.linalg.norm(q[:2] - np.asarray(r)[-1, :2]) <= 0.31 for q, r in zip(p, routes))


def test_dense_same_direction_flow_does_not_choose_a_yielder():
    routes = [[[-3, 0, 0], [3, 0, 0]], [[-1, 0, 0], [5, 0, 0]]]
    _, minimum, nav, _ = simulation(routes)
    assert minimum >= 0.6
    assert not any(event["action"] == "yield" for event in nav.events)


def test_faster_pedestrian_overtakes_without_contact():
    routes = [[[-3, 0, 0], [3, 0, 0]], [[-1, 0, 0], [5, 0, 0]]]
    p, minimum, nav, _ = simulation(
        routes, {"yielding_enabled": False}, nominal_speeds=[1.2, 0.6], seconds=40
    )
    assert minimum >= 0.6
    assert all(np.linalg.norm(q[:2] - np.asarray(r)[-1, :2]) <= 0.31 for q, r in zip(p, routes))


def test_probability_uses_actual_encounter_selection():
    from isaac_composer.crowd.behavior.yielding import YieldCoordinator

    scene = BoxScene()
    context = WalkableContext(scene)
    config = validate_social_config({"enabled": True})
    behavior = YieldCoordinator(scene, context, [{"mu": 0.8}, {"mu": 0.2}], config, 43)
    positions = np.array([[-3.0, 0.0, 0.0], [0.0, -3.0, 0.0]])
    routes = [np.array([positions[0], [3, 0, 0]]), np.array([positions[1], [0, 3, 0]])]
    count = 0
    for _ in range(5000):
        encounter = behavior._start_encounter((0, 1), routes[1], routes, [(1, 2, 10), (1, 2, 10)])
        encounter["blocked_time"] = 0.5
        behavior._select_yielder(
            encounter,
            (0, 1),
            positions[1],
            np.array([0, 0.8]),
            routes[1],
            positions,
            routes,
            [0.3, 0.3],
        )
        count += encounter["yielder"] == 0
    assert 0.78 < count / 5000 < 0.82
