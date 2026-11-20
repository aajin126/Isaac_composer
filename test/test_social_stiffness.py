"""Behavioral checks against official RVO2, independently of recovery heuristics."""

import json

import numpy as np
import pytest

from isaac_composer.crowd.avoidance.clearance import interaction_radii, pair_clearance
from isaac_composer.crowd.behavior.stiffness import SocialStiffness
from isaac_composer.crowd.config import validate_social_config
from isaac_composer.crowd.runtime.recording import StateRecorder
from isaac_composer.crowd.runtime.animation import AnimationCommands
from isaac_composer.crowd.avoidance.rvo2 import RVO2Solver
from isaac_composer.crowd.context.walkable import WalkableContext
from isaac_composer.crowd.behavior.preference import pedestrian_preferences
from test_social_navigation import BoxScene, simulation

ROUTES = [[[-3, -0.1, 0], [3, -0.1, 0]], [[3, 0.1, 0], [-3, 0.1, 0]]]


def passing(mu, overrides=None):
    result = simulation(
        ROUTES,
        {"yielding_enabled": False, "passing_bias_mps": 0, **(overrides or {})},
        mus=[mu, mu],
        seconds=30,
    )
    result[2].solver.close()
    return result


def metrics(result):
    positions, minimum, nav, history = result
    lateral = np.max(abs(history[:, :, 1] - np.array([-0.1, 0.1])), axis=1)
    start = np.flatnonzero(lateral > 0.03)[0]
    assert all(
        np.linalg.norm(q[:2] - np.asarray(r)[-1, :2]) <= 0.31 for q, r in zip(positions, ROUTES)
    )
    assert not any(e["action"] == "yield" for e in nav.events)
    return (
        minimum,
        float(lateral.max()),
        float(np.linalg.norm(history[start, 0, :2] - history[start, 1, :2])),
    )


def test_mu_changes_ordinary_passing_without_yielding_or_right_hand_bias():
    measured = np.asarray([metrics(passing(mu)) for mu in (0, 0.5, 1)])
    # Minimum separation, lateral deviation, and separation at first avoidance
    # all increase in this controlled, unobstructed encounter.
    assert np.all(np.diff(measured, axis=0) > 0)
    assert measured[-1, 0] - measured[0, 0] > 0.25


@pytest.mark.parametrize("mus", [[0, 1], [1, 0]])
def test_mixed_traits_change_individual_avoidance_with_recovery_disabled(mus):
    positions, minimum, nav, history = simulation(
        ROUTES, {"yielding_enabled": False, "passing_bias_mps": 0}, mus=mus, seconds=30
    )
    try:
        deviations = np.max(abs(history[:, :, 1] - np.array([-0.1, 0.1])), axis=0)
        cautious, direct = mus.index(1), mus.index(0)
        assert deviations[cautious] > 0.4 and deviations[direct] < 0.03
        assert minimum >= 0.76
        assert not any(e["action"] == "yield" for e in nav.events)
        assert all(
            np.linalg.norm(q[:2] - np.asarray(r)[-1, :2]) <= 0.31 for q, r in zip(positions, ROUTES)
        )
    finally:
        nav.solver.close()


def test_horizon_changes_anticipation_even_without_a_personal_buffer():
    isolated = {
        "stiffness_buffer_max_m": 0,
        "stiffness_neighbor_distance_min_m": 10,
        "stiffness_neighbor_distance_max_m": 10,
    }
    low = metrics(passing(0, isolated))
    high = metrics(passing(1, isolated))
    assert high[2] > low[2] + 1.0
    assert min(low[0], high[0]) >= 0.6


def test_buffer_changes_clearance_even_with_a_fixed_prediction_horizon():
    fixed = {
        "stiffness_time_horizon_min_s": 2.5,
        "stiffness_time_horizon_max_s": 2.5,
        "stiffness_neighbor_distance_min_m": 10,
        "stiffness_neighbor_distance_max_m": 10,
    }
    low, high = metrics(passing(0, fixed)), metrics(passing(1, fixed))
    assert high[0] > low[0] + 0.25
    assert high[1] > low[1]


def test_stiffness_can_be_disabled_independently_of_deadlock_recovery():
    off = {"social_stiffness_enabled": False}
    low, high = passing(0, off), passing(1, off)
    np.testing.assert_array_equal(low[3], high[3])
    routes = [[[-3, 0, 0], [3, 0, 0]], [[3, 0, 0], [-3, 0, 0]]]
    result = simulation(
        routes, {**off, "yielding_enabled": True}, scene=BoxScene(True), mus=[0.2, 0.8], seconds=45
    )
    try:
        assert any(e["action"] == "yield" and e["blocked_for_s"] >= 0.5 for e in result[2].events)
        assert all(
            np.linalg.norm(q[:2] - np.asarray(r)[-1, :2]) <= 0.31 for q, r in zip(result[0], routes)
        )
    finally:
        result[2].solver.close()


def test_effective_clearance_matches_recovery_for_mixed_traits_and_robot():
    cfg = validate_social_config({})
    people = [{"mu": 0}, {"mu": 0.5}, {"mu": 1}]
    profile = SocialStiffness(people, cfg)
    robots = [(np.zeros(3), np.zeros(2), 0.55)]
    effective = interaction_radii([0.3] * 3, robots, cfg, profile.buffers)
    np.testing.assert_allclose(profile.horizons, [2, 6, 10])
    for i in range(3):
        assert effective[i] + effective[-1] == pytest.approx(
            0.3 + 0.55 + pair_clearance(cfg, robot=True, buffer_i=profile.buffers[i])
        )
        for j in range(i):
            assert effective[i] + effective[j] == pytest.approx(
                0.6 + pair_clearance(cfg, buffer_i=profile.buffers[i], buffer_j=profile.buffers[j])
            )
    # Physical bodies are independent of the interpolated neighbor range.
    assert people == [{"mu": 0}, {"mu": 0.5}, {"mu": 1}]
    np.testing.assert_allclose(profile.neighbor_distances, [2, 6, 10])


def test_recorded_profile_contains_effective_episode_parameters(tmp_path):
    cfg = validate_social_config({"yielding_enabled": False})
    recorder = StateRecorder(
        {
            "state_output_dir": str(tmp_path),
            "social_navigation": cfg,
            "people": [{"id": 7, "mu": 1}],
        }
    )
    recorder.close()
    schema = json.loads((tmp_path / "crowd_state_schema.json").read_text())
    profile = schema["social_stiffness"]
    assert profile["enabled"] is True and schema["deadlock_recovery_enabled"] is False
    assert profile["actors"] == [
        {
            "actor_id": 7,
            "mu": 1,
            "time_horizon_s": 10,
            "personal_buffer_m": 0.15,
            "neighbor_distance_m": 10,
        }
    ]


def test_animation_reference_is_checked_against_the_mu_specific_orca_constraints():
    cfg = validate_social_config({"enabled": True, "yielding_enabled": False})
    scene = BoxScene()
    people = [{"mu": 1, "pose": {"yaw": 0}}]
    solver = RVO2Solver(WalkableContext(scene), cfg, SocialStiffness(people, cfg))
    adapter = AnimationCommands(people, cfg)
    positions, velocity = np.array([[-2.0, 0.2, 0.0]]), np.zeros((1, 2))
    try:
        robots = [(np.zeros(3), np.array([0, 0.3]), 0.55)]
        output, feasible = solver.compute(
            0.02, positions, velocity, np.array([[0.8, 0]]), [0.8], [0.3], robots
        )
        assert feasible[0]
        applied, _, ok = adapter.update(0.02, positions, output, scene, solver)
        assert ok[0] and adapter.walk[0] > 0
        projected = solver.project_velocity(0, positions[0], applied[0])
        assert projected is not None
        np.testing.assert_allclose(applied[0], projected, atol=0.02)
    finally:
        solver.close()


@pytest.mark.parametrize(
    "invalid",
    [
        {"social_stiffness_enabled": "true"},
        {"stiffness_time_horizon_min_s": 0},
        {"stiffness_time_horizon_min_s": 11, "stiffness_time_horizon_max_s": 10},
        {"stiffness_buffer_max_m": -0.1},
        {"stiffness_neighbor_distance_min_m": 0},
        {"stiffness_neighbor_distance_min_m": 11, "stiffness_neighbor_distance_max_m": 10},
    ],
)
def test_stiffness_configuration_rejects_invalid_ranges(invalid):
    with pytest.raises(ValueError):
        validate_social_config(invalid)


@pytest.mark.parametrize("enabled", [True, False])
def test_paper_interpolation_reaches_the_actual_native_agent_parameters(enabled):
    """Audit RVO2 getters for a dense mu sweep, not just the Python profile."""
    cfg = validate_social_config(
        {
            "enabled": True,
            "social_stiffness_enabled": enabled,
            "safety_margin_m": 0,
            "tracking_margin_m": 0,
            "yielding_enabled": False,
            "passing_bias_mps": 0,
            "stationary_robot_bypass_enabled": False,
        }
    )
    mus = np.linspace(0, 1, 9)
    profile = SocialStiffness([{"mu": float(mu)} for mu in mus], cfg)
    solver = RVO2Solver(WalkableContext(BoxScene()), cfg, profile)
    p = np.c_[np.linspace(-4, 4, len(mus)), np.zeros((len(mus), 2))]
    try:
        solver.compute(
            0.02,
            p,
            np.zeros((len(mus), 2)),
            np.zeros((len(mus), 2)),
            [1] * len(mus),
            [0.3] * len(mus),
            [],
        )
        for i, mu in enumerate(mus):
            actual = solver.agent_parameters(i)
            assert actual["time_horizon_s"] == pytest.approx(2 + 8 * mu if enabled else 2.5)
            assert actual["neighbor_distance_m"] == pytest.approx(2 + 8 * mu if enabled else 6)
            assert actual["effective_radius_m"] == pytest.approx(
                0.3 + (0.15 * mu if enabled else 0)
            )
    finally:
        solver.close()


def test_only_neighbor_range_changes_whether_the_native_tree_sees_a_distant_actor():
    cfg = validate_social_config(
        {
            "stiffness_time_horizon_min_s": 5,
            "stiffness_time_horizon_max_s": 5,
            "stiffness_buffer_max_m": 0,
        }
    )
    profile = SocialStiffness([{"mu": 0}, {"mu": 1}], cfg)
    solver = RVO2Solver(WalkableContext(BoxScene()), cfg, profile)
    p = np.array([[-2, 0, 0], [2, 0, 0]], float)
    intent = np.array([[0.8, 0], [-0.8, 0]])
    try:
        output, feasible = solver.compute(
            0.02, p, np.zeros((2, 2)), intent, [0.8, 0.8], [0.3, 0.3], []
        )
        assert feasible.all()
        assert solver.agent_neighbors(0) == []  # 4m is outside mu=0's 2m range.
        assert solver.agent_neighbors(1) == [0]  # It is inside mu=1's 10m range.
        np.testing.assert_allclose(output[0], intent[0], atol=1e-6)
        assert np.linalg.norm(output[1] - intent[1]) > 0.1
    finally:
        solver.close()


def test_paper_reference_preferences_can_disable_every_added_steering_rule():
    cfg = validate_social_config(
        {"yielding_enabled": False, "passing_bias_mps": 0, "stationary_robot_bypass_enabled": False}
    )
    p = np.array([[-2.0, 0, 0]])
    routes = [np.array([p[0], [3.0, 0, 0]])]
    robot = [(np.zeros(3), np.zeros(2), 0.55)]
    pure = pedestrian_preferences(p, routes, [1], [0.3], robot, cfg, [0.15])
    np.testing.assert_allclose(pure, [[1, 0]])


@pytest.mark.parametrize(
    "mu,distance,detected", [(0, 1.99, True), (0, 2.01, False), (1, 9.99, True), (1, 10.01, False)]
)
def test_native_neighbor_detection_respects_the_paper_endpoint_distances(mu, distance, detected):
    cfg = validate_social_config({})
    profile = SocialStiffness([{"mu": mu}, {"mu": mu}], cfg)
    solver = RVO2Solver(WalkableContext(BoxScene()), cfg, profile)
    p = np.array([[-distance / 2, 0, 0], [distance / 2, 0, 0]])
    try:
        solver.compute(0.02, p, np.zeros((2, 2)), np.zeros((2, 2)), [1, 1], [0.3, 0.3], [])
        assert solver.agent_neighbors(0) == ([1] if detected else [])
        assert solver.agent_neighbors(1) == ([0] if detected else [])
    finally:
        solver.close()
