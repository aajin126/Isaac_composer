"""Audit directional shares in the real RVO2 LP and integrated navigation."""

import json
import math
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np
import pytest

from isaac_composer.crowd.behavior.responsibility import (
    DirectionalResponsibility,
    character_body_yaws,
)
from isaac_composer.crowd.behavior.stiffness import SocialStiffness
from isaac_composer.crowd.config import validate_social_config
from isaac_composer.crowd.avoidance.rvo2 import RVO2Solver
from isaac_composer.crowd.context.walkable import WalkableContext
from isaac_composer.crowd.navigation.controller import SocialNavigator
from isaac_composer.crowd.runtime.isaac import CrowdRuntime
from isaac_composer.crowd.runtime.recording import StateRecorder
from test_social_navigation import BoxScene


def config(**settings):
    return validate_social_config(
        {
            "enabled": True,
            "directional_responsibility_enabled": True,
            "yielding_enabled": False,
            "passing_bias_mps": 0,
            "stationary_robot_bypass_enabled": False,
            **settings,
        }
    )


def policy(headings=(0, 0), **settings):
    return DirectionalResponsibility([{"pose": {"yaw": h}} for h in headings], config(**settings))


@pytest.mark.parametrize(
    "headings,share",
    [
        ([0, math.pi], 0.5),  # Both see the other in front.
        ([0, 0], 1 / 1.1),  # First sees front; second sees rear.
        ([math.pi, math.pi], 0.1 / 1.1),
        ([math.pi, 0], 0.5),  # Both see the other behind.
        ([math.pi / 2, math.pi / 2], 0.5),
        ([0, math.pi / 2], 1 / 1.55),  # Front versus side attention.
    ],
)
def test_front_side_rear_and_pair_complementarity(headings, share):
    model = policy()
    actual = model.update(0.02, [[-1, 0, 0], [1, 0, 0]], np.zeros((2, 2)), [0.3, 0.3], headings)
    assert actual[0, 1] == pytest.approx(share)
    np.testing.assert_allclose(actual + actual.T, 1, atol=1e-12)


def test_angular_response_is_continuous_and_rotation_invariant():
    angles = np.linspace(0, math.pi, 181)
    shares = []
    for angle in angles:
        p = [[-1, 0], [1, 0]]
        model = policy()
        shares.append(model.update(0.02, p, np.zeros((2, 2)), [0.3, 0.3], [angle, math.pi])[0, 1])
        rotate = 0.723
        rot = np.array(
            [[math.cos(rotate), -math.sin(rotate)], [math.sin(rotate), math.cos(rotate)]]
        )
        other = policy().update(
            0.02,
            np.asarray(p) @ rot.T,
            np.zeros((2, 2)),
            [0.3, 0.3],
            [angle + rotate, math.pi + rotate],
        )
        assert other[0, 1] == pytest.approx(shares[-1])
    assert np.all(np.diff(shares) <= 0)
    assert max(abs(np.diff(shares))) < 0.01  # No semicircle cutoff at 90 degrees.


def test_physical_contact_restores_equal_shares_without_filter_delay():
    model = policy()
    v = np.zeros((2, 2))
    far = model.update(0.02, [[0, 0], [2, 0]], v, [0.3, 0.3], [0, 0])
    assert far[0, 1] > 0.9
    middle = model.update(0.02, [[0, 0], [0.9, 0]], v, [0.3, 0.3], [0, 0])
    assert 0.5 < middle[0, 1] < far[0, 1]
    for distance in (0.75, 0.6, 0.3, 0):
        close = model.update(0.02, [[0, 0], [distance, 0]], v, [0.3, 0.3], [0, 0])
        np.testing.assert_allclose(close, 0.5, atol=1e-12)
    # Body radii, not a hardcoded 0.3m radius or mu's safety buffer.
    close = model.update(0.02, [[0, 0], [1.15, 0]], v, [0.5, 0.5], [0, 0])
    np.testing.assert_allclose(close, 0.5, atol=1e-12)


def test_observed_heading_overrides_motion_and_survives_idle():
    model = policy()
    pos = [[-1, 0], [1, 0]]
    opposing_motion = [[0, 1], [0, -1]]
    actual = model.update(0.02, pos, opposing_motion, [0.3, 0.3], [0, 0])
    assert actual[0, 1] == pytest.approx(1 / 1.1)
    for _ in range(5):
        actual = model.update(0.02, pos, np.zeros((2, 2)), [0.3, 0.3])
    assert actual[0, 1] == pytest.approx(1 / 1.1)
    np.testing.assert_allclose(model.heading, [0, 0])


def test_filters_wrap_body_heading_and_share_changes_without_losing_complementarity():
    model = policy()
    pos, v = [[-1, 0], [1, 0]], np.zeros((2, 2))
    model.update(0.02, pos, v, [0.3, 0.3], [math.radians(179), 0])
    old_heading = model.heading.copy()
    model.update(0.02, pos, v, [0.3, 0.3], [math.radians(-179), math.pi])
    diff = model.heading - old_heading
    diff = np.arctan2(np.sin(diff), np.cos(diff))
    assert 0 < diff[0] < math.radians(2)
    old = model.shares.copy()
    for _ in range(100):
        new = model.update(0.02, pos, v, [0.3, 0.3], [0, math.pi])
        assert np.max(abs(new - old)) < 0.05
        np.testing.assert_allclose(new + new.T, 1, atol=1e-12)
        old = new.copy()


def test_character_world_quaternion_heading_is_used_even_when_stationary():
    headings = [0, math.pi / 2, -math.pi, -0.4]
    # People assets face local -Y: root +90 degrees means world +X body front.
    quats = [
        [0, 0, 2 * math.sin((h + math.pi / 2) / 2), 2 * math.cos((h + math.pi / 2) / 2)]
        for h in headings
    ]
    runtime = CrowdRuntime.__new__(CrowdRuntime)
    runtime.people = [SimpleNamespace(state=SimpleNamespace(orientation=q)) for q in quats]
    error = runtime.body_headings() - headings
    np.testing.assert_allclose(np.arctan2(np.sin(error), np.cos(error)), 0, atol=1e-12)
    assert character_body_yaws([]).shape == (0,)
    with pytest.raises(ValueError):
        character_body_yaws([[0, 0, 0, 0]])


def test_native_front_walker_changes_less_and_rear_walker_more_than_equal_sharing():
    cfg = config()
    p = np.array([[0, 0.1, 0], [-1.8, -0.1, 0]])
    v = np.array([[0.6, 0], [1.2, 0]])
    model = policy()
    shares = model.update(0.02, p, v, [0.3, 0.3], [0, 0])
    solver = RVO2Solver(WalkableContext(BoxScene()), cfg)
    try:
        original, ok = solver.compute(0.02, p, v, v, [0.6, 1.2], [0.3, 0.3], [])
        assert ok.all() and solver.agent_responsibility(0, 1) == 0.5
        weighted, ok = solver.compute(
            0.02, p, v, v, [0.6, 1.2], [0.3, 0.3], [], responsibilities=shares
        )
        assert ok.all()
        assert solver.agent_responsibility(0, 1) == pytest.approx(shares[0, 1])
        assert solver.agent_responsibility(1, 0) == pytest.approx(shares[1, 0])
        changes = np.linalg.norm(weighted - v, axis=1)
        baseline = np.linalg.norm(original - v, axis=1)
        assert changes[0] < baseline[0] * 0.3
        assert changes[1] > baseline[1] * 1.5
        # Disabling clears native per-agent overrides as well.
        disabled, ok = solver.compute(0.02, p, v, v, [0.6, 1.2], [0.3, 0.3], [])
        np.testing.assert_allclose(disabled, original, atol=1e-6)
        assert solver.agent_responsibility(0, 1) == 0.5
    finally:
        solver.close()


@pytest.mark.parametrize("mus", [[0, 1], [1, 0]])
def test_one_sided_mu_detection_never_delegates_to_unaware_pedestrian(mus):
    cfg = config()
    stiffness = SocialStiffness([{"mu": m} for m in mus], cfg)
    solver = RVO2Solver(WalkableContext(BoxScene()), cfg, stiffness)
    p, v = np.array([[-2, 0, 0], [2, 0, 0]], float), np.zeros((2, 2))
    shares = policy().update(0.02, p, v, [0.3, 0.3], [0, 0])
    try:
        solver.compute(0.02, p, v, v, [1, 1], [0.3, 0.3], [], responsibilities=shares)
        unaware, aware = mus.index(0), mus.index(1)
        assert solver.agent_neighbors(unaware) == []
        assert solver.agent_responsibility(unaware, aware) is None
        assert solver.agent_responsibility(aware, unaware) == 1
        # Use CURRENT neighbor lists, including when an actor enters/leaves range.
        p[1, 0] = -0.5
        solver.compute(0.02, p, v, v, [1, 1], [0.3, 0.3], [], responsibilities=shares)
        assert solver.agent_responsibility(aware, unaware) == pytest.approx(shares[aware, unaware])
        p[1, 0] = 2
        solver.compute(0.02, p, v, v, [1, 1], [0.3, 0.3], [], responsibilities=shares)
        assert solver.agent_responsibility(aware, unaware) == 1
        for i, mu in enumerate(mus):
            native = solver.agent_parameters(i)
            assert native["time_horizon_s"] == pytest.approx(2 + 8 * mu)
            assert native["neighbor_distance_m"] == pytest.approx(2 + 8 * mu)
            assert native["effective_radius_m"] == pytest.approx(0.33 + 0.15 * mu)
    finally:
        solver.close()


def test_teleop_robot_still_gets_full_pedestrian_responsibility_in_every_direction():
    cfg = config()
    solver = RVO2Solver(WalkableContext(BoxScene()), cfg)
    p, v = np.array([[0, 0, 0]]), np.array([[0.6, 0]])
    robot_v = np.array([1.0, 0])
    try:
        for x in (-1.5, 1.5):
            robots = [(np.array([x, 0.1, 0]), robot_v, 0.55)]
            original, _ = solver.compute(0.02, p, v, v, [0.6], [0.3], robots)
            weighted, _ = solver.compute(
                0.02, p, v, v, [0.6], [0.3], robots, responsibilities=np.array([[0.5]])
            )
            np.testing.assert_allclose(weighted, original, atol=1e-6)
            assert solver.agent_responsibility(0, 1) == 1
            assert solver.agent_responsibility(1, 0) == 0
    finally:
        solver.close()


@pytest.mark.parametrize("case", ["head_on", "crossing", "rear_approach"])
@pytest.mark.parametrize("mus", [(0, 0), (0.5, 0.5), (1, 1), (0, 1), (1, 0)])
def test_integrated_weighted_orca_reaches_goals_without_body_overlap(case, mus):
    routes = np.array([[[-3, -0.1, 0], [3, -0.1, 0]], [[3, 0.1, 0], [-3, 0.1, 0]]], float)
    headings, speeds = np.array([0, math.pi]), np.array([0.8, 0.8])
    if case == "crossing":
        routes[1] = [[0.1, -3, 0], [0.1, 3, 0]]
        headings[1] = math.pi / 2
    elif case == "rear_approach":
        # Destinations must also fit two bodies; otherwise the first arrival
        # physically occupies the follower's goal and arrival is infeasible.
        routes[1] = [[-5, 0.1, 0], [4.5, 0.1, 0]]
        headings[1] = 0
        speeds[:] = [0.65, 1.1]
    scene = BoxScene()
    people = [{"mu": mu, "pose": {"yaw": h}} for mu, h in zip(mus, headings)]
    nav = SocialNavigator(scene, WalkableContext(scene), people, config())
    p, v = routes[:, 0].copy(), np.zeros((2, 2))
    minimum = math.inf
    try:
        for _ in range(750):
            moving = np.linalg.norm(v, axis=1) > 0.05
            headings[moving] = np.arctan2(v[moving, 1], v[moving, 0])
            active = speeds * (np.linalg.norm(p[:, :2] - routes[:, -1, :2], axis=1) > 0.3)
            v, pref = nav.step(0.04, p, v, routes, active, [0.3, 0.3], headings=headings)
            p[:, :2] += v * 0.04
            minimum = min(minimum, np.linalg.norm(p[0, :2] - p[1, :2]))
        assert minimum >= 0.6
        assert np.all(np.linalg.norm(p[:, :2] - routes[:, -1, :2], axis=1) <= 0.31), p
        assert nav.infeasible_count == 0
        assert all(event["action"] in ("mutual_avoidance", "resume") for event in nav.events)
    finally:
        nav.solver.close()


def test_same_observation_reuses_pair_shares_when_yielding_replans_preference():
    scene = BoxScene()
    nav = SocialNavigator(scene, WalkableContext(scene), [{}, {}], config())
    p, v = np.array([[-1, 0, 0], [1, 0, 0]]), np.zeros((2, 2))
    routes = np.array([[p[0], [3, 0, 0]], [p[1], [-3, 0, 0]]])
    try:
        with patch.object(nav.responsibility, "update", wraps=nav.responsibility.update) as update:
            with patch.object(nav.recovery, "update", return_value=np.zeros((2, 2))):
                with patch.object(nav.solver, "compute", wraps=nav.solver.compute) as solve:
                    nav.step(0.02, p, v, routes, [0.8, 0.8], [0.3, 0.3], headings=[0, math.pi])
                    assert update.call_count == 1 and solve.call_count == 2
                    np.testing.assert_array_equal(
                        solve.call_args_list[0].kwargs["responsibilities"],
                        solve.call_args_list[1].kwargs["responsibilities"],
                    )
    finally:
        nav.solver.close()


def test_recording_schema_captures_directional_policy_and_independent_mu_settings(tmp_path):
    recorder = StateRecorder(
        {"state_output_dir": str(tmp_path), "people": [{"mu": 0.8}], "social_navigation": config()}
    )
    recorder.close()
    saved = json.loads((tmp_path / "crowd_state_schema.json").read_text())
    assert saved["directional_responsibility"]["enabled"] is True
    assert saved["social_stiffness"]["enabled"] is True
    assert saved["deadlock_recovery_enabled"] is False
    assert saved["directional_responsibility"]["robot_avoidance_responsibility"] == 1
    assert json.dumps(saved, ensure_ascii=False).isascii()


@pytest.mark.parametrize(
    "invalid",
    [
        {"directional_responsibility_enabled": "true"},
        {"directional_rear_weight": 0},
        {"directional_rear_weight": -1},
        {"directional_rear_weight": 1.01},
        {"directional_smoothing_s": 0},
        {"directional_smoothing_s": float("nan")},
        {"directional_emergency_gap_m": -1},
        {"directional_emergency_transition_m": 0},
    ],
)
def test_invalid_directional_configuration_is_rejected(invalid):
    with pytest.raises(ValueError):
        config(**invalid)
