"""Reuse work without changing safety decisions or recorded packets."""

import json
from unittest.mock import patch

import numpy as np
import pytest

from isaac_composer.crowd.config import validate_social_config
from isaac_composer.crowd.context.walkable import WalkableContext
from isaac_composer.crowd.runtime.recording import StateRecorder
from isaac_composer.crowd.avoidance.prediction import encounter_risk
from test_social_navigation import BoxScene


@pytest.mark.parametrize("target,allowed", [([0.5, 0], True), ([20.0, 0], False)])
def test_motion_check_results_are_identical_and_expire_at_the_next_observation(target, allowed):
    context = WalkableContext(BoxScene())
    with patch.object(context, "_motion_allowed", wraps=context._motion_allowed) as check:
        # Callers outside the crowd loop retain uncached behavior.
        assert context.motion_allowed([0, 0], target, 0.3, 0.02) == allowed
        context.begin_tick()
        for _ in range(10):
            assert context.motion_allowed([0, 0], target, 0.3, 0.02) == allowed
        assert check.call_count == 2
        context.motion_allowed([0, 0], target, 0.31, 0.02)
        context.motion_allowed([0, 0], target, 0.3, 0.03)
        assert check.call_count == 4
        context.begin_tick()
        assert context.motion_allowed([0, 0], target, 0.3, 0.02) == allowed
        assert check.call_count == 5


def test_published_serialization_is_reused_verbatim_in_the_ground_truth_record(tmp_path):
    recorder = StateRecorder(
        {
            "state_output_dir": str(tmp_path),
            "people": [],
            "social_navigation": validate_social_config(
                {"enabled": False, "yielding_enabled": False}
            ),
        }
    )
    packet = {"stamp_ns": 1000000000, "frame_id": "world", "pedestrians": [[1, 2, 3]]}
    serialized = json.dumps(packet, separators=(",", ":"))
    with patch("isaac_composer.crowd.runtime.recording.json.dumps", side_effect=AssertionError):
        recorder.write(packet, serialized=serialized)
    recorder.close()
    assert (tmp_path / "crowd_states.jsonl").read_text() == serialized + "\n"


def test_collection_can_disable_high_rate_crowd_files_without_affecting_runtime_packets(tmp_path):
    recorder = StateRecorder(
        {
            "state_output_dir": str(tmp_path),
            "people": [],
            "social_navigation": validate_social_config({"record_states_enabled": False}),
        }
    )
    recorder.write({"stamp_ns": 1000000000, "pedestrians": []})
    recorder.close()
    assert recorder.stream is None
    assert not list(tmp_path.iterdir())


def test_repeated_native_origins_are_projected_once_without_reusing_next_tick_results():
    scene = BoxScene()
    scene.routing_radius_m = 0.35
    scene.project_world = lambda p, agent_radius: p
    context = WalkableContext(scene)
    with patch.object(scene, "project_world", wraps=scene.project_world) as project:
        context.begin_tick()
        assert context.motion_allowed([0, 0], [0.02, 0], 0.3, 0.02)
        assert context.motion_allowed([0, 0], [0, 0.02], 0.3, 0.02)
        assert project.call_count == 3  # One origin, two endpoints.
        context.begin_tick()
        assert context.motion_allowed([0, 0], [0.02, 0], 0.3, 0.02)
        assert project.call_count == 5


def test_scalar_encounter_prediction_matches_original_numpy_formula():
    rng = np.random.default_rng(123)
    cases = [
        (
            rng.normal(size=3),
            rng.normal(size=3),
            rng.normal(size=2),
            rng.normal(size=2),
            rng.uniform(0, 3),
            rng.uniform(0, 10),
        )
        for _ in range(2000)
    ]
    cases += [
        ([0, 0, 0], [1, 0, 0], [0, 0], [0, 0], 1, 3),
        ([0, 0, 0], [0.9, 0, 0], [0, 0], [0, 0], 1, 3),
        ([0, 0, 0], [2, 0, 0], [1, 0], [-1, 0], 0.6, 3),
    ]
    for p, q, v, w, radius, horizon in cases:
        relative_p = np.asarray(q)[:2] - np.asarray(p)[:2]
        relative_v = np.asarray(w) - np.asarray(v)
        speed_sq = float(np.dot(relative_v, relative_v))
        t = float(np.clip(-np.dot(relative_p, relative_v) / max(speed_sq, 1e-12), 0, horizon))
        expected = bool(np.linalg.norm(relative_p + t * relative_v) < radius)
        risk, actual_t = encounter_risk(p, q, v, w, radius, horizon)
        assert risk == expected
        assert actual_t == pytest.approx(t, abs=1e-12)
