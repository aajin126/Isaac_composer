"""Training-contract regression tests, runnable without Isaac Sim."""
import sys
from types import SimpleNamespace
from unittest.mock import MagicMock, patch

import numpy as np
import pytest

from isaac_composer.controllers.a2_policy import (
    A2FlatTerrainPolicy, MJLAB_JOINT_NAMES,
    build_observation, policy_joint_indices,
)


def observation(command, time=0.0):
    return build_observation(
        linear_velocity_world=[100, 200, 300],
        angular_velocity_world=[1, 2, 3],
        orientation_wxyz=[1, 0, 0, 0], command=command,
        joint_position=np.arange(12) + 0.5, joint_velocity=np.arange(12) + 20,
        default_position=np.full(12, 0.5), last_action=np.arange(12) + 40,
        policy_format='mjlab', policy_time=time,
    )


def test_mjlab_observation_layout_and_phase():
    obs = observation([0.5, 0, 0], 0.15)
    assert obs.shape == (47,)
    np.testing.assert_allclose(obs[:9], [1, 2, 3, 0, 0, -1, 0.5, 0, 0])
    np.testing.assert_allclose(obs[9:11], [1, 0], atol=1e-7)
    np.testing.assert_array_equal(obs[11:23], np.arange(12))
    np.testing.assert_array_equal(obs[23:35], np.arange(12) + 20)
    np.testing.assert_array_equal(obs[35:47], np.arange(12) + 40)
    np.testing.assert_allclose(observation([0.5, 0, 0], 0.75), obs, atol=1e-7)
    np.testing.assert_array_equal(observation([0, 0, 0], 0.15)[9:11], [0, 0])
    # Standing threshold uses the full command, including turning in place.
    np.testing.assert_allclose(observation([0, 0, 0.1])[9:11], [0, 1])


def test_permuted_usd_joints_round_trip():
    usd_names = tuple(reversed(MJLAB_JOINT_NAMES))
    indices = policy_joint_indices(usd_names, 'mjlab')
    assert tuple(usd_names[i] for i in indices) == MJLAB_JOINT_NAMES
    np.testing.assert_array_equal(policy_joint_indices(usd_names, 'flat'), np.arange(12))
    with pytest.raises(ValueError):
        policy_joint_indices(usd_names[:-1], 'mjlab')


def test_runtime_gathers_observations_scatters_actions_and_resets_phase():
    runtime = A2FlatTerrainPolicy.__new__(A2FlatTerrainPolicy)
    runtime.robot = MagicMock()
    runtime.robot.dof_names = tuple(reversed(MJLAB_JOINT_NAMES))
    runtime._policy_format = 'mjlab'
    runtime._policy_dt = 0.02
    runtime._action_scale = 0.25
    runtime._last_action = np.ones(12)
    runtime.initialize()
    runtime.robot.get_articulation_controller.return_value.set_effort_modes.assert_not_called()
    runtime.robot.get_world_pose.return_value = (np.zeros(3), [1, 0, 0, 0])
    runtime.robot.get_linear_velocity.return_value = [0, 0, 0]
    runtime.robot.get_angular_velocity.return_value = [0, 0, 0]
    runtime.robot.get_joint_positions.return_value = runtime.default_pos + np.arange(12)
    runtime.robot.get_joint_velocities.return_value = np.arange(12) + 20
    runtime._torch = MagicMock()
    runtime._policy = MagicMock()
    actions = np.arange(12, dtype=float)
    runtime._policy.return_value.squeeze.return_value.cpu.return_value.numpy.return_value = actions
    with patch.dict(sys.modules, {'isaacsim.core.utils.types': SimpleNamespace(ArticulationAction=SimpleNamespace)}):
        runtime.forward(0.002, [0.5, 0, 0])
        obs = runtime._torch.from_numpy.call_args.args[0]
        np.testing.assert_array_equal(obs[11:23], np.arange(12)[::-1])
        target = runtime.robot.apply_action.call_args.args[0].joint_positions
        np.testing.assert_allclose(target, runtime.default_pos + 0.25 * actions[::-1])
        np.testing.assert_array_equal(obs[35:], np.zeros(12))
        for _ in range(10):
            runtime.forward(0.002, [0.5, 0, 0])
        obs = runtime._torch.from_numpy.call_args.args[0]
        np.testing.assert_array_equal(obs[35:], actions)
        np.testing.assert_allclose(obs[9:11], [np.sin(2*np.pi/30), np.cos(2*np.pi/30)])
    runtime.initialize()
    assert runtime._policy_steps == 0
    assert runtime._joint_target is None
    np.testing.assert_array_equal(runtime._last_action, np.zeros(12))
