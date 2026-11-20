"""Runtime adapter for the exported Isaac Lab Unitree A2 policy."""

from __future__ import annotations

from pathlib import Path
from typing import Any, Sequence

import numpy as np

MJLAB_JOINT_NAMES = tuple(
    f"{leg}_{joint}_joint"
    for leg in ("FL", "FR", "RL", "RR")
    for joint in ("hip", "thigh", "calf")
)


def policy_joint_indices(joint_names: Sequence[str], policy_format: str) -> np.ndarray:
    """Indices gathering USD joint state into the policy's training order."""
    if policy_format not in ("flat", "mjlab"):
        raise ValueError(f"Unknown A2 policy format: {policy_format!r}")
    if len(joint_names) != 12 or set(joint_names) != set(MJLAB_JOINT_NAMES):
        raise ValueError(f"Expected the 12 A2 joints, got {joint_names}")
    order = MJLAB_JOINT_NAMES if policy_format == "mjlab" else joint_names
    return np.asarray([joint_names.index(name) for name in order], dtype=np.int64)


def _rotation_matrix_from_wxyz(quaternion: Sequence[float]) -> np.ndarray:
    """Return the body-to-world rotation matrix for a WXYZ quaternion."""

    w, x, y, z = np.asarray(quaternion, dtype=np.float64)
    norm = np.linalg.norm([w, x, y, z])
    if norm == 0.0:
        raise ValueError("Robot orientation quaternion must be non-zero")
    w, x, y, z = np.asarray([w, x, y, z], dtype=np.float64) / norm
    return np.asarray(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def _joint_parameters(joint_names: Sequence[str]) -> tuple[np.ndarray, ...]:
    """Build the training-time A2 defaults and drive parameters in USD DOF order."""

    default_position = []
    stiffness = []
    damping = []
    max_effort = []
    expected_suffixes = ("_hip_joint", "_thigh_joint", "_calf_joint")
    for name in joint_names:
        if name.endswith("_hip_joint"):
            default_position.append(-0.1 if "L_hip_joint" in name else 0.1)
            stiffness.append(100.0)
            damping.append(4.0)
            max_effort.append(120.0)
        elif name.endswith("_thigh_joint"):
            default_position.append(0.9)
            stiffness.append(100.0)
            damping.append(4.0)
            max_effort.append(120.0)
        elif name.endswith("_calf_joint"):
            default_position.append(-1.8)
            stiffness.append(150.0)
            damping.append(6.0)
            max_effort.append(180.0)
        else:
            expected = ", ".join(expected_suffixes)
            raise RuntimeError(f"Unexpected A2 joint {name!r}; expected suffixes: {expected}")

    if len(joint_names) != 12:
        raise RuntimeError(f"A2 policy requires 12 joints, found {len(joint_names)}")
    return tuple(
        np.asarray(values, dtype=np.float64)
        for values in (default_position, stiffness, damping, max_effort)
    )


def build_observation(
    *,
    linear_velocity_world: Sequence[float],
    angular_velocity_world: Sequence[float],
    orientation_wxyz: Sequence[float],
    command: Sequence[float],
    joint_position: Sequence[float],
    joint_velocity: Sequence[float],
    default_position: Sequence[float],
    last_action: Sequence[float],
    policy_format: str = "flat",
    policy_time: float = 0.0,
) -> np.ndarray:
    """Build flat (48) or MJLab (47) input; joint arrays are in policy order."""

    if policy_format not in ("flat", "mjlab"):
        raise ValueError(f"Unknown A2 policy format: {policy_format!r}")

    rotation_world_from_body = _rotation_matrix_from_wxyz(orientation_wxyz)
    rotation_body_from_world = rotation_world_from_body.T
    linear_velocity_body = rotation_body_from_world @ np.asarray(
        linear_velocity_world, dtype=np.float64
    )
    angular_velocity_body = rotation_body_from_world @ np.asarray(
        angular_velocity_world, dtype=np.float64
    )
    projected_gravity = rotation_body_from_world @ np.asarray(
        [0.0, 0.0, -1.0], dtype=np.float64
    )
    command = np.asarray(command, dtype=np.float64)
    prefix = [angular_velocity_body, projected_gravity, command]
    if policy_format == "mjlab":
        angle = 2.0 * np.pi * ((policy_time % 0.6) / 0.6)
        phase = np.asarray([np.sin(angle), np.cos(angle)])
        if np.linalg.norm(command) < 0.1:
            phase[:] = 0.0
        prefix.append(phase)
    else:
        prefix.insert(0, linear_velocity_body)
    observation = np.concatenate(
        (
            *prefix,
            np.asarray(joint_position, dtype=np.float64)
            - np.asarray(default_position, dtype=np.float64),
            np.asarray(joint_velocity, dtype=np.float64),
            np.asarray(last_action, dtype=np.float64),
        )
    ).astype(np.float32)
    expected_size = 47 if policy_format == "mjlab" else 48
    if observation.shape != (expected_size,):
        raise RuntimeError(f"A2 policy observation must have shape ({expected_size},), got {observation.shape}")
    return observation


class A2FlatTerrainPolicy:
    """Run an exported Isaac Lab A2 policy on an Isaac Sim articulation."""

    def __init__(
        self,
        *,
        prim_path: str,
        name: str,
        usd_path: str,
        policy_path: str | Path,
        position: np.ndarray,
        orientation: np.ndarray,
        policy_dt: float = 0.02,
        action_scale: float = 0.25,
        policy_format: str = "flat",
    ) -> None:
        import torch
        from isaacsim.core.prims import SingleArticulation
        from isaacsim.core.utils.prims import define_prim, get_prim_at_path

        prim = get_prim_at_path(prim_path)
        if not prim.IsValid():
            prim = define_prim(prim_path, "Xform")
            prim.GetReferences().AddReference(usd_path)

        self.robot = SingleArticulation(
            prim_path=prim_path,
            name=name,
            position=position,
            orientation=orientation,
        )
        self._torch = torch
        self._policy = torch.jit.load(str(policy_path), map_location="cpu").eval()
        self._policy_dt = float(policy_dt)
        self._action_scale = float(action_scale)
        if policy_format not in ("flat", "mjlab"):
            raise ValueError(f"Unknown A2 policy format: {policy_format!r}")
        self._policy_format = policy_format
        if self._policy_dt <= 0.0:
            raise ValueError("A2 policy_dt must be positive")
        if self._action_scale <= 0.0:
            raise ValueError("A2 action_scale must be positive")

        self.default_pos = np.empty(0, dtype=np.float64)
        self.default_vel = np.empty(0, dtype=np.float64)
        self._last_action = np.zeros(12, dtype=np.float64)
        self._joint_target: np.ndarray | None = None
        self._elapsed = self._policy_dt
        self._policy_steps = 0
        self._policy_joint_indices = np.arange(12)

    def initialize(self) -> None:
        self.robot.initialize()
        joint_names = tuple(self.robot.dof_names)
        self._policy_joint_indices = policy_joint_indices(joint_names, self._policy_format)
        default_pos, stiffness, damping, max_effort = _joint_parameters(joint_names)
        self.default_pos = default_pos
        self.default_vel = np.zeros_like(default_pos)

        controller = self.robot.get_articulation_controller()
        # MJLab training keeps the asset's drive type rather than rewriting
        # the USD drive mode after initialization.
        if self._policy_format == "flat":
            controller.set_effort_modes("force")
        controller.switch_control_mode("position")
        controller.set_gains(kps=stiffness, kds=damping)
        controller.set_max_efforts(max_effort)
        if self._policy_format == "mjlab":
            # Match UNITREE_A2_CFG and MJLab's velocity-limit override.
            view = self.robot._articulation_view
            view.set_armatures(np.full((1, 12), 0.03))
            view.set_max_joint_velocities(np.full((1, 12), 1.0e9))
            self.robot.set_joint_positions(self.default_pos)
            self.robot.set_joint_velocities(self.default_vel)
        self._last_action[:] = 0.0
        self._joint_target = None
        self._elapsed = self._policy_dt
        self._policy_steps = 0

    def forward(self, dt: float, command: Sequence[float]) -> None:
        from isaacsim.core.utils.types import ArticulationAction

        self._elapsed += float(dt)
        if self._joint_target is None or self._elapsed + 1.0e-12 >= self._policy_dt:
            self._elapsed %= self._policy_dt
            _, orientation = self.robot.get_world_pose()
            indices = self._policy_joint_indices
            observation = build_observation(
                linear_velocity_world=self.robot.get_linear_velocity(),
                angular_velocity_world=self.robot.get_angular_velocity(),
                orientation_wxyz=orientation,
                command=command,
                joint_position=np.asarray(self.robot.get_joint_positions())[indices],
                joint_velocity=np.asarray(self.robot.get_joint_velocities())[indices],
                default_position=self.default_pos[indices],
                last_action=self._last_action,
                policy_format=self._policy_format,
                policy_time=self._policy_steps * self._policy_dt,
            )
            with self._torch.inference_mode():
                action = (
                    self._policy(self._torch.from_numpy(observation).unsqueeze(0))
                    .squeeze(0)
                    .cpu()
                    .numpy()
                )
            if action.shape != (12,) or not np.all(np.isfinite(action)):
                raise RuntimeError(f"A2 policy returned an invalid action: shape={action.shape}")
            self._last_action = action.astype(np.float64)
            self._joint_target = self.default_pos.copy()
            self._joint_target[indices] += self._action_scale * self._last_action
            self._policy_steps += 1

        self.robot.apply_action(ArticulationAction(joint_positions=self._joint_target))
