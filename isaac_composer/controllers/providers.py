"""Built-in controller lifecycle implementations."""

from __future__ import annotations

from pathlib import Path
from typing import Any

from isaac_composer.models import RobotContext
from isaac_composer.registry import controller_providers


class _ProviderBase:
    def __init__(
        self,
        *,
        package_root: str | Path,
        config_root: str | Path | None = None,
    ) -> None:
        del package_root, config_root


@controller_providers.register("embedded")
@controller_providers.register("joint_command_graph")
class GraphOwnedController(_ProviderBase):
    """No-op lifecycle for controllers owned by a USD or ROS ActionGraph."""

    def prepare(self, robot: RobotContext, config: dict[str, Any]) -> None:
        del robot, config

    def start_after_reset(self, robot: RobotContext, world: Any) -> None:
        del robot, world

    def step(self, step_size: float) -> None:
        del step_size

    def shutdown(self) -> None:
        return None


@controller_providers.register("from_spawner")
class PolicyBasedController(_ProviderBase):
    """Drive a policy-based robot (Spot, A2) from the generated ROS Twist graph."""

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self.robot: RobotContext | None = None
        self.policy: Any | None = None
        self.world: Any | None = None
        self.config: dict[str, Any] = {}
        self.callback_name: str | None = None
        self.linear_velocity_attribute: Any | None = None
        self.angular_velocity_attribute: Any | None = None
        self.initialized = False
        self.command_warning_printed = False

    def prepare(self, robot: RobotContext, config: dict[str, Any]) -> None:
        if robot.runtime_handle is None:
            raise RuntimeError("from_spawner controller requires a policy handle")
        self.robot = robot
        self.policy = robot.runtime_handle
        self.config = dict(config)

    def start_after_reset(self, robot: RobotContext, world: Any) -> None:
        import omni.graph.core as og

        graph_path = f"{robot.root_prim_path}/ROS2_ActionGraph"
        self.linear_velocity_attribute = og.Controller.attribute(
            f"{graph_path}/SubscribeTwist.outputs:linearVelocity"
        )
        self.angular_velocity_attribute = og.Controller.attribute(
            f"{graph_path}/SubscribeTwist.outputs:angularVelocity"
        )
        if (
            not self.linear_velocity_attribute.is_valid()
            or not self.angular_velocity_attribute.is_valid()
        ):
            raise RuntimeError(f"Policy cmd_vel subscriber is missing from {graph_path}")

        # Initialize policy if it has an initialize method (e.g., A2FlatTerrainPolicy)
        if hasattr(self.policy, 'initialize'):
            self.policy.initialize()

        self.world = world
        self.callback_name = f"{robot.name}_policy_locomotion"
        world.add_physics_callback(
            self.callback_name,
            callback_fn=self._on_physics_step,
        )

    def _read_command(self):
        import numpy as np
        import omni.graph.core as og

        try:
            linear = og.Controller.get(self.linear_velocity_attribute)
            angular = og.Controller.get(self.angular_velocity_attribute)
            if linear is None or angular is None:
                raise ValueError("cmd_vel graph output is not ready")
            command = np.asarray(
                [linear[0], linear[1], angular[2]],
                dtype=np.float64,
            )
            if not np.all(np.isfinite(command)):
                raise ValueError(f"invalid cmd_vel output: {command}")
            limits = np.asarray(
                self.config.get("max_command", [2.0, 1.0, 2.0]),
                dtype=np.float64,
            )
            return np.clip(command, -limits, limits)
        except Exception as error:
            if not self.command_warning_printed:
                robot_name = self.robot.name if self.robot else "Robot"
                print(
                    f"[{robot_name}] cmd_vel is not ready; using zero command: {error}",
                    flush=True,
                )
                self.command_warning_printed = True
            return np.zeros(3, dtype=np.float64)

    def _on_physics_step(self, step_size: float) -> None:
        import numpy as np

        if not self.initialized:
            self.policy.initialize()
            policy_robot = self.policy.robot
            default_position = np.asarray(self.policy.default_pos, dtype=np.float64)
            default_velocity = np.asarray(self.policy.default_vel, dtype=np.float64)
            pose = self.robot.metadata.get("spawn_pose", {})
            if pose:
                policy_robot.set_world_pose(
                    position=np.asarray(pose["position"], dtype=np.float64),
                    orientation=np.asarray(
                        pose["orientation_quat_wxyz"],
                        dtype=np.float64,
                    ),
                )
            policy_robot.set_linear_velocity(np.zeros(3, dtype=np.float64))
            policy_robot.set_angular_velocity(np.zeros(3, dtype=np.float64))
            policy_robot.set_joint_positions(default_position)
            policy_robot.set_joint_velocities(default_velocity)
            policy_robot.set_joints_default_state(
                positions=default_position,
                velocities=default_velocity,
            )
            self.initialized = True
            return

        self.policy.forward(step_size, self._read_command())

    def step(self, step_size: float) -> None:
        # Policy-based robot (Spot, A2) is advanced by the physics callback registered above.
        del step_size

    def shutdown(self) -> None:
        if self.world is not None and self.callback_name is not None:
            try:
                self.world.remove_physics_callback(self.callback_name)
            except Exception:
                pass
        self.robot = None
        self.policy = None
        self.world = None
        self.callback_name = None
        self.linear_velocity_attribute = None
        self.angular_velocity_attribute = None
        self.initialized = False
