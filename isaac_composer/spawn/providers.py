"""Built-in robot spawn providers."""

from __future__ import annotations

from pathlib import Path
from typing import Any

from isaac_composer.assets import resolve_asset
from isaac_composer.models import LinkContext, RobotContext
from isaac_composer.registry import spawn_providers
from isaac_composer.stage import add_usd_reference, set_xform_pose


class _SpawnerBase:
    def __init__(
        self,
        *,
        package_root: str | Path,
        config_root: str | Path | None = None,
    ) -> None:
        self.package_root = Path(package_root)
        self.config_root = Path(config_root) if config_root is not None else None

    @staticmethod
    def _root_path(config: dict[str, Any]) -> str:
        preset = config["preset"]
        instance = config["instance"]
        configured = preset["robot"]["prim_path"]
        if config.get("multiple_instances", False):
            return f"/World/Robots/{instance['instance_name']}"
        return configured

    @staticmethod
    def _context(
        config: dict[str, Any],
        root_path: str,
        *,
        runtime_handle: Any | None = None,
    ) -> RobotContext:
        preset = config["preset"]
        robot_config = preset["robot"]
        links = {}
        for alias, link in robot_config.get("links", {}).items():
            relative_path = link["prim_path"].strip("/")
            links[alias] = LinkContext(
                name=alias,
                prim_path=f"{root_path}/{relative_path}",
                frame_id=link["frame_id"],
            )

        articulation = robot_config.get("articulation_root")
        articulation_path = (
            f"{root_path}/{articulation.strip('/')}" if articulation else None
        )
        instance = config["instance"]
        return RobotContext(
            name=instance["instance_name"],
            root_prim_path=root_path,
            articulation_prim_path=articulation_path,
            links=links,
            runtime_handle=runtime_handle,
            metadata={
                "namespace": instance.get("namespace", "").strip("/"),
                "model": preset.get("model", preset["name"]),
                "preset_name": preset["name"],
            },
        )


@spawn_providers.register("usd_reference")
class UsdReferenceSpawner(_SpawnerBase):
    """Create a robot prim that references a configured USD asset."""

    def spawn(
        self,
        config: dict[str, Any],
        *,
        stage: Any,
        world: Any | None,
    ) -> RobotContext:
        del world
        preset = config["preset"]
        instance = config["instance"]
        root_path = self._root_path(config)
        usd_path = resolve_asset(preset["robot"]["usd_path"], self.package_root)
        add_usd_reference(stage, root_path, usd_path)
        pose = instance["pose"]
        set_xform_pose(
            stage,
            root_path,
            pose["position"],
            pose["orientation_quat_wxyz"],
        )
        return self._context(config, root_path)


@spawn_providers.register("spot_policy")
class SpotPolicySpawner(_SpawnerBase):
    """Let Isaac Sim's Spot policy create and own the Spot articulation."""

    def spawn(
        self,
        config: dict[str, Any],
        *,
        stage: Any,
        world: Any | None,
    ) -> RobotContext:
        del stage
        if world is None:
            raise RuntimeError("Spot policy spawning requires an Isaac Sim World")

        import numpy as np
        from isaacsim.robot.policy.examples.robots import SpotFlatTerrainPolicy

        preset = config["preset"]
        instance = config["instance"]
        root_path = self._root_path(config)
        pose = instance["pose"]
        policy = SpotFlatTerrainPolicy(
            prim_path=root_path,
            name=instance["instance_name"],
            usd_path=resolve_asset(preset["robot"]["usd_path"], self.package_root),
            position=np.asarray(pose["position"], dtype=np.float64),
            orientation=np.asarray(
                pose["orientation_quat_wxyz"],
                dtype=np.float64,
            ),
        )
        context = self._context(config, root_path, runtime_handle=policy)
        context.metadata["spawn_pose"] = pose
        return context


@spawn_providers.register("a2_policy")
class A2PolicySpawner(_SpawnerBase):
    """Create A2 with the package-owned exported locomotion policy."""

    def spawn(
        self,
        config: dict[str, Any],
        *,
        stage: Any,
        world: Any | None,
    ) -> RobotContext:
        del stage
        if world is None:
            raise RuntimeError("A2 policy spawning requires an Isaac Sim World")

        import numpy as np

        from isaac_composer.controllers.a2_policy import A2FlatTerrainPolicy

        preset = config["preset"]
        instance = config["instance"]
        root_path = self._root_path(config)
        pose = instance["pose"]
        policy = A2FlatTerrainPolicy(
            prim_path=root_path,
            name=instance["instance_name"],
            usd_path=resolve_asset(preset["robot"]["usd_path"], self.package_root),
            policy_path=resolve_asset(
                preset["controller"]["policy_path"],
                self.package_root,
            ),
            position=np.asarray(pose["position"], dtype=np.float64),
            orientation=np.asarray(
                pose["orientation_quat_wxyz"],
                dtype=np.float64,
            ),
            policy_dt=float(preset["controller"].get("policy_dt", 0.02)),
            action_scale=float(preset["controller"].get("action_scale", 0.25)),
        )
        context = self._context(config, root_path, runtime_handle=policy)
        context.metadata["spawn_pose"] = pose
        return context
