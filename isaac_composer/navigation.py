"""Turn a validated scenario bundle into robot-agnostic Nav2 launch plans."""

from __future__ import annotations

import copy
import math
import os
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from .config import ConfigError


@dataclass
class NavigationInstancePlan:
    """All launch-time values for one Nav2 stack."""

    instance_name: str
    namespace: str
    map_file: Path
    params_file: Path
    rviz_file: Path
    initial_pose: dict[str, float]
    use_sim_time: bool
    autostart: bool
    use_rviz: bool
    pointcloud_enabled: bool
    cloud_topic: str
    cloud_topics: list[str]
    scan_topic: str
    converter_scan_topics: list[str]
    pointcloud_parameters: dict[str, Any]
    scan_fusion_parameters: dict[str, Any]
    lidar_fusion_enabled: bool
    lidar_fusion_input_topics: list[str]
    lidar_fusion_output_topic: str
    lidar_fusion_target_frame: str
    parameter_overrides: dict[str, Any]

    @property
    def use_namespace(self) -> bool:
        return bool(self.namespace)


@dataclass
class NavigationPlan:
    """Scenario-level launch plan shared by single- and multi-robot cases."""

    scenario_name: str
    enabled: bool
    startup_delay_sec: float
    instances: list[NavigationInstancePlan]


def _resolve_asset(package_root: Path, relative_path: str, context: str) -> Path:
    raw = Path(relative_path)
    if raw.is_absolute():
        raise ConfigError(f"{context} must be relative to the isaac_composer package share")
    # Keep the lexical package-share path here. ``resolve()`` follows colcon's
    # --symlink-install links into build/source and would falsely report that a
    # valid installed asset escaped the package share.
    package_root = Path(os.path.abspath(package_root))
    resolved = Path(os.path.abspath(package_root / raw))
    if package_root not in resolved.parents:
        raise ConfigError(f"{context} escapes the isaac_composer package share: {raw}")
    if not resolved.is_file():
        raise ConfigError(f"{context} does not exist: {resolved}")
    return resolved


def _yaw_from_quaternion_wxyz(quaternion: list[float]) -> float:
    w, x, y, z = quaternion
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _initial_pose(instance: dict[str, Any]) -> dict[str, float]:
    override = instance["navigation"].get("initial_pose")
    if override is not None:
        return {key: float(override[key]) for key in ("x", "y", "z", "yaw")}

    spawn_pose = instance["pose"]
    position = spawn_pose["position"]
    return {
        "x": float(position[0]),
        "y": float(position[1]),
        # AMCL is planar. The physical spawn height is intentionally not copied.
        "z": 0.0,
        "yaw": _yaw_from_quaternion_wxyz(spawn_pose["orientation_quat_wxyz"]),
    }


def _merge_parameter_overrides(*mappings: dict[str, Any]) -> dict[str, Any]:
    merged: dict[str, Any] = {}
    for mapping in mappings:
        merged.update(copy.deepcopy(mapping))
    return merged


def build_navigation_plan(
    bundle: dict[str, Any],
    package_root: str | Path | None = None,
) -> NavigationPlan:
    """Build a concrete plan without importing ROS 2 launch libraries."""

    scenario = bundle["scenario"]
    settings = bundle["navigation"]["settings"]
    enabled = settings["enabled"]
    delay = float(settings.get("startup_delay_sec", 0.0))
    if not enabled:
        return NavigationPlan(scenario["name"], False, delay, [])

    root = (
        Path(package_root).resolve()
        if package_root is not None
        else Path(bundle["config_root"]).resolve().parent
    )
    map_config = bundle["navigation"]["map"]
    if map_config is None:
        raise ConfigError("Enabled navigation requires a loaded map preset")
    map_file = _resolve_asset(root, map_config["map_file"], "navigation_map.map_file")

    use_sim_time = settings.get("use_sim_time", True)
    autostart = settings.get("autostart", True)
    use_rviz = settings.get("use_rviz", True)
    plans: list[NavigationInstancePlan] = []
    namespaces: set[str] = set()

    for item in bundle["robots"]:
        instance = item["instance"]
        if not instance["navigation"]["enabled"]:
            continue
        profile = item["navigation_profile"]
        if profile is None:
            raise ConfigError(
                f"Robot {instance['instance_name']!r} has no navigation profile"
            )
        if not profile["supported"]:
            raise ConfigError(
                f"Navigation profile {profile['name']!r} is unsupported: "
                f"{profile['reason']}"
            )

        namespace = instance.get("namespace", "").strip("/")
        if namespace in namespaces:
            display = namespace or "<root>"
            raise ConfigError(f"Duplicate navigation namespace: {display}")
        namespaces.add(namespace)

        converter = profile["pointcloud_to_laserscan"]
        cloud_topic = converter.get("cloud_topic", "")
        cloud_topics = [str(topic) for topic in converter.get("cloud_topics", [])]
        if not cloud_topics:
            cloud_topics = [cloud_topic]
        scan_topic = converter.get("scan_topic", "")
        converter_scan_topics = [
            str(topic) for topic in converter.get("intermediate_scan_topics", [])
        ]
        if len(cloud_topics) == 1:
            converter_scan_topics = [scan_topic]
        elif len(converter_scan_topics) != len(cloud_topics):
            raise ConfigError(
                f"Profile {profile['name']!r} must provide one "
                "intermediate_scan_topic for each cloud_topic"
            )
        converter_topics = (*cloud_topics, *converter_scan_topics, scan_topic)
        if namespace and any(topic.startswith("/") for topic in converter_topics):
            raise ConfigError(
                f"Profile {profile['name']!r} must use relative converter topics "
                "when a namespace is configured"
            )

        fusion = profile.get("lidar_fusion", {})
        fusion_enabled = bool(fusion.get("enabled", False))
        fusion_input_topics = [str(topic) for topic in fusion.get("input_topics", [])]
        fusion_output_topic = str(fusion.get("output_topic", ""))
        fusion_target_frame = str(fusion.get("target_frame", ""))
        if fusion_enabled:
            if len(fusion_input_topics) != 2:
                raise ConfigError(
                    f"Profile {profile['name']!r} lidar_fusion.input_topics must "
                    "list exactly two topics"
                )
            if not fusion_output_topic or not fusion_target_frame:
                raise ConfigError(
                    f"Profile {profile['name']!r} lidar_fusion requires "
                    "output_topic and target_frame when enabled"
                )
            fusion_topics = (*fusion_input_topics, fusion_output_topic)
            if namespace and any(topic.startswith("/") for topic in fusion_topics):
                raise ConfigError(
                    f"Profile {profile['name']!r} must use relative lidar_fusion "
                    "topics when a namespace is configured"
                )

        nav2 = profile["nav2"]
        plans.append(
            NavigationInstancePlan(
                instance_name=instance["instance_name"],
                namespace=namespace,
                map_file=map_file,
                params_file=_resolve_asset(
                    root,
                    nav2["params_file"],
                    f"navigation profile {profile['name']}.nav2.params_file",
                ),
                rviz_file=_resolve_asset(
                    root,
                    nav2["rviz_file"],
                    f"navigation profile {profile['name']}.nav2.rviz_file",
                ),
                initial_pose=_initial_pose(instance),
                use_sim_time=use_sim_time,
                autostart=autostart,
                use_rviz=use_rviz,
                pointcloud_enabled=converter["enabled"],
                cloud_topic=cloud_topic,
                cloud_topics=cloud_topics,
                scan_topic=scan_topic,
                converter_scan_topics=converter_scan_topics,
                pointcloud_parameters=copy.deepcopy(converter.get("parameters", {})),
                scan_fusion_parameters=copy.deepcopy(
                    converter.get("fusion_parameters", {})
                ),
                lidar_fusion_enabled=fusion_enabled,
                lidar_fusion_input_topics=fusion_input_topics,
                lidar_fusion_output_topic=fusion_output_topic,
                lidar_fusion_target_frame=fusion_target_frame,
                parameter_overrides=_merge_parameter_overrides(
                    profile.get("parameter_overrides", {}),
                    settings.get("parameter_overrides", {}),
                    instance["navigation"].get("parameter_overrides", {}),
                ),
            )
        )

    if not plans:
        raise ConfigError("Navigation is enabled but no robot navigation instances are enabled")
    return NavigationPlan(scenario["name"], True, delay, plans)
