"""Load and validate unified Isaac Composer YAML documents."""

from __future__ import annotations

import copy
import math
from pathlib import Path
from typing import Any

import yaml


ROBOT_SECTIONS = (
    "schema_version",
    "name",
    "spawn",
    "robot",
    "controller",
    "sensors",
    "ros",
    "clock",
)

SCENARIO_SECTIONS = (
    "schema_version",
    "name",
    "simulation",
    "environment",
    "pedestrians",
    "navigation",
    "robots",
)


class ConfigError(ValueError):
    """Raised when a composition document is missing required information."""


def default_config_root() -> Path:
    """Find source-tree config first, then the installed ROS package share."""

    source_root = Path(__file__).resolve().parent.parent / "config"
    if source_root.is_dir():
        return source_root

    try:
        from ament_index_python.packages import get_package_share_directory

        installed_root = Path(get_package_share_directory("isaac_composer")) / "config"
        if installed_root.is_dir():
            return installed_root
    except (ImportError, LookupError):
        pass

    raise ConfigError("Could not locate the isaac_composer config directory")


def _config_root(config_root: str | Path | None) -> Path:
    return Path(config_root).resolve() if config_root is not None else default_config_root()


def _load_yaml(path: Path) -> dict[str, Any]:
    if not path.is_file():
        raise ConfigError(f"Configuration file does not exist: {path}")
    with path.open("r", encoding="utf-8") as stream:
        document = yaml.safe_load(stream) or {}
    if not isinstance(document, dict):
        raise ConfigError(f"Top-level YAML value must be a mapping: {path}")
    return document


def _resolve_named_document(name_or_path: str | Path, directory: Path) -> Path:
    raw = Path(name_or_path).expanduser()
    if raw.is_file():
        return raw.resolve()
    if raw.suffix in {".yaml", ".yml"} or raw.parent != Path("."):
        raise ConfigError(f"Configuration file does not exist: {raw}")
    candidate = directory / f"{raw.name}.yaml"
    if not candidate.is_file():
        available = ", ".join(path.stem for path in sorted(directory.glob("*.yaml")))
        raise ConfigError(
            f"Unknown configuration {raw.name!r}; available: {available or '<none>'}"
        )
    return candidate.resolve()


def _require_sections(document: dict[str, Any], sections: tuple[str, ...], context: str) -> None:
    missing = [key for key in sections if key not in document]
    if missing:
        raise ConfigError(f"{context} is missing sections: {', '.join(missing)}")


def _require_mapping(document: dict[str, Any], key: str, context: str) -> dict[str, Any]:
    value = document.get(key)
    if not isinstance(value, dict):
        raise ConfigError(f"{context}.{key} must be a mapping")
    return value


def _require_string(document: dict[str, Any], key: str, context: str) -> str:
    value = document.get(key)
    if not isinstance(value, str) or not value.strip():
        raise ConfigError(f"{context}.{key} must be a non-empty string")
    return value


def _validate_vector(value: Any, length: int, context: str) -> None:
    if not isinstance(value, list) or len(value) != length:
        raise ConfigError(f"{context} must be a list of {length} numbers")
    if any(not isinstance(item, (int, float)) for item in value):
        raise ConfigError(f"{context} must contain only numbers")


def _validate_positive_number(value: Any, context: str) -> None:
    if not isinstance(value, (int, float)) or value <= 0:
        raise ConfigError(f"{context} must be a positive number")


def _validate_nonnegative_number(value: Any, context: str) -> None:
    if not isinstance(value, (int, float)) or value < 0:
        raise ConfigError(f"{context} must be a non-negative number")


def _validate_bool(value: Any, context: str) -> None:
    if not isinstance(value, bool):
        raise ConfigError(f"{context} must be a boolean")


def _validate_provider(section: dict[str, Any], context: str) -> None:
    _require_string(section, "provider", context)


def _validate_pose(pose: Any, context: str) -> None:
    if not isinstance(pose, dict):
        raise ConfigError(f"{context} must be a mapping")
    _validate_vector(pose.get("position"), 3, f"{context}.position")
    _validate_vector(
        pose.get("orientation_quat_wxyz"),
        4,
        f"{context}.orientation_quat_wxyz",
    )


def _validate_robot_links(robot: dict[str, Any]) -> None:
    links = robot.get("links", {})
    if not isinstance(links, dict):
        raise ConfigError("robot.links must be a mapping")
    for alias, link in links.items():
        if not isinstance(alias, str) or not alias:
            raise ConfigError("robot.links aliases must be non-empty strings")
        if not isinstance(link, dict):
            raise ConfigError(f"robot.links.{alias} must be a mapping")
        _require_string(link, "prim_path", f"robot.links.{alias}")
        _require_string(link, "frame_id", f"robot.links.{alias}")


def _validate_sensor_item(item: Any, index: int) -> None:
    context = f"sensors.items[{index}]"
    if not isinstance(item, dict):
        raise ConfigError(f"{context} must be a mapping")
    _require_string(item, "name", context)
    _require_string(item, "type", context)
    source = _require_string(item, "source", context)
    if not isinstance(item.get("enabled", True), bool):
        raise ConfigError(f"{context}.enabled must be a boolean")

    if source == "generated":
        _require_string(item, "preset", context)
        mount = _require_mapping(item, "mount", context)
        _require_string(mount, "parent", f"{context}.mount")
        _validate_vector(mount.get("translation"), 3, f"{context}.mount.translation")
        _validate_vector(
            mount.get("orientation_quat_wxyz", [1.0, 0.0, 0.0, 0.0]),
            4,
            f"{context}.mount.orientation_quat_wxyz",
        )

    ros = item.get("ros")
    if ros is not None:
        if not isinstance(ros, dict):
            raise ConfigError(f"{context}.ros must be a mapping")
        _require_string(ros, "frame_id", f"{context}.ros")
        _require_string(ros, "topic", f"{context}.ros")


def validate_robot_config(document: dict[str, Any]) -> dict[str, Any]:
    """Validate intrinsic robot configuration; instance pose belongs to scenario."""

    normalized = copy.deepcopy(document)
    _require_sections(normalized, ROBOT_SECTIONS, "robot_config")
    if normalized["schema_version"] != 1:
        raise ConfigError("Only robot schema_version 1 is supported")

    _require_string(normalized, "name", "robot_config")
    spawn = _require_mapping(normalized, "spawn", "robot_config")
    robot = _require_mapping(normalized, "robot", "robot_config")
    controller = _require_mapping(normalized, "controller", "robot_config")
    sensors = _require_mapping(normalized, "sensors", "robot_config")
    ros = _require_mapping(normalized, "ros", "robot_config")
    clock = _require_mapping(normalized, "clock", "robot_config")

    for name, section in (
        ("spawn", spawn),
        ("controller", controller),
        ("sensors", sensors),
        ("ros", ros),
        ("clock", clock),
    ):
        _validate_provider(section, name)

    _require_string(robot, "usd_path", "robot")
    _require_string(robot, "prim_path", "robot")
    if "pose" in robot:
        raise ConfigError("robot.pose belongs in a scenario robot instance")
    _validate_robot_links(robot)

    items = sensors.get("items", [])
    if not isinstance(items, list):
        raise ConfigError("sensors.items must be a list")
    seen_names: set[str] = set()
    for index, item in enumerate(items):
        _validate_sensor_item(item, index)
        name = item["name"]
        if name in seen_names:
            raise ConfigError(f"Duplicate sensor name: {name}")
        seen_names.add(name)
    normalized["sensors"]["items"] = items
    return normalized


def validate_sensor_preset(document: dict[str, Any]) -> dict[str, Any]:
    normalized = copy.deepcopy(document)
    if normalized.get("schema_version") != 1:
        raise ConfigError("Only sensor preset schema_version 1 is supported")
    _require_string(normalized, "name", "sensor_preset")
    _require_string(normalized, "type", "sensor_preset")
    implementation = _require_mapping(normalized, "implementation", "sensor_preset")
    _require_string(implementation, "provider", "sensor_preset.implementation")
    return normalized


def validate_environment_config(document: dict[str, Any]) -> dict[str, Any]:
    normalized = copy.deepcopy(document)
    if normalized.get("schema_version") != 1:
        raise ConfigError("Only environment schema_version 1 is supported")
    _require_string(normalized, "name", "environment")
    asset = _require_mapping(normalized, "asset", "environment")
    _require_string(asset, "usd_path", "environment.asset")
    _require_string(asset, "prim_path", "environment.asset")
    return normalized


def validate_simulation_config(document: dict[str, Any]) -> dict[str, Any]:
    normalized = copy.deepcopy(document)
    if normalized.get("schema_version") != 1:
        raise ConfigError("Only simulation schema_version 1 is supported")
    _require_string(normalized, "name", "simulation")
    physics = _require_mapping(normalized, "physics", "simulation")
    rendering = _require_mapping(normalized, "rendering", "simulation")
    timeline = _require_mapping(normalized, "timeline", "simulation")
    _validate_positive_number(physics.get("dt"), "simulation.physics.dt")
    _validate_positive_number(rendering.get("dt"), "simulation.rendering.dt")
    _require_string(rendering, "renderer", "simulation.rendering")
    if "indirect_diffuse_intensity" in rendering:
        _validate_nonnegative_number(
            rendering["indirect_diffuse_intensity"],
            "simulation.rendering.indirect_diffuse_intensity",
        )
    _validate_positive_number(timeline.get("target_fps"), "simulation.timeline.target_fps")
    if "viewport" in normalized:
        viewport = _require_mapping(normalized, "viewport", "simulation")
        if viewport.get("mode", "follow") not in ("follow", "top_down", "free"):
            raise ConfigError("simulation.viewport.mode must be follow, top_down, or free")
        for key in ("distance_m", "height_m", "top_down_height_m"):
            if key in viewport:
                _validate_positive_number(viewport[key], f"simulation.viewport.{key}")
        if 'follow_cutaway' in viewport:
            _validate_bool(viewport['follow_cutaway'], 'simulation.viewport.follow_cutaway')
        if 'follow_cutaway_margin_m' in viewport:
            value = viewport['follow_cutaway_margin_m']
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value < 0:
                raise ConfigError('simulation.viewport.follow_cutaway_margin_m must be finite and non-negative')
        for key in ("look_ahead_m", "smoothing_sec", "heading_smoothing_sec", "height_smoothing_sec"):
            if key in viewport:
                value = viewport[key]
                if not isinstance(value, (int, float)) or not math.isfinite(value) or value < 0:
                    raise ConfigError(f"simulation.viewport.{key} must be finite and non-negative")
    return normalized


def validate_pedestrian_config(document: dict[str, Any]) -> dict[str, Any]:
    normalized = copy.deepcopy(document)
    if normalized.get("schema_version") != 1:
        raise ConfigError("Only pedestrian schema_version 1 is supported")
    _require_string(normalized, "name", "pedestrians")
    _require_mapping(normalized, "navmesh", "pedestrians")
    reference_speed = normalized.get("animation_walk_reference_mps", 1.0)
    if (isinstance(reference_speed, bool) or not isinstance(reference_speed, (int, float))
            or not math.isfinite(reference_speed) or reference_speed <= 0):
        raise ConfigError("pedestrians.animation_walk_reference_mps must be positive and finite")
    people = normalized.get("people")
    if not isinstance(people, list):
        raise ConfigError("pedestrians.people must be a list")
    for index, person in enumerate(people):
        context = f"pedestrians.people[{index}]"
        if not isinstance(person, dict):
            raise ConfigError(f"{context} must be a mapping")
        _require_string(person, "name", context)
        _require_string(person, "character", context)
        pose = _require_mapping(person, "pose", context)
        _validate_vector(pose.get("position"), 3, f"{context}.pose.position")
        if not isinstance(pose.get("yaw"), (int, float)):
            raise ConfigError(f"{context}.pose.yaw must be a number")
        waypoints = person.get("waypoints", [])
        if not isinstance(waypoints, list):
            raise ConfigError(f"{context}.waypoints must be a list")
        for waypoint_index, waypoint in enumerate(waypoints):
            _validate_vector(waypoint, 3, f"{context}.waypoints[{waypoint_index}]")
    generation = normalized.get("generation")
    if generation is not None:
        if not isinstance(generation, dict):
            raise ConfigError("pedestrians.generation must be a mapping")
        if people:
            raise ConfigError("Use either pedestrians.people or generation, not both")
        mode = generation.get("mode")
        if mode not in ("fixed", "random"):
            raise ConfigError("pedestrians.generation.mode must be fixed or random")
        if not normalized["navmesh"].get("enabled", True):
            raise ConfigError("Pedestrian route generation requires NavMesh")
        if mode == "fixed":
            for name in ("start", "goal"):
                point = generation.get(name)
                _validate_vector(point, 3, f"pedestrians.generation.{name}")
                if not all(math.isfinite(value) for value in point):
                    raise ConfigError(f"pedestrians.generation.{name} must be finite")
            if generation["start"][:2] == generation["goal"][:2]:
                raise ConfigError("Pedestrian start and goal must differ")
        else:
            for name, default in (("count", 5), ("max_attempts_per_person", 500)):
                value = generation.get(name, default)
                if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
                    raise ConfigError(f"pedestrians.generation.{name} must be a positive integer")
            speeds = generation.get("speed_range_mps", [0.6, 1.4])
            _validate_vector(speeds, 2, "pedestrians.generation.speed_range_mps")
            if not all(math.isfinite(v) and v > 0 for v in speeds) or speeds[0] > speeds[1]:
                raise ConfigError("speed_range_mps must be positive, finite, and ordered")
            if 'target_density_per_m2' in generation:
                density=generation['target_density_per_m2']
                if isinstance(density,bool) or not isinstance(density,(int,float)) or not math.isfinite(density) or density<=0:
                    raise ConfigError('generation.target_density_per_m2 must be positive and finite')
            maximum=generation.get('max_count',30)
            if isinstance(maximum,bool) or not isinstance(maximum,int) or maximum<=0:
                raise ConfigError('generation.max_count must be a positive integer')
        for name, default in (("speed_mps", 1.0), ("radius", 0.3), ("height", 1.7),
                              ("min_route_length_m", 3.0), ("max_projection_m", 0.25),
                              ("min_spawn_separation_m", 1.0), ("min_robot_separation_m", 1.5)):
            value = generation.get(name, default)
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
                raise ConfigError(f"pedestrians.generation.{name} must be positive and finite")
        if "repeat" in generation:
            _validate_bool(generation["repeat"], "pedestrians.generation.repeat")
        characters = generation.get("characters", ["F_Business_02"])
        if not isinstance(characters, list) or not characters or any(not isinstance(v, str) or not v for v in characters):
            raise ConfigError("pedestrians.generation.characters must be a nonempty list of names")
        if not isinstance(generation.get("scene", {}), dict):
            raise ConfigError("pedestrians.generation.scene must be a mapping")
    return normalized


def validate_navigation_map(document: dict[str, Any]) -> dict[str, Any]:
    """Validate a reusable map descriptor.

    Asset paths are package-share relative, so the same YAML works from the
    source tree and after ``colcon build``.
    """

    normalized = copy.deepcopy(document)
    if normalized.get("schema_version") != 1:
        raise ConfigError("Only navigation map schema_version 1 is supported")
    _require_string(normalized, "name", "navigation_map")
    _require_string(normalized, "map_file", "navigation_map")
    return normalized


def validate_navigation_profile(document: dict[str, Any]) -> dict[str, Any]:
    """Validate robot-specific Nav2 and sensor-to-scan wiring."""

    normalized = copy.deepcopy(document)
    if normalized.get("schema_version") != 1:
        raise ConfigError("Only navigation profile schema_version 1 is supported")
    _require_string(normalized, "name", "navigation_profile")
    supported = normalized.get("supported")
    _validate_bool(supported, "navigation_profile.supported")
    if not supported:
        _require_string(normalized, "reason", "navigation_profile")
        return normalized

    nav2 = _require_mapping(normalized, "nav2", "navigation_profile")
    _require_string(nav2, "params_file", "navigation_profile.nav2")
    _require_string(nav2, "rviz_file", "navigation_profile.nav2")

    frames = _require_mapping(normalized, "frames", "navigation_profile")
    for key, value in frames.items():
        if not isinstance(key, str) or not key or not isinstance(value, str) or not value:
            raise ConfigError("navigation_profile.frames must map names to frame strings")

    topics = _require_mapping(normalized, "topics", "navigation_profile")
    for key, value in topics.items():
        if not isinstance(key, str) or not key or not isinstance(value, str) or not value:
            raise ConfigError("navigation_profile.topics must map names to topic strings")

    converter = _require_mapping(
        normalized,
        "pointcloud_to_laserscan",
        "navigation_profile",
    )
    _validate_bool(
        converter.get("enabled"),
        "navigation_profile.pointcloud_to_laserscan.enabled",
    )
    if converter["enabled"]:
        _require_string(
            converter,
            "cloud_topic",
            "navigation_profile.pointcloud_to_laserscan",
        )
        _require_string(
            converter,
            "scan_topic",
            "navigation_profile.pointcloud_to_laserscan",
        )
        _require_mapping(
            converter,
            "parameters",
            "navigation_profile.pointcloud_to_laserscan",
        )

    overrides = normalized.get("parameter_overrides", {})
    if not isinstance(overrides, dict):
        raise ConfigError("navigation_profile.parameter_overrides must be a mapping")
    normalized["parameter_overrides"] = overrides
    return normalized


def validate_scenario_config(document: dict[str, Any]) -> dict[str, Any]:
    normalized = copy.deepcopy(document)
    _require_sections(normalized, SCENARIO_SECTIONS, "scenario")
    if normalized["schema_version"] != 1:
        raise ConfigError("Only scenario schema_version 1 is supported")
    _require_string(normalized, "name", "scenario")
    _require_string(normalized, "simulation", "scenario")
    _require_string(normalized, "environment", "scenario")
    _require_string(normalized, "pedestrians", "scenario")

    navigation = _require_mapping(normalized, "navigation", "scenario")
    enabled = navigation.get("enabled")
    _validate_bool(enabled, "scenario.navigation.enabled")
    if enabled:
        _require_string(navigation, "map", "scenario.navigation")
    for setting in ("use_sim_time", "autostart", "use_rviz"):
        if setting in navigation:
            _validate_bool(navigation[setting], f"scenario.navigation.{setting}")
    if "startup_delay_sec" in navigation:
        _validate_nonnegative_number(
            navigation["startup_delay_sec"],
            "scenario.navigation.startup_delay_sec",
        )
    overrides = navigation.get("parameter_overrides", {})
    if not isinstance(overrides, dict):
        raise ConfigError("scenario.navigation.parameter_overrides must be a mapping")
    navigation["parameter_overrides"] = overrides

    robots = normalized["robots"]
    if not isinstance(robots, list) or not robots:
        raise ConfigError("scenario.robots must be a non-empty list")
    instance_names: set[str] = set()
    for index, instance in enumerate(robots):
        context = f"scenario.robots[{index}]"
        if not isinstance(instance, dict):
            raise ConfigError(f"{context} must be a mapping")
        _require_string(instance, "preset", context)
        instance_name = _require_string(instance, "instance_name", context)
        if "scan_profile" in instance:
            _require_string(instance, "scan_profile", context)
        namespace = instance.get("namespace", "")
        if not isinstance(namespace, str):
            raise ConfigError(f"{context}.namespace must be a string")
        _validate_pose(instance.get("pose"), f"{context}.pose")
        instance_navigation = instance.get("navigation", {})
        if not isinstance(instance_navigation, dict):
            raise ConfigError(f"{context}.navigation must be a mapping")
        instance_navigation_enabled = instance_navigation.get("enabled", enabled)
        _validate_bool(instance_navigation_enabled, f"{context}.navigation.enabled")
        if instance_navigation_enabled and not enabled:
            raise ConfigError(
                f"{context}.navigation cannot be enabled when scenario.navigation is disabled"
            )
        instance_navigation["enabled"] = instance_navigation_enabled
        if instance_navigation_enabled:
            _require_string(instance_navigation, "profile", f"{context}.navigation")
        initial_pose = instance_navigation.get("initial_pose")
        if initial_pose is not None:
            if not isinstance(initial_pose, dict):
                raise ConfigError(f"{context}.navigation.initial_pose must be a mapping")
            for coordinate in ("x", "y", "z", "yaw"):
                value = initial_pose.get(coordinate)
                if not isinstance(value, (int, float)):
                    raise ConfigError(
                        f"{context}.navigation.initial_pose.{coordinate} must be a number"
                    )
        instance_overrides = instance_navigation.get("parameter_overrides", {})
        if not isinstance(instance_overrides, dict):
            raise ConfigError(f"{context}.navigation.parameter_overrides must be a mapping")
        instance_navigation["parameter_overrides"] = instance_overrides
        instance["navigation"] = instance_navigation
        if instance_name in instance_names:
            raise ConfigError(f"Duplicate scenario robot instance: {instance_name}")
        instance_names.add(instance_name)
    return normalized


def _load_named(name_or_path, directory, validator, config_root):
    root = _config_root(config_root)
    path = _resolve_named_document(name_or_path, root / directory)
    return validator(_load_yaml(path))


def load_robot_config(name_or_path, config_root=None):
    return _load_named(name_or_path, "robots", validate_robot_config, config_root)


def load_sensor_preset(name_or_path, config_root=None):
    return _load_named(name_or_path, "sensors", validate_sensor_preset, config_root)


def load_environment_config(name_or_path, config_root=None):
    return _load_named(name_or_path, "environments", validate_environment_config, config_root)


def load_pedestrian_config(name_or_path, config_root=None):
    return _load_named(name_or_path, "pedestrians", validate_pedestrian_config, config_root)


def load_simulation_config(name_or_path, config_root=None):
    return _load_named(name_or_path, "simulation", validate_simulation_config, config_root)


def load_navigation_map(name_or_path, config_root=None):
    return _load_named(
        name_or_path,
        "navigation/maps",
        validate_navigation_map,
        config_root,
    )


def load_navigation_profile(name_or_path, config_root=None):
    return _load_named(
        name_or_path,
        "navigation/profiles",
        validate_navigation_profile,
        config_root,
    )


def load_scenario_config(name_or_path, config_root=None):
    return _load_named(name_or_path, "scenarios", validate_scenario_config, config_root)


def override_first_robot_pose(bundle, *, x="", y="", z="", yaw="") -> None:
    """Apply launch coordinates to the first robot before spawning or planning Nav2.

    Empty values preserve the scenario pose. Yaw is in radians; z is the
    physical spawn height, while Nav2 continues to use a planar z of zero.
    """
    values = {}
    for key, raw in (("x", x), ("y", y), ("z", z), ("yaw", yaw)):
        if raw is None or raw == "":
            continue
        try:
            value = float(raw)
        except (TypeError, ValueError) as error:
            raise ConfigError(f"robot_{key} must be a finite number") from error
        if not math.isfinite(value):
            raise ConfigError(f"robot_{key} must be a finite number")
        values[key] = value
    if not values:
        return

    instance = bundle["robots"][0]["instance"]
    position = instance["pose"]["position"]
    for index, key in enumerate(("x", "y", "z")):
        if key in values:
            position[index] = values[key]
    if "yaw" in values:
        half_yaw = values["yaw"] / 2.0
        instance["pose"]["orientation_quat_wxyz"] = [
            math.cos(half_yaw), 0.0, 0.0, math.sin(half_yaw)
        ]

    # An explicit localization pose must follow the same launch override.
    initial_pose = instance["navigation"].get("initial_pose")
    if initial_pose is not None:
        for key in ("x", "y", "yaw"):
            if key in values:
                initial_pose[key] = values[key]


def load_scenario_bundle(name_or_path, config_root=None) -> dict[str, Any]:
    """Load one scenario and validate every referenced reusable preset."""

    root = _config_root(config_root)
    scenario = load_scenario_config(name_or_path, root)
    robot_instances = []
    for instance in scenario["robots"]:
        navigation_profile = None
        if instance["navigation"]["enabled"]:
            navigation_profile = load_navigation_profile(
                instance["navigation"]["profile"],
                root,
            )
        robot_instances.append(
            {
                "instance": copy.deepcopy(instance),
                "preset": load_robot_config(instance["preset"], root),
                "navigation_profile": navigation_profile,
            }
        )
    navigation_map = None
    if scenario["navigation"]["enabled"]:
        navigation_map = load_navigation_map(scenario["navigation"]["map"], root)
    return {
        "scenario": scenario,
        "simulation": load_simulation_config(scenario["simulation"], root),
        "environment": load_environment_config(scenario["environment"], root),
        "pedestrians": load_pedestrian_config(scenario["pedestrians"], root),
        "navigation": {
            "settings": copy.deepcopy(scenario["navigation"]),
            "map": navigation_map,
        },
        "robots": robot_instances,
        "config_root": root,
    }
