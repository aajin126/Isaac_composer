"""Scenario-driven pedestrian creation migrated from the legacy runner."""

from __future__ import annotations

import random
from time import perf_counter
from typing import Any

from .motion_events import scheduled_speed, validate_motion_events


def _find_first_skelroot_path(stage: Any, root_path: str) -> str | None:
    from pxr import Usd, UsdSkel

    root = stage.GetPrimAtPath(root_path)
    for prim in Usd.PrimRange(root):
        if prim.IsA(UsdSkel.Root):
            return str(prim.GetPath())
    for prim in Usd.PrimRange(root):
        name = prim.GetName().lower()
        if "manroot" in name or "skelroot" in name:
            return str(prim.GetPath())
    return None


def _add_collider(stage: Any, parent_path: str, radius: float, height: float) -> None:
    from pxr import Gf, UsdGeom, UsdPhysics

    capsule = UsdGeom.Capsule.Define(stage, f"{parent_path}/CollisionCapsule")
    capsule.CreateRadiusAttr(float(radius))
    capsule.CreateHeightAttr(float(height))
    capsule.CreateAxisAttr("Z")
    xform = UsdGeom.Xformable(capsule.GetPrim())
    xform.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, float(height) * 0.5))
    UsdPhysics.CollisionAPI.Apply(capsule.GetPrim())
    capsule.GetPrim().GetAttribute("visibility").Set("invisible")


def _install_motion_schedule(
    world: Any,
    person: Any,
    waypoints: list[list[float]],
    walk_speed: float,
    start_delay: float,
    repeat: bool,
    events: list[dict[str, Any]] | None = None,
) -> None:
    """Add delayed start and optional ping-pong repetition around the existing Person API."""
    import numpy as np

    route = [list(map(float, point)) for point in waypoints]
    if not route:
        return
    # Include the spawn point so every return leg reaches the original start.
    start = list(map(float, person.state.position))
    if np.linalg.norm(np.asarray(start) - np.asarray(route[0])) > 1e-6:
        route.insert(0, start)
    motion_events = validate_motion_events(events or [])

    state = {
        "elapsed": 0.0,
        "started": False,
        "route": route,
        "cooldown": 0.0,
        "finished": False,
    }
    callback_name = f"{person._stage_prefix}/scenario_schedule"

    def reset_target(points: list[list[float]]) -> None:
        person._current_point_index = 0
        person.update_target_position(points, walk_speed=walk_speed)

    def update_schedule(dt: float) -> None:
        state["elapsed"] += float(dt)
        state["cooldown"] = max(0.0, state["cooldown"] - float(dt))
        if not state["started"]:
            if state["elapsed"] >= start_delay:
                reset_target(state["route"])
                state["started"] = True
            else:
                person._target_speed = 0.0
                return
        if state["finished"]:
            person._target_speed = 0.0
            return

        person._target_speed = scheduled_speed(
            motion_events, state["elapsed"] - start_delay, walk_speed
        )
        if person._target_speed == 0.0:
            return

        position = np.asarray(person.state.position, dtype=float)
        final_point = np.asarray(state["route"][-1], dtype=float)
        if np.linalg.norm(position[:2] - final_point[:2]) > 0.3 or state["cooldown"] > 0.0:
            return

        if repeat and len(state["route"]) >= 2:
            state["route"] = list(reversed(state["route"]))
            reset_target(state["route"])
            person._target_speed = scheduled_speed(
                motion_events, state["elapsed"] - start_delay, walk_speed
            )
            state["cooldown"] = 0.75
        else:
            state["finished"] = True
            person._target_speed = 0.0

    world.add_physics_callback(callback_name, update_schedule)
    callbacks = getattr(person, "_scenario_callback_names", [])
    callbacks.append(callback_name)
    person._scenario_callback_names = callbacks


def build_pedestrians(
    world: Any,
    config: dict[str, Any],
    navmesh_prepared: bool = False,
    nav_scene: Any = None,
    robots=(),
) -> list[Any]:
    """Create configured animated people and register their world callbacks."""

    import numpy as np
    import omni.usd

    from isaac_composer.pedestrian.simulator.logic.people.person import Person
    from isaac_composer.pedestrian.simulator.logic.people_manager import PeopleManager

    people = config.get("people", [])
    if not people:
        return []

    seed = int(config.get("seed", 42))
    random.seed(seed)
    np.random.seed(seed)
    navmesh = config.get("navmesh", {})
    if not navmesh_prepared:
        manager = PeopleManager.get_people_manager()
        manager.rebuild_nav_mesh(
            height=float(navmesh.get("agent_height", 1.8)),
            radius=float(navmesh.get("agent_radius", 0.4)),
            auto_rebake_on_changes=bool(navmesh.get("auto_rebake_on_changes", False)),
            auto_rebake_delay_seconds=int(navmesh.get("auto_rebake_delay_seconds", 4)),
            exclude_rigid_bodies=bool(navmesh.get("exclude_rigid_bodies", False)),
            view_nav_mesh=bool(navmesh.get("view", False)),
            dynamic_avoidance_enabled=bool(navmesh.get("dynamic_avoidance", True)),
            navmesh_enabled=bool(navmesh.get("enabled", True)),
        )

    stage = omni.usd.get_context().get_stage()
    from .crowd_safety import CrowdNavigator, yielding_velocities

    # Fail before creating visible agents if a manually supplied scenario overlaps.
    for i, item in enumerate(people):
        for other in people[:i]:
            distance = np.linalg.norm(
                np.asarray(item["pose"]["position"])[:2] - np.asarray(other["pose"]["position"])[:2]
            )
            required = float(item.get("radius", 0.25)) + float(other.get("radius", 0.25)) + 0.20
            if distance < required:
                raise ValueError(
                    "Pedestrian spawn positions overlap; separate them before starting"
                )
    spawned = []
    for index, item in enumerate(people):
        pose = item["pose"]
        person = Person(
            world=world,
            stage_prefix=item.get("name", f"Character_{index}"),
            character_name=item["character"],
            init_pos=pose["position"],
            init_yaw=float(pose["yaw"]),
            controller=None,
            backend=None,
        )
        skelroot_path = _find_first_skelroot_path(stage, person._stage_prefix)
        if skelroot_path is not None:
            _add_collider(
                stage,
                skelroot_path,
                float(item.get("radius", 0.25)),
                float(item.get("height", 1.6)),
            )
        waypoints = item.get("waypoints", [])
        if item.get("planner_source") == "cosmos":
            person._target_speed = float(item.get("walk_speed", 1.0))
        elif waypoints:
            _install_motion_schedule(
                world=world,
                person=person,
                waypoints=waypoints,
                walk_speed=float(item.get("walk_speed", 1.0)),
                start_delay=float(item.get("start_delay", 0.0)),
                repeat=bool(item.get("repeat", False)),
                events=item.get("events", []),
            )
        spawned.append(person)

    activity = None
    if config.get("semantic_execution"):
        from .semantic_scenario.runtime.compiler import animation_query_scene
        from .semantic_scenario.runtime.executor import SceneExecutor

        nav_scene = animation_query_scene(
            nav_scene, config["social_navigation"]["tracking_margin_m"]
        )
        activity = SceneExecutor(
            nav_scene,
            spawned,
            config["semantic_execution"],
            config.get("state_output_dir"),
        )
        spawned[0]._semantic_executor = activity

    backend = None
    if nav_scene is not None:
        from .crowd.runtime.isaac import CrowdRuntime

        backend = CrowdRuntime(world, nav_scene, spawned, config, robots)
        spawned[0]._crowd_runtime = backend
    navigator = (
        backend.social
        if backend and backend.social
        else (CrowdNavigator(nav_scene) if nav_scene is not None else None)
    )
    if navigator is not None:
        spawned[0]._crowd_navigator = navigator
        # The crowd controller writes the final animation command itself.
        # Avoid reading/writing every character again at the 500 Hz robot rate.
        for person in spawned:
            world.remove_physics_callback(f"{person._stage_prefix}/state")
            world.remove_physics_callback(f"{person._stage_prefix}/update")

    control_elapsed = 0.0

    def guard_crowd(dt: float) -> None:
        # Registered last, after route schedules. Generated crowds own the final
        # animation command at 50 Hz; legacy people retain their own callbacks.
        nonlocal control_elapsed
        if navigator is not None:
            control_elapsed += float(dt)
            if control_elapsed + 1e-9 < 0.02:
                return
            dt, control_elapsed = control_elapsed, 0.0
        import carb

        tick_start = perf_counter()
        if activity:
            for person in spawned:
                person.update_state(dt)
            activity.step(dt, [person.state.position for person in spawned])

        velocities = []
        for person in spawned:
            if not activity:
                person.update_state(dt)
            if person._num_path_points:
                target = person._target_position[person._current_point_index]
            else:
                target = person.state.position
            delta = np.asarray(target)[:2] - person.state.position[:2]
            length = np.linalg.norm(delta)
            velocities.append(
                delta / length * max(0.0, person._target_speed) if length > 1e-6 else np.zeros(2)
            )
        positions = [person.state.position for person in spawned]
        radii = [float(item.get("radius", 0.25)) for item in people]
        paths = [
            person._target_position if person._num_path_points else [person.state.position]
            for person in spawned
        ]
        observation = backend.observe(positions) if backend else None
        observed_at = perf_counter()
        if backend and backend.social:
            stamp, actual, robot_states, robot_poses = observation
            vectors, preferred = navigator.step(
                dt,
                positions,
                actual,
                paths,
                [max(0.0, p._target_speed) for p in spawned],
                radii,
                robot_states,
                headings=(
                    backend.body_headings()
                    if navigator.config["directional_responsibility_enabled"]
                    else None
                ),
            )
            navigation_at = perf_counter()
            # Keep the animation's preview on native NavMesh, including corners;
            # a long straight target can cross a wall despite a safe short step.
            vectors, paths = backend.animation_commands(dt, positions, vectors)
            commands = backend.animation.walk
        elif navigator is not None:
            commands, paths = navigator.step(
                dt, positions, paths, [max(0.0, person._target_speed) for person in spawned], radii
            )
        else:
            safe = yielding_velocities(positions, velocities, radii)
            commands = np.linalg.norm(safe, axis=1)
        animation_at = perf_counter()
        for index, (person, speed, path) in enumerate(zip(spawned, commands, paths)):
            person._animation_playback_rate = (
                float(backend.animation.playback_rate[index]) if backend and backend.social else 1.0
            )
            if navigator is not None:
                person._crowd_state = navigator.states[index]
            if person.character_graph:
                person.character_graph.set_variable("Walk", float(speed))
                look = bool(activity and activity.action(index) == "LookAround" and speed <= 0.02)
                was_look = getattr(person, "_semantic_lookaround", False)
                if look != was_look:
                    person.character_graph.set_variable("lookaround", 1.0 if look else 0.0)
                    person._semantic_lookaround = look
                if look:
                    if not was_look:
                        person.character_graph.set_variable("Action", "None")
                        person.character_graph.set_variable(
                            "PathPoints", [carb.Float3(person.state.position)]
                        )
                    continue
                moving = (
                    bool(backend.animation.walking[index])
                    if backend and backend.social
                    else speed > 0
                )
                social_animation = backend.animation if backend and backend.social else None
                if social_animation is None or social_animation.action_changed[index] or was_look:
                    action_moving = (
                        social_animation.action_walking[index]
                        if social_animation is not None
                        else moving
                    )
                    person.character_graph.set_variable(
                        "Action", "Walk" if action_moving else "Idle"
                    )
                if (social_animation is not None and social_animation.path_changed[index]) or (
                    social_animation is None and moving
                ):
                    person.character_graph.set_variable(
                        "PathPoints", [carb.Float3(point) for point in path]
                    )
        if backend:
            stamp, actual, robot_states, robot_poses = observation
            if not backend.social:
                vectors = np.asarray(velocities)
                preferred = vectors
            emission_at = perf_counter()
            backend.emit(
                stamp, positions, actual, robot_states, robot_poses, vectors, preferred, navigator
            )
            ended_at = perf_counter()
            backend.timings.record(
                observation=observed_at - tick_start,
                navigation=(navigation_at - observed_at) if backend.social else 0.0,
                animation=(animation_at - navigation_at) if backend.social else 0.0,
                graph_commands=emission_at - animation_at,
                state_output=ended_at - emission_at,
                total=ended_at - tick_start,
            )

    callback = f"{spawned[0]._stage_prefix}/crowd_safety"
    world.add_physics_callback(callback, guard_crowd)
    callbacks = getattr(spawned[0], "_scenario_callback_names", [])
    callbacks.append(callback)
    spawned[0]._scenario_callback_names = callbacks
    return spawned


def shutdown_pedestrians(world: Any, people: list[Any]) -> None:
    """Unregister callbacks installed by migrated Person instances."""

    from isaac_composer.pedestrian.simulator.logic.people_manager import PeopleManager

    if people and getattr(people[0], "_semantic_executor", None):
        people[0]._semantic_executor.close()
    if people and getattr(people[0], "_crowd_runtime", None) is not None:
        people[0]._crowd_runtime.shutdown()

    for person in people:
        prefix = person._stage_prefix
        for callback_name in getattr(person, "_scenario_callback_names", []):
            try:
                world.remove_physics_callback(callback_name)
            except Exception:
                pass
        for callback_name in (f"{prefix}/state", f"{prefix}/update"):
            try:
                if world.physics_callback_exists(callback_name):
                    world.remove_physics_callback(callback_name)
            except Exception:
                pass
        try:
            world.remove_timeline_callback(f"{prefix}/start_stop_sim")
        except Exception:
            pass
    PeopleManager.get_people_manager().remove_all_people()
