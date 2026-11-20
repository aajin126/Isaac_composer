"""Scenario-driven pedestrian creation migrated from the legacy runner."""

from __future__ import annotations

import random
from typing import Any

from .motion_events import scheduled_speed, validate_motion_events


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
) -> list[Any]:
    """Create configured animated people and register their world callbacks."""

    import numpy as np

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
            dynamic_avoidance_enabled=False,
            navmesh_enabled=bool(navmesh.get("enabled", True)),
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
        person._animation_walk_reference_mps = config.get("animation_walk_reference_mps", 1.0)
        waypoints = item.get("waypoints", [])
        if waypoints:
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

    from .pedestrian_collisions import add_pedestrian_capsules

    add_pedestrian_capsules(world.stage, spawned, people)
    return spawned


def shutdown_pedestrians(world: Any, people: list[Any]) -> None:
    """Unregister callbacks installed by migrated Person instances."""

    from isaac_composer.pedestrian.simulator.logic.people_manager import PeopleManager
    from .pedestrian_collisions import remove_pedestrian_capsules

    remove_pedestrian_capsules(world.stage, people)

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
