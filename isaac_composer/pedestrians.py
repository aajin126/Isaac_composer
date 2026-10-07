"""Scenario-driven pedestrian creation migrated from the legacy runner."""

from __future__ import annotations

import random
from typing import Any


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


def build_pedestrians(world: Any, config: dict[str, Any]) -> list[Any]:
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
    manager = PeopleManager.get_people_manager()
    manager.rebuild_nav_mesh(
        height=float(navmesh.get("agent_height", 1.8)),
        radius=float(navmesh.get("agent_radius", 0.4)),
        auto_rebake_on_changes=bool(
            navmesh.get("auto_rebake_on_changes", False)
        ),
        auto_rebake_delay_seconds=int(
            navmesh.get("auto_rebake_delay_seconds", 4)
        ),
        exclude_rigid_bodies=bool(navmesh.get("exclude_rigid_bodies", False)),
        view_nav_mesh=bool(navmesh.get("view", False)),
        dynamic_avoidance_enabled=bool(
            navmesh.get("dynamic_avoidance", True)
        ),
        navmesh_enabled=bool(navmesh.get("enabled", True)),
    )

    stage = omni.usd.get_context().get_stage()
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
        if waypoints:
            person.update_target_position(
                waypoints,
                walk_speed=float(item.get("walk_speed", 1.0)),
            )
        spawned.append(person)
    return spawned


def shutdown_pedestrians(world: Any, people: list[Any]) -> None:
    """Unregister callbacks installed by migrated Person instances."""

    from isaac_composer.pedestrian.simulator.logic.people_manager import PeopleManager

    for person in people:
        prefix = person._stage_prefix
        for callback_name in (f"{prefix}/state", f"{prefix}/update"):
            try:
                world.remove_physics_callback(callback_name)
            except Exception:
                pass
        try:
            world.remove_timeline_callback(f"{prefix}/start_stop_sim")
        except Exception:
            pass
    PeopleManager.get_people_manager().remove_all_people()
