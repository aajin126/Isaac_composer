"""Kinematic pedestrian capsules, with only pedestrian pairs filtered out."""

from __future__ import annotations

import math


CAPSULE_ROOT = "/World/PedestrianColliders"
COLLISION_GROUP = "/World/PedestrianCollisionGroup"


class PedestrianCapsule:
    """A hidden physics body driven by the animated character's world position."""

    def __init__(self, stage, path, radius: float, height: float):
        from pxr import Gf, UsdGeom, UsdPhysics

        if not math.isfinite(radius) or radius <= 0:
            raise ValueError("Pedestrian capsule radius must be positive and finite")
        if not math.isfinite(height) or height < 2 * radius:
            raise ValueError("Pedestrian capsule height must be at least twice its radius")
        scale = UsdGeom.GetStageMetersPerUnit(stage)
        self._vec3 = Gf.Vec3d
        self._center_height = height / (2 * scale)
        self.path = path
        capsule = UsdGeom.Capsule.Define(stage, path)
        capsule.CreateAxisAttr(UsdGeom.Tokens.z)
        capsule.CreateRadiusAttr(radius / scale)
        # USD height excludes the two hemispherical caps.
        capsule.CreateHeightAttr((height - 2 * radius) / scale)
        capsule.CreateVisibilityAttr(UsdGeom.Tokens.invisible)
        self._translate = capsule.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble)
        prim = capsule.GetPrim()
        UsdPhysics.CollisionAPI.Apply(prim).CreateCollisionEnabledAttr(True)
        body = UsdPhysics.RigidBodyAPI.Apply(prim)
        body.CreateRigidBodyEnabledAttr(True)
        body.CreateKinematicEnabledAttr(True)

    def update(self, position) -> None:
        """Position is the character's foot origin, in stage units."""
        self._translate.Set(self._vec3(
            float(position[0]), float(position[1]), float(position[2]) + self._center_height
        ))


def add_pedestrian_capsules(stage, people, specs) -> None:
    """Keep robot contacts enabled; disable contacts inside the pedestrian group."""
    from pxr import UsdGeom, UsdPhysics

    if len(people) != len(specs):
        raise ValueError("Each pedestrian needs one capsule specification")
    if not people:
        return
    UsdGeom.Xform.Define(stage, CAPSULE_ROOT)
    group = UsdPhysics.CollisionGroup.Define(stage, COLLISION_GROUP)
    group.CreateFilteredGroupsRel().SetTargets([group.GetPath()])
    paths = []
    for index, (person, spec) in enumerate(zip(people, specs)):
        capsule = PedestrianCapsule(
            stage, f"{CAPSULE_ROOT}/Person_{index}",
            float(spec.get("radius", 0.3)), float(spec.get("height", 1.7)),
        )
        capsule.update(person.state.position)
        person._collision_capsule = capsule
        paths.append(capsule.path)
    collection = group.GetCollidersCollectionAPI()
    collection.CreateExpansionRuleAttr("explicitOnly")
    collection.CreateIncludesRel().SetTargets(paths)


def remove_pedestrian_capsules(stage, people) -> None:
    for person in people:
        person._collision_capsule = None
    for path in (COLLISION_GROUP, CAPSULE_ROOT):
        if stage.GetPrimAtPath(path):
            stage.RemovePrim(path)
