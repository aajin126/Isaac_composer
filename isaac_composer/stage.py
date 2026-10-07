"""Small USD stage operations shared by environment and spawn providers."""

from __future__ import annotations

from typing import Any

from .assets import as_reference_url


def ensure_xform(stage: Any, prim_path: str):
    prim = stage.GetPrimAtPath(prim_path)
    if prim and prim.IsValid():
        return prim
    return stage.DefinePrim(prim_path, "Xform")


def add_usd_reference(stage: Any, prim_path: str, resolved_path: str):
    prim = ensure_xform(stage, prim_path)
    references = prim.GetReferences()
    references.ClearReferences()
    references.AddReference(as_reference_url(resolved_path))
    return prim


def set_xform_pose(stage: Any, prim_path: str, position, quaternion_wxyz) -> None:
    from pxr import Gf, UsdGeom

    prim = ensure_xform(stage, prim_path)
    xformable = UsdGeom.Xformable(prim)
    translate_op = None
    orient_op = None
    for operation in xformable.GetOrderedXformOps():
        if operation.GetOpName() == "xformOp:translate":
            translate_op = operation
        elif operation.GetOpName() == "xformOp:orient":
            orient_op = operation

    if translate_op is None:
        translate_op = xformable.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble)
    if orient_op is None:
        orient_op = xformable.AddOrientOp(UsdGeom.XformOp.PrecisionDouble)

    x, y, z = (float(value) for value in position)
    w, qx, qy, qz = (float(value) for value in quaternion_wxyz)
    translate_op.Set(Gf.Vec3d(x, y, z))
    orient_op.Set(Gf.Quatd(w, Gf.Vec3d(qx, qy, qz)))


def ensure_physics_scene(stage: Any, scene_path: str = "/World/PhysicsScene"):
    from pxr import Gf, PhysxSchema, UsdPhysics

    old_scene = stage.GetPrimAtPath("/physicsScene")
    if old_scene and old_scene.IsValid():
        stage.RemovePrim("/physicsScene")
    ensure_xform(stage, "/World")
    scene = UsdPhysics.Scene.Define(stage, scene_path)
    scene.CreateGravityDirectionAttr(Gf.Vec3f(0.0, 0.0, -1.0))
    scene.CreateGravityMagnitudeAttr(9.81)
    physx_scene = PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim())
    physx_scene.CreateEnableCCDAttr(True)
    physx_scene.CreateEnableStabilizationAttr(True)
    physx_scene.CreateEnableGPUDynamicsAttr(False)
    return scene.GetPrim()
