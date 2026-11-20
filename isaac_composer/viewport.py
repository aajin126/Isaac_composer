"""Keep the Isaac Sim viewport centered on a robot during teleoperation."""

from __future__ import annotations

import math
from typing import Any


class RobotViewport:
    """Move a viewing camera at the rendering rate, independently of sensors."""

    def __init__(self, robot: Any, config: dict[str, Any]) -> None:
        import omni.usd
        from omni.kit.viewport.utility import get_active_viewport
        from pxr import Gf, Usd, UsdGeom

        self.mode = config.get("mode", "follow")
        self.viewport = get_active_viewport()
        self.last_update: float | None = None
        self.distance = float(config.get("distance_m", 6.0))
        self.height = float(config.get("height_m", 5.0))
        self.look_ahead = float(config.get("look_ahead_m", 2.0))
        self.top_down_height = float(config.get("top_down_height_m", 12.0))
        self.smoothing = float(config.get("smoothing_sec", 0.2))
        self.heading_smoothing = float(config.get("heading_smoothing_sec", 0.5))
        self.height_smoothing = float(config.get("height_smoothing_sec", 1.0))
        self.follow_cutaway = bool(config.get("follow_cutaway", True))
        self.cutaway_margin = float(config.get("follow_cutaway_margin_m", 0.7))
        self.anchor = None
        self.heading = None
        self.eye = None
        self.target = None
        self.clipping_mode = None
        self.clipping_near = None
        self.Gf, self.Usd, self.UsdGeom = Gf, Usd, UsdGeom
        self.stage = omni.usd.get_context().get_stage()
        self.meters_per_unit = UsdGeom.GetStageMetersPerUnit(self.stage)
        base = robot.links.get("base")
        self.robot_prim = self.stage.GetPrimAtPath(
            base.prim_path if base is not None else robot.root_prim_path
        )
        if not self.robot_prim.IsValid():
            raise ValueError(f"Viewport robot prim does not exist: {robot.root_prim_path}")
        self.camera = UsdGeom.Camera.Define(self.stage, "/World/RobotViewCamera")
        self.camera.CreateClippingRangeAttr().Set(Gf.Vec2f(
            0.1 / self.meters_per_unit, 1000.0 / self.meters_per_unit
        ))
        self.camera.CreateFocalLengthAttr().Set(18.0)
        self.transform = self.camera.MakeMatrixXform()
        if self.viewport is not None:
            self.viewport.camera_path = self.camera.GetPath()

    def update(self, simulation_time: float, *, force: bool = False) -> None:
        # The runner calls this once per rendered frame. A second dt gate would
        # skip alternate frames when physics quantizes 1/30 s to 0.032 s.
        Gf, Usd, UsdGeom = self.Gf, self.Usd, self.UsdGeom
        pose = UsdGeom.XformCache(Usd.TimeCode.Default()).GetLocalToWorldTransform(self.robot_prim)
        position = pose.ExtractTranslation()
        forward = pose.TransformDir(Gf.Vec3d(1, 0, 0))
        yaw = math.atan2(forward[1], forward[0])
        if self.anchor is not None and not force:
            dt = max(0.0, simulation_time - self.last_update)
            xy_alpha = 1.0 - math.exp(-dt / self.smoothing) if self.smoothing > 0 else 1.0
            z_alpha = 1.0 - math.exp(-dt / self.height_smoothing) if self.height_smoothing > 0 else 1.0
            yaw_alpha = 1.0 - math.exp(-dt / self.heading_smoothing) if self.heading_smoothing > 0 else 1.0
            position = Gf.Vec3d(
                self.anchor[0] + xy_alpha * (position[0] - self.anchor[0]),
                self.anchor[1] + xy_alpha * (position[1] - self.anchor[1]),
                self.anchor[2] + z_alpha * (position[2] - self.anchor[2]),
            )
            # Blend heading along the shortest turn, including the +/- pi seam.
            yaw_delta = math.atan2(math.sin(yaw - self.heading), math.cos(yaw - self.heading))
            yaw = self.heading + yaw_alpha * yaw_delta
            yaw = math.atan2(math.sin(yaw), math.cos(yaw))
        self.anchor, self.heading = position, yaw
        forward = Gf.Vec3d(math.cos(yaw), math.sin(yaw), 0)
        scale = self.meters_per_unit
        if self.mode == "top_down":
            eye = position + Gf.Vec3d(0, 0, self.top_down_height / scale)
            target = position
            up = Gf.Vec3d(0, 1, 0)
        else:
            eye = position - forward * (self.distance / scale) + Gf.Vec3d(0, 0, self.height / scale)
            target = position + forward * (self.look_ahead / scale)
            up = Gf.Vec3d(0, 0, 1)
        # Clip foreground walls/roofs in this viewing camera only. Keep a margin
        # before the robot's center so its body remains inside the visible range.
        if self.mode == "top_down":
            near = max(0.1, self.top_down_height - 2.0)
        elif self.follow_cutaway:
            view_direction = (target - eye).GetNormalized()
            near = max(0.1, Gf.Dot(position - eye, view_direction) * scale - self.cutaway_margin)
        else:
            near = 0.1
        if self.clipping_mode != self.mode or self.clipping_near is None or abs(near - self.clipping_near) > 1e-5:
            self.camera.GetClippingRangeAttr().Set(Gf.Vec2f(near / scale, 1000.0 / scale))
            self.clipping_mode = self.mode
            self.clipping_near = near
        self.eye, self.target = eye, target
        self.transform.Set(Gf.Matrix4d().SetLookAt(eye, target, up).GetInverse())
        self.last_update = simulation_time
