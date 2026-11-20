"""Isaac animation adapter and timestamped world-state recording/publishing."""

import json
import math
import numpy as np
from ..state import MeasuredVelocities, PEDESTRIAN_FIELDS, STATE_CODES
from ..context.walkable import WalkableContext
from ..navigation.controller import SocialNavigator
from .recording import StateRecorder
from .publisher import ROSStatePublisher
from .animation import AnimationCommands
from ..behavior.responsibility import character_body_yaws
from ...performance import TimingStats


class CrowdRuntime:
    def __init__(self, world, scene, people, config, robots):
        import omni.usd
        from pxr import Usd, UsdGeom, Gf

        self.world, self.people, self.config, self.robots = world, people, config, robots
        self.Usd, self.UsdGeom, self.Gf = Usd, UsdGeom, Gf
        self.stage = omni.usd.get_context().get_stage()
        self.scale = UsdGeom.GetStageMetersPerUnit(self.stage)
        self.robot_prims = [
            self.stage.GetPrimAtPath(r.require_link("base").prim_path) for r in robots
        ]
        self.measured = MeasuredVelocities()
        self.context = WalkableContext(scene, getattr(scene, "social_raster", None))
        self.social = (
            SocialNavigator(
                scene,
                self.context,
                config["people"],
                config["social_navigation"],
                config.get("seed", 42),
            )
            if config["social_navigation"]["enabled"]
            else None
        )
        self.latest = None
        self.timings = TimingStats()
        self.animation = AnimationCommands(config["people"], config["social_navigation"])
        self.orca_velocity = None
        self.recorder = StateRecorder(config)
        self.publisher = ROSStatePublisher(config["social_navigation"]["state_topic"])

    def body_headings(self):
        return character_body_yaws([person.state.orientation for person in self.people])

    def observe(self, positions):
        cache = self.UsdGeom.XformCache(self.Usd.TimeCode.Default())
        poses = []
        for prim in self.robot_prims:
            transform = cache.GetLocalToWorldTransform(prim)
            p = transform.ExtractTranslation()
            forward = transform.TransformDir(self.Gf.Vec3d(1, 0, 0))
            poses.append([p[0] * self.scale, p[1] * self.scale, math.atan2(forward[1], forward[0])])
        stamp = float(self.world.current_time)
        xy = [list(p[:2]) for p in positions] + [p[:2] for p in poses]
        velocity = self.measured.update(stamp, xy)
        radius = self.config["social_navigation"]["robot_radius_m"]
        robots = [
            (np.array([p[0], p[1], 0]), v, radius)
            for p, v in zip(poses, velocity[len(positions) :])
        ]
        return stamp, velocity[: len(positions)], robots, poses

    def emit(self, stamp, positions, velocities, robots, poses, commands, preferred, navigator):
        people = self.config["people"]
        rows = []
        for i, (p, v, item) in enumerate(zip(positions, velocities, people)):
            density, area, width = (
                navigator.local_context[i]
                if self.social
                else self.context.local(
                    p, np.asarray(positions), self.config["social_navigation"]["density_radius_m"]
                )
            )
            state = navigator.states[i] if self.social else "walking"
            rows.append(
                [
                    item.get("id", i),
                    float(p[0]),
                    float(p[1]),
                    float(v[0]),
                    float(v[1]),
                    item.get("mu", 0.5),
                    density,
                    area,
                    width,
                    STATE_CODES.get(state, 0),
                    navigator.yield_to[i] if self.social else -999,
                ]
            )
        packet = {
            "stamp_ns": round(stamp * 1e9),
            "frame_id": "world",
            "pedestrian_fields": PEDESTRIAN_FIELDS,
            "pedestrians": rows,
            "commanded_velocity": np.asarray(commands).tolist(),
            "preferred_velocity": np.asarray(preferred).tolist(),
            "robots": [
                {"id": -i - 1, "pose": p, "velocity": v.tolist(), "radius": radius}
                for i, (p, (_, v, radius)) in enumerate(zip(poses, robots))
            ],
            "feasible": navigator.last_feasible if self.social else [True] * len(rows),
            "progress": navigator.progress.diagnostics() if self.social else None,
            "orca_velocity": self.orca_velocity,
            "social_stiffness_enabled": self.social.stiffness.enabled if self.social else False,
            "directional_responsibility": (
                self.social.responsibility.diagnostics() if self.social else None
            ),
            "deadlock_recovery_enabled": (
                self.config["social_navigation"]["yielding_enabled"] if self.social else False
            ),
            "animation": self.animation.diagnostics() if self.social else None,
        }
        executor = getattr(self.people[0], "_semantic_executor", None) if self.people else None
        if executor is not None:
            packet["semantic_scenario"] = executor.diagnostics()
            packet["planner_sources"] = [p.get("planner_source", "random") for p in people]
        self.latest = packet
        data = json.dumps(packet, allow_nan=False, separators=(",", ":"))
        self.recorder.write(packet, serialized=data)
        self.publisher.publish(data)

    def animation_commands(self, dt, positions, vectors):
        self.orca_velocity = np.asarray(vectors).tolist()
        applied, paths, feasible = self.animation.update(
            dt, positions, vectors, self.context.scene, self.social.solver
        )
        for i, ok in enumerate(feasible):
            if not ok:
                self.social.states[i] = "infeasible"
                if self.social.last_feasible[i]:
                    self.social.infeasible_count += 1
                self.social.last_feasible[i] = False
        return applied, paths

    def diagnostics(self):
        return {
            **self.social.diagnostics(),
            "animation": self.animation.diagnostics(),
            "cpu_timings": self.timings.summary(),
        }

    def shutdown(self):
        self.recorder.close()
        self.publisher.close()
        if self.social:
            self.social.solver.close()
