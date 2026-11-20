"""Sample reachable teleoperation tasks and monitor arrival at a fixed goal."""

from __future__ import annotations

import copy
import json
import math
import random
from pathlib import Path

import numpy as np

from .robot_task_config import GOAL_FRAME, TASK_DEFAULTS
from .goal_route import robot_navigation_scene


def rotation_2d(yaw):
    c, s = math.cos(float(yaw)), math.sin(float(yaw))
    return np.array([[c, -s], [s, c]])


def calibrate_world_from_odom(world_pose, odom_pose):
    """Infer the actual planar frame transform from simultaneous base poses."""
    yaw = float(world_pose[2] - odom_pose[2])
    translation = np.asarray(world_pose[:2]) - rotation_2d(yaw) @ np.asarray(odom_pose[:2])
    return [float(translation[0]), float(translation[1]), yaw]


def world_point_to_odom(point, world_from_odom):
    return rotation_2d(-world_from_odom[2]) @ (
        np.asarray(point[:2]) - np.asarray(world_from_odom[:2])
    )


class ArrivalMonitor:
    """Require proximity and a sustained stop; passing through is insufficient."""

    def __init__(self, goal, config):
        self.goal = np.asarray(goal[:2], dtype=float)
        self.config = config
        self.since = None
        self.reached = False

    def update(self, simulation_time, position, velocity, yaw_rate):
        inside = (
            np.linalg.norm(np.asarray(position[:2]) - self.goal) <= self.config["goal_radius_m"]
        )
        stopped = (
            np.linalg.norm(np.asarray(velocity[:2])) <= self.config["stopped_speed_mps"]
            and abs(float(yaw_rate)) <= self.config["stopped_yaw_rate_rps"]
        )
        if inside and stopped:
            if self.since is None:
                self.since = float(simulation_time)
            self.reached |= float(simulation_time) - self.since >= self.config["arrival_hold_sec"]
        else:
            self.since = None
        return self.reached


class RobotTaskPlanner:
    def __init__(
        self, scene, config, seed, excluded_starts=(), min_separation=1.5, path_scene=None
    ):
        from scipy import ndimage

        self.scene = robot_navigation_scene(scene, config)
        self.path_scene = path_scene if path_scene is not None else self.scene
        self.config = config
        self.seed = int(seed)
        self.random = random.Random(seed)
        self.raster = scene.rasterize(margin_m=0)
        extra = max(0.0, config["robot_radius_m"] - getattr(scene, "agent_radius", 0.0))
        distance = ndimage.distance_transform_edt(np.pad(self.raster.mask, 1))[1:-1, 1:-1]
        self.mask = self.raster.mask & (distance * self.raster.resolution_m > extra)
        self.candidates = np.argwhere(self.mask)
        self.excluded = list(excluded_starts)
        self.min_separation = min_separation

    def _allowed(self, point):
        if self.path_scene is not self.scene:
            return self.path_scene.is_allowed_world(point)
        bounds = self.raster.bounds
        row = int((bounds.y_max - point[1]) / (bounds.y_max - bounds.y_min) * self.raster.height)
        col = int((point[0] - bounds.x_min) / (bounds.x_max - bounds.x_min) * self.raster.width)
        if not (
            0 <= row < self.raster.height and 0 <= col < self.raster.width and self.mask[row, col]
        ):
            return False
        return not hasattr(self.scene, "is_allowed_world") or self.scene.is_allowed_world(point)

    def _sample(self):
        if not len(self.candidates):
            raise RuntimeError("No robot-clearance free space is available")
        row, col = self.candidates[self.random.randrange(len(self.candidates))]
        raw = self.raster.pixel_to_world(int(row), int(col))
        projected = list(map(float, self.scene.project_world(raw)))
        if math.dist(raw[:2], projected[:2]) > 0.15 or not self._allowed(projected):
            raise RuntimeError("Robot task point cannot be projected safely")
        return projected

    def plan(self):
        last_error = None
        for _ in range(self.config["max_attempts"]):
            try:
                start, goal = self._sample(), self._sample()
                distance = math.dist(start[:2], goal[:2])
                if (
                    not self.config["min_goal_distance_m"]
                    <= distance
                    <= self.config["max_goal_distance_m"]
                ):
                    continue
                if any(math.dist(start[:2], p[:2]) < self.min_separation for p in self.excluded):
                    continue
                path = [list(map(float, p)) for p in self.path_scene.shortest_path(start, goal)]
                if (
                    len(path) < 2
                    or math.dist(path[0][:2], start[:2]) > 0.05
                    or math.dist(path[-1][:2], goal[:2]) > 0.05
                ):
                    raise RuntimeError("Robot path does not reach the requested endpoints")
                for a, b in zip(path[:-1], path[1:]):
                    for point in np.linspace(
                        a, b, max(2, math.ceil(math.dist(a, b) / self.raster.resolution_m) + 1)
                    ):
                        if not self._allowed(point):
                            raise RuntimeError("Robot path violates footprint clearance")
                yaw = self.random.uniform(-math.pi, math.pi)
                spawn = [start[0], start[1], start[2] + self.config["spawn_height_m"]]
                return {
                    "seed": self.seed,
                    "spawn_world": spawn,
                    "spawn_yaw": yaw,
                    "spawn_velocity": [0.0, 0.0, 0.0],
                    "goal_world": goal,
                    "reference_path_world": path,
                    "reference_path_source": getattr(
                        self.path_scene, "path_source", "native_navmesh"
                    ),
                    "straight_distance_m": distance,
                    "path_length_m": sum(math.dist(a, b) for a, b in zip(path[:-1], path[1:])),
                    "config": dict(self.config),
                    "status": "planned",
                }
            except RuntimeError as error:
                last_error = error
        raise RuntimeError(
            f'No reachable robot start/goal pair after {self.config["max_attempts"]} attempts: {last_error}'
        )


def prepare_robot_task(bundle, scene, output, excluded_starts=(), planned_task=None):
    config = bundle["robot_task"]
    if not config["enabled"]:
        return None
    if len(bundle["robots"]) != 1:
        raise ValueError("Goal-directed collection requires exactly one robot")
    generation = bundle["pedestrians"].get("generation", {})
    excluded = [p["pose"]["position"] for p in bundle["pedestrians"].get("people", [])]
    excluded.extend(excluded_starts)
    if generation.get("mode") == "fixed":
        excluded.append(generation["start"])
    seed = int(bundle["pedestrians"].get("seed", 42))
    from .static_map_path import configured_static_map

    map_scene = configured_static_map(bundle, scene)
    if map_scene is not None:
        bundle["robot_navigation_scene"] = map_scene
    planner = RobotTaskPlanner(
        scene,
        config,
        seed,
        excluded,
        float(generation.get("min_robot_separation_m", 1.5)),
        path_scene=map_scene,
    )
    if planned_task is None:
        task = planner.plan()
    else:
        task = copy.deepcopy(planned_task)
        for key in ("robot_radius_m", "clearance_m", "spawn_height_m"):
            if task["config"][key] != config[key]:
                raise ValueError(
                    "Reviewed robot task footprint does not match the current configuration"
                )
        path = task["reference_path_world"]
        if (
            not path
            or math.dist(path[0][:2], task["spawn_world"][:2]) > 0.05
            or math.dist(path[-1][:2], task["goal_world"][:2]) > 0.05
        ):
            raise ValueError("Reviewed robot task endpoints do not match its reference path")
        if map_scene is not None:
            task["reference_path_world"] = map_scene.shortest_path(
                [*task["spawn_world"][:2], task["goal_world"][2]], task["goal_world"]
            )
            task["reference_path_source"] = map_scene.path_source
            task["path_length_m"] = sum(
                math.dist(a, b)
                for a, b in zip(task["reference_path_world"], task["reference_path_world"][1:])
            )
        path = task["reference_path_world"]
        for a, b in zip(path[:-1], path[1:]):
            for point in np.linspace(
                a, b, max(2, math.ceil(math.dist(a, b) / planner.raster.resolution_m) + 1)
            ):
                if not planner._allowed(point):
                    raise ValueError("Reviewed robot task violates the current footprint clearance")
    if map_scene is not None:
        task["path_map_file"] = str(map_scene.path)
    robot = bundle["robots"][0]["instance"]
    yaw = task["spawn_yaw"]
    robot["pose"] = {
        "position": task["spawn_world"],
        "orientation_quat_wxyz": [math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)],
    }
    task["robot_name"] = robot["instance_name"]
    task["odom_frame_id"] = bundle["robots"][0]["preset"]["ros"].get("odom_frame_id", "odom")
    task["goal_frame_id"] = GOAL_FRAME
    task["output_path"] = str(Path(output) / "robot_task.json")
    bundle["robot_task_instance"] = task
    save_task(task)
    return task


def save_task(task):
    path = Path(task["output_path"])
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(".json.tmp")
    temporary.write_text(json.dumps(task, indent=2))
    temporary.replace(path)


class RobotTaskRuntime:
    """Use the actual odometry origin and monitor goal arrival after playback."""

    def __init__(self, robot, task):
        import omni.graph.core as og
        import omni.usd
        from pxr import Gf, Usd, UsdGeom

        self.robot, self.task, self.og = robot, task, og
        self.Gf, self.Usd, self.UsdGeom = Gf, Usd, UsdGeom
        stage = omni.usd.get_context().get_stage()
        self.base_prim = stage.GetPrimAtPath(robot.require_link("base").prim_path)
        self.scale = UsdGeom.GetStageMetersPerUnit(stage)
        self.graph = f"{robot.root_prim_path}/ROS2_ActionGraph"
        self.attributes = {
            name: og.Controller.attribute(f"{self.graph}/ComputeOdometry.outputs:{name}")
            for name in ("position", "orientation", "linearVelocity", "angularVelocity")
        }
        if not all(a.is_valid() for a in self.attributes.values()):
            raise RuntimeError("Robot task requires generated odometry")
        self.monitor = ArrivalMonitor(task["goal_world"], task["config"])
        self.overlay = None
        self.reached = False

    def world_pose(self):
        pose = self.UsdGeom.XformCache(self.Usd.TimeCode.Default()).GetLocalToWorldTransform(
            self.base_prim
        )
        p = pose.ExtractTranslation()
        self.world_z = p[2] * self.scale
        f = pose.TransformDir(self.Gf.Vec3d(1, 0, 0))
        return [p[0] * self.scale, p[1] * self.scale, math.atan2(f[1], f[0])]

    def _publish_goal(self):
        goal = self.task["goal_odom"]
        world = self.task["world_from_odom"]
        namespace = self.robot.metadata.get("namespace", "").strip("/")
        topic = f"/{namespace}/tf_static" if namespace else "/tf_static"
        nodes = [
            (
                "PublishTaskGoal",
                self.task["odom_frame_id"],
                GOAL_FRAME,
                [goal[0], goal[1], self.task["goal_world"][2] - self.task["world_from_odom_z"]],
                [0, 0, 0, 1],
            ),
            (
                "PublishTaskOrigin",
                "world",
                self.task["odom_frame_id"],
                [world[0], world[1], self.task["world_from_odom_z"]],
                [0, 0, math.sin(world[2] / 2), math.cos(world[2] / 2)],
            ),
        ]
        keys = self.og.Controller.Keys
        values, connections = [], []
        for name, parent, child, translation, rotation in nodes:
            values.extend(
                [
                    (f"{name}.inputs:parentFrameId", parent),
                    (f"{name}.inputs:childFrameId", child),
                    (f"{name}.inputs:translation", translation),
                    (f"{name}.inputs:rotation", rotation),
                    (f"{name}.inputs:staticPublisher", True),
                    (f"{name}.inputs:topicName", topic),
                ]
            )
            # Existing graph nodes need absolute paths when extending a graph.
            connections.extend(
                [
                    (f"{self.graph}/OnPlaybackTick.outputs:tick", f"{name}.inputs:execIn"),
                    (f"{self.graph}/Context.outputs:context", f"{name}.inputs:context"),
                    (
                        f"{self.graph}/ReadSimTime.outputs:simulationTime",
                        f"{name}.inputs:timeStamp",
                    ),
                ]
            )
        self.og.Controller.edit(
            self.graph,
            {
                keys.CREATE_NODES: [
                    (name, "isaacsim.ros2.bridge.ROS2PublishRawTransformTree") for name, *_ in nodes
                ],
                keys.SET_VALUES: values,
                keys.CONNECT: connections,
            },
        )

    def update(self, simulation_time):
        world = self.world_pose()
        values = {
            name: self.og.Controller.get(attribute) for name, attribute in self.attributes.items()
        }
        q = values["orientation"]
        if hasattr(q, "GetReal"):
            x, y, z = q.GetImaginary()
            w = q.GetReal()
        else:
            x, y, z, w = q
        yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
        if "goal_odom" not in self.task:
            odom = [values["position"][0], values["position"][1], yaw]
            self.task["world_from_odom"] = calibrate_world_from_odom(world, odom)
            self.task["world_from_odom_z"] = float(self.world_z - values["position"][2])
            self.task["goal_odom"] = world_point_to_odom(
                self.task["goal_world"], self.task["world_from_odom"]
            ).tolist()
            self.task.update(status="active", calibration_sim_time=float(simulation_time))
            self._publish_goal()
            save_task(self.task)
        self.reached = self.monitor.update(
            simulation_time, world, values["linearVelocity"], values["angularVelocity"][2]
        )
        if self.overlay is not None:
            self.overlay.update(world, self.reached, simulation_time=simulation_time)
        if self.reached:
            self.task.update(
                status="success",
                reached_sim_time=float(simulation_time),
                final_distance_m=float(np.linalg.norm(np.asarray(world[:2]) - self.monitor.goal)),
            )
            save_task(self.task)
        return self.reached

    def shutdown(self):
        if self.overlay is not None:
            self.overlay.destroy()
            self.overlay = None
