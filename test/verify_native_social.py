"""Native Isaac tests, isolated from production data and joystick input."""

import json
import argparse
import tempfile
import subprocess
import math
import sys
from pathlib import Path
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import isaac_composer.runtime as runtime
from isaac_composer.pedestrian_routes import PedestrianRoutePlanner

parser = argparse.ArgumentParser(description="Isolated native warehouse crowd verification")
parser.add_argument(
    "case",
    choices=[
        "head_on",
        "rear_approach",
        "crossing",
        "robot",
        "moving_robot",
        "yield",
        "close_pass",
        "narrow_robot",
        "locomotion",
        "random",
    ],
)
parser.add_argument("--duration-sec", type=float, default=60)
parser.add_argument("--output-dir", type=Path)
parser.add_argument(
    "--replay-scenario-dir",
    type=Path,
    help="Reuse recorded starts and pedestrian routes; A2 remains stationary",
)
parser.add_argument("--video", action="store_true", help="Record actual Isaac RGB video")
parser.add_argument("--gui", action="store_true", help="Show the actual Isaac window")
parser.add_argument(
    "--light", action="store_true", help="Disable LiDAR for motion-only verification"
)
parser.add_argument("--worker", action="store_true", help=argparse.SUPPRESS)
args = parser.parse_args()
if args.replay_scenario_dir and args.case != "random":
    parser.error("--replay-scenario-dir requires random")
case = args.case
root = args.output_dir if args.output_dir else Path(tempfile.mkdtemp(prefix="isaac-social-")) / case
if not args.worker:
    # Kit can terminate the interpreter while closing SimulationApp. Validate
    # recorded results in a parent process so a failed assertion is not skipped.
    command = [
        sys.executable,
        str(Path(__file__).resolve()),
        case,
        "--worker",
        "--duration-sec",
        str(args.duration_sec),
        "--output-dir",
        str(root),
    ]
    if args.light:
        command.append("--light")
    if args.video:
        command.append("--video")
    if args.gui:
        command.append("--gui")
    if args.replay_scenario_dir:
        command.extend(["--replay-scenario-dir", str(args.replay_scenario_dir.resolve())])
    completed = subprocess.run(command)
    if completed.returncode:
        raise SystemExit(completed.returncode)
    result = json.loads((root / "verification.json").read_text())
    if case != "random":
        rear_packets = (
            [
                json.loads(line)
                for line in (root / "scenario/crowd_states.jsonl").read_text().splitlines()
            ]
            if case == "rear_approach"
            else None
        )
        for item in result["people"]:
            # A completed, non-looping actor still participates in avoidance:
            # a later pass can displace it after arrival has been latched. Check
            # that arrival actually occurred, not that the body became frozen.
            if rear_packets is not None:
                distances = [
                    np.linalg.norm(
                        np.asarray(p["pedestrians"][item["id"]][1:3]) - np.asarray(item["goal"])[:2]
                    )
                    for p in rear_packets
                ]
                assert min(distances) <= 0.35, item
                item["minimum_observed_goal_distance_m"] = float(min(distances))
                item["final_goal_distance_m"] = float(
                    np.linalg.norm(np.asarray(item["end"])[:2] - np.asarray(item["goal"])[:2])
                )
                continue
            assert (
                np.linalg.norm(np.array(item["end"])[:2] - np.array(item["goal"])[:2]) <= 0.35
            ), item
        diagnostics = result["diagnostics"]
        for field, minimum in [
            ("minimum_pedestrian_separation_m", 0.6),
            ("minimum_robot_separation_m", 0.85),
        ]:
            value = diagnostics[field]
            assert value is None or value >= minimum, diagnostics
        if case == "yield":
            assert any(e["action"] == "yield" for e in diagnostics["events"]), diagnostics
            assert any(e["action"] == "resume" for e in diagnostics["events"]), diagnostics
        if case in ("head_on", "rear_approach"):
            assert diagnostics["directional_responsibility"]["enabled"]
            assert diagnostics["social_stiffness"]["enabled"]
            assert not diagnostics["deadlock_recovery_enabled"]
            packets = [
                json.loads(line)
                for line in (root / "scenario/crowd_states.jsonl").read_text().splitlines()
            ]
            errors = []
            for i in range(len(result["people"])):
                moving = [
                    p
                    for p in packets
                    if p["stamp_ns"] > 600000000 and np.linalg.norm(p["pedestrians"][i][3:5]) > 0.3
                ]
                assert len(moving) >= 20, "Need actual walking to verify character front axis"
                offsets = np.array(
                    [
                        p["directional_responsibility"]["body_heading_rad"][i]
                        - math.atan2(p["pedestrians"][i][4], p["pedestrians"][i][3])
                        for p in moving
                    ]
                )
                error = float(np.median(abs(np.arctan2(np.sin(offsets), np.cos(offsets)))))
                assert error < math.radians(30), (
                    "Character front axis differs from actual motion",
                    i,
                    error,
                )
                errors.append(error)
            result["directional_native_audit"] = {
                "body_heading_median_motion_error_rad": errors,
                "character_local_forward_axis": "negative_y",
                "walking_observed": True,
            }
            (root / "verification.json").write_text(json.dumps(result, indent=2))
        if case in ("yield", "close_pass", "narrow_robot"):
            packets = [
                json.loads(line)
                for line in (root / "scenario/crowd_states.jsonl").read_text().splitlines()
            ]
            xy = np.array([packet["pedestrians"] for packet in packets])[:, :, 1:3]
            minimum_wall = float("inf")
            for x, y, length, width in [
                (0, 9.0, 8, 0.2),
                (0, 11.0, 2, 0.2),
                (-3.75, 11.0, 0.5, 0.2),
                (3.75, 11.0, 0.5, 0.2),
            ]:
                delta = np.maximum(np.abs(xy - np.array([x, y])) - np.array([length, width]) / 2, 0)
                minimum_wall = min(minimum_wall, float(np.linalg.norm(delta, axis=-1).min()))
            assert minimum_wall >= 0.3 - 1e-5, ("Wall overlap", minimum_wall)
            print("MINIMUM_WALL_CLEARANCE", minimum_wall, flush=True)

    print("NATIVE_SOCIAL_VERIFIED", root, flush=True)
    raise SystemExit(0)

root.mkdir(parents=True, exist_ok=False)
original_load = runtime.load_scenario_bundle
original_plan = PedestrianRoutePlanner.plan
original_shutdown = runtime.SimulationRunner.shutdown
original_setup = runtime.SimulationRunner.setup
original_create_world = runtime.SimulationRunner._create_world
original_step = runtime.SimulationRunner.step


def load(name):
    bundle = original_load(name)
    if args.replay_scenario_dir:
        task = json.loads((args.replay_scenario_dir / "robot_task.json").read_text())
        bundle["pedestrians"]["seed"] = task["seed"]
        pose = bundle["robots"][0]["instance"]["pose"]
        pose["position"] = task["spawn_world"]
        pose["orientation_quat_wxyz"] = [
            math.cos(task["spawn_yaw"] / 2),
            0,
            0,
            math.sin(task["spawn_yaw"] / 2),
        ]
    if args.light:
        bundle["robots"][0]["preset"]["sensors"]["items"] = []
        bundle["simulation"]["application"].update(width=320, height=240)
    if args.video:
        bundle["simulation"]["viewport"]["mode"] = "free"
    cfg = bundle["pedestrians"]["social_navigation"]
    cfg.update(
        enabled=True, yielding_enabled=case in ("yield", "close_pass", "narrow_robot", "random")
    )
    if case == "yield":
        cfg.update(mu_range=[1, 1])
    if case in ("yield", "close_pass", "narrow_robot"):
        # This fixture adds new USD geometry absent from the prebuilt warehouse
        # occupancy map. Use its freshly baked native mesh for this test only.
        bundle["pedestrians"]["generation"]["scene"].pop("occupancy_map", None)
    if case == "robot":
        bundle["robots"][0]["instance"]["pose"]["position"] = [0, 10, 0.65]
    if case == "moving_robot":
        bundle["robots"][0]["instance"]["pose"]["position"] = [0, 8, 0.65]
    if case == "narrow_robot":
        cfg["mu_range"] = [1, 1]
        pose = bundle["robots"][0]["instance"]["pose"]
        pose["position"] = [3, 10, 0.65]
        pose["orientation_quat_wxyz"] = [0, 0, 0, 1]
    return bundle


def plan(self):
    if case == "random":
        if args.replay_scenario_dir:
            return json.loads((args.replay_scenario_dir / "pedestrian_routes.json").read_text())[
                "people"
            ]
        return original_plan(self)
    self.config["repeat"] = False
    pairs = [([-3, 10, 0], [3, 10, 0]), ([3, 10, 0], [-3, 10, 0])]
    if case == "crossing":
        pairs[1] = ([0, 7, 0], [0, 13, 0])
    if case == "rear_approach":
        pairs = [([-1, 10.1, 0], [3, 10.1, 0]), ([-3, 9.9, 0], [4.5, 9.9, 0])]
    if case in ("robot", "moving_robot", "narrow_robot", "locomotion"):
        pairs = pairs[:1]
    return [
        self._person(
            i,
            self._route(self._project(a), self._project(b)),
            ([0.65, 1.1][i] if case == "rear_approach" else 0.8),
        )
        for i, (a, b) in enumerate(pairs)
    ]


def create_world(self):
    original_create_world(self)
    if case in ("yield", "close_pass", "narrow_robot"):
        # Real USD walls create a narrow central corridor with passing bays on
        # both sides. Bake includes these walls; production environments unchanged.
        import omni.usd
        from pxr import UsdGeom, UsdPhysics, Gf

        stage = omni.usd.get_context().get_stage()
        parent = self.bundle["environment"]["asset"]["prim_path"] + "/CrowdTestWalls"
        pieces = [
            (0, 9.0, 8, 0.2),
            (0, 11.0, 2, 0.2),
            (-3.75, 11.0, 0.5, 0.2),
            (3.75, 11.0, 0.5, 0.2),
        ]
        for i, (x, y, length, width) in enumerate(pieces):
            cube = UsdGeom.Mesh.Define(stage, parent + "/Wall_" + str(i))
            cube.CreatePointsAttr(
                [
                    (-0.5, -0.5, -0.5),
                    (0.5, -0.5, -0.5),
                    (0.5, 0.5, -0.5),
                    (-0.5, 0.5, -0.5),
                    (-0.5, -0.5, 0.5),
                    (0.5, -0.5, 0.5),
                    (0.5, 0.5, 0.5),
                    (-0.5, 0.5, 0.5),
                ]
            )
            cube.CreateFaceVertexCountsAttr([4] * 6)
            cube.CreateFaceVertexIndicesAttr(
                [0, 3, 2, 1, 4, 5, 6, 7, 0, 1, 5, 4, 1, 2, 6, 5, 2, 3, 7, 6, 3, 0, 4, 7]
            )
            cube.CreateSubdivisionSchemeAttr("none")
            cube.AddTranslateOp().Set(Gf.Vec3d(x, y, 1.0))
            cube.AddScaleOp().Set(Gf.Vec3d(length, width, 2.0))
            UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
        self.simulation_app.update()


def setup(self):
    original_setup(self)
    if case in ("moving_robot", "narrow_robot"):
        from pxr import Usd, UsdGeom, Gf
        import omni.usd

        prim = (
            omni.usd.get_context()
            .get_stage()
            .GetPrimAtPath(self.robots[0].context.require_link("base").prim_path)
        )

        def command():
            t = float(self.world.current_time)
            transform = UsdGeom.XformCache(Usd.TimeCode.Default()).GetLocalToWorldTransform(prim)
            direction = transform.TransformDir(Gf.Vec3d(1, 0, 0))
            yaw = math.atan2(direction[1], direction[0])
            if case == "narrow_robot":
                position = transform.ExtractTranslation()
                delta = np.array([-3.0 - position[0], 10.0 - position[1]])
                distance = float(np.linalg.norm(delta))
                if t < 2 or distance < 0.25:
                    return np.zeros(3)
                world_velocity = delta / distance * min(0.45, distance)
                c, s = math.cos(yaw), math.sin(yaw)
                body = np.array(
                    [
                        c * world_velocity[0] + s * world_velocity[1],
                        -s * world_velocity[0] + c * world_velocity[1],
                    ]
                )
                yaw_error = math.atan2(math.sin(math.pi - yaw), math.cos(math.pi - yaw))
                return np.r_[body, np.clip(yaw_error, -0.6, 0.6)]
            speed = 0.6 if 1.5 < t < 10 else 0
            return np.array([math.sin(yaw) * speed, math.cos(yaw) * speed, 0])

        self.robots[0].controller._read_command = command
    if args.video:
        from native_demo_video import DemoVideo

        title = "Pedestrians: opposite directions | corridor width 1.8 m"
        if case == "narrow_robot":
            title = "Pedestrian vs A2 | A2: scripted demo motion, 0.45 m/s"
        elif case == "locomotion":
            title = "Single pedestrian | path reuse and locomotion tracking"
        elif case == "random":
            title = "Warehouse pedestrians | A2 stationary | locomotion verification"
        self.demo_video = DemoVideo(self, root, title)


def step(self):
    original_step(self)
    if args.video:
        self.demo_video.capture(self)


def shutdown(self):
    if getattr(self, "demo_video", None):
        self.demo_video.close()
    if self.people:
        backend = self.people[0]._crowd_runtime
        rows = [
            {"id": i, "end": p.state.position.tolist(), "goal": c["waypoints"][-1]}
            for i, (p, c) in enumerate(zip(self.people, self.bundle["pedestrians"]["people"]))
        ]
        result = {
            "case": case,
            "people": rows,
            "diagnostics": backend.diagnostics(),
            "publisher_connected": backend.publisher.connected,
            "navigation": {
                "pedestrian_bake_radius_m": backend.context.scene.agent_radius,
                "pedestrian_query_radius_m": getattr(
                    backend.context.scene, "routing_radius_m", None
                ),
            },
        }
        (root / "verification.json").write_text(json.dumps(result, indent=2))
        print("NATIVE_SOCIAL_RESULT", result, flush=True)
    original_shutdown(self)


runtime.load_scenario_bundle = load
PedestrianRoutePlanner.plan = plan
runtime.SimulationRunner.shutdown = shutdown
runtime.SimulationRunner.setup = setup
runtime.SimulationRunner._create_world = create_world
runtime.SimulationRunner.step = step
runtime.main(
    [
        "warehouse_a2_random",
        "--no-headless" if args.gui else "--headless",
        "--robot-task",
        "false",
        "--seed",
        (
            str(json.loads((args.replay_scenario_dir / "robot_task.json").read_text())["seed"])
            if args.replay_scenario_dir
            else "43"
        ),
        "--duration-sec",
        str(args.duration_sec),
        "--generation-output-dir",
        str(root / "scenario"),
        "--run-report",
        str(root / "report.json"),
    ]
)
