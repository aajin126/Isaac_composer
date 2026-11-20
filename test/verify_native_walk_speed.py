"""Measure actual walking and animation root motion in an isolated Isaac process.

Run with Isaac Python and an unused ROS_DOMAIN_ID. No joystick, LiDAR or
production episode output is used. The parent validates after Kit shuts down.
"""

import argparse
import json
import subprocess
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--output-dir", type=Path, required=True)
parser.add_argument("--worker", action="store_true")
parser.add_argument("--video", action="store_true")
parser.add_argument("--automatic-baseline", action="store_true")
args = parser.parse_args()
root = args.output_dir.resolve()

if not args.worker:
    command = [sys.executable, str(Path(__file__).resolve()), "--worker", "--output-dir", str(root)]
    command += [
        flag
        for flag, enabled in [
            ("--video", args.video),
            ("--automatic-baseline", args.automatic_baseline),
        ]
        if enabled
    ]
    subprocess.run(command, check=True)
    report = json.loads((root / "report.json").read_text())
    assert report["status"] == "complete", report
    result = json.loads((root / "results.json").read_text())
    measured = [p["measured_mps"] for p in result["people"]]
    assert result["fabric_vs_graph_max_error_m"] < 0.02, result
    if not args.automatic_baseline:
        assert measured[0] < measured[1] < measured[2], result
        assert measured[2] > 1.4, result
        assert 1.4 < measured[2] / measured[1] < 1.9, result  # Reject double advancement.
        assert result["people"][2]["maximum_playback_rate"] > 1.5, result
    print("NATIVE_WALK_SPEED_VERIFIED", json.dumps(result), flush=True)
    raise SystemExit(0)

import isaac_composer.runtime as runtime
from isaac_composer.pedestrian_routes import PedestrianRoutePlanner

root.mkdir(parents=True, exist_ok=False)
original_load = runtime.load_scenario_bundle
original_setup = runtime.SimulationRunner.setup
original_step = runtime.SimulationRunner.step
original_shutdown = runtime.SimulationRunner.shutdown
speeds = [0.6, 1.0, 1.6]
samples = []


def load(name):
    bundle = original_load(name)
    bundle["robots"][0]["preset"]["sensors"]["items"] = []
    bundle["simulation"]["application"].update(width=320, height=240)
    bundle["simulation"]["viewport"]["mode"] = "free"
    bundle["pedestrians"]["social_navigation"].update(
        enabled=True, yielding_enabled=False, record_states_enabled=True, mu_range=[0, 0]
    )
    return bundle


def plan(self):
    self.config["repeat"] = False
    self.config["characters"] = ["male_adult_construction_01_new"]
    return [
        self._person(i, self._route(self._project([-3, y, 0]), self._project([6, y, 0])), speed)
        for i, (y, speed) in enumerate(zip([7, 10, 13], speeds))
    ]


def setup(self):
    original_setup(self)
    if args.video:
        from native_demo_video import DemoVideo

        self.speed_video = DemoVideo(self, root, "Walk targets: 0.6 / 1.0 / 1.6 m/s")
        from pxr import Gf, UsdGeom

        camera = UsdGeom.Camera(self.world.stage.GetPrimAtPath("/World/CrowdDemoCamera"))
        camera.CreateFocalLengthAttr().Set(14.0)
        UsdGeom.Xformable(camera.GetPrim()).GetOrderedXformOps()[0].Set(
            Gf.Matrix4d()
            .SetLookAt(Gf.Vec3d(1, 10, 5), Gf.Vec3d(1, 10, 0), Gf.Vec3d(0, 1, 0))
            .GetInverse()
        )


def step(self):
    original_step(self)
    if not self.play_started or not self.people:
        return
    if args.video:
        self.speed_video.capture(self)
    import carb
    from isaacsim.core.utils.xforms import get_world_pose

    stamp = float(self.world.current_time)
    actual, rendered = [], []
    for person in self.people:
        rendered.append(
            get_world_pose(person.character_skel_root_stage_path, fabric=True)[0].tolist()
        )
        xyz, quat = carb.Float3(0, 0, 0), carb.Float4(0, 0, 0, 0)
        person.character_graph.get_world_transform(xyz, quat)
        actual.append(list(xyz))
    samples.append(
        {
            "time_s": stamp,
            "positions": actual,
            "fabric_positions": rendered,
            "playback_rate": [p._animation_playback_rate for p in self.people],
        }
    )


def shutdown(self):
    if getattr(self, "speed_video", None):
        self.speed_video.close()
    if samples:
        rows = []
        window = [s for s in samples if 2 <= s["time_s"] <= 4]
        times = np.array([s["time_s"] for s in window])
        for i, speed in enumerate(speeds):
            xy = np.array([s["positions"][i][:2] for s in window])
            velocity = [float(np.polyfit(times, xy[:, k], 1)[0]) for k in range(2)]
            rows.append(
                {
                    "requested_mps": speed,
                    "measured_mps": float(np.linalg.norm(velocity)),
                    "velocity_xy_mps": velocity,
                    "maximum_playback_rate": max(s["playback_rate"][i] for s in samples),
                }
            )
        maximum_error = max(
            np.linalg.norm(
                np.array(s["positions"])[:, :2] - np.array(s["fabric_positions"])[:, :2], axis=1
            ).max()
            for s in samples
        )
        result = {
            "measurement": "Least-squares slope of actual root XY over simulation seconds 2 to 4",
            "people": rows,
            "fabric_vs_graph_max_error_m": float(maximum_error),
        }
        (root / "results.json").write_text(json.dumps(result, indent=2))
        (root / "samples.json").write_text(json.dumps(samples))
        print("NATIVE_WALK_SPEED_RESULT", json.dumps(result), flush=True)
    original_shutdown(self)


if args.automatic_baseline:
    from isaac_composer.crowd.runtime.animation import AnimationCommands

    original_reference = AnimationCommands._reference

    def capped_reference(self, index, requested, dt):
        limit = self.config["animation_walk_reference_mps"]
        norm = np.linalg.norm(requested)
        return original_reference(self, index, requested * min(1, limit / max(norm, 1e-9)), dt)

    def enable_automatic(self):
        import omni.kit.app

        manager = omni.kit.app.get_app().get_extension_manager()
        for extension in runtime.EXTENSIONS:
            manager.set_extension_enabled_immediate(extension, True)
        self.simulation_app.update()

    AnimationCommands._reference = capped_reference
    runtime.SimulationRunner._enable_extensions = enable_automatic

runtime.SimulationRunner.setup = setup
runtime.load_scenario_bundle = load
PedestrianRoutePlanner.plan = plan
runtime.SimulationRunner.step = step
runtime.SimulationRunner.shutdown = shutdown
runtime.main(
    [
        "warehouse_a2_random",
        "--headless",
        "--robot-task",
        "false",
        "--seed",
        "43",
        "--duration-sec",
        "6",
        "--generation-output-dir",
        str(root / "scenario"),
        "--run-report",
        str(root / "report.json"),
    ]
)
