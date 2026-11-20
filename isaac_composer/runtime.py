"""Isaac Sim scenario runner backed by the configured provider registries."""

from __future__ import annotations

import argparse
import math
import time
import traceback
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from .assets import resolve_asset
from .config import load_scenario_bundle, override_first_robot_pose
from .models import RobotContext, SensorHandle
from .registry import (
    clock_providers,
    controller_providers,
    ros_providers,
    sensor_providers,
    spawn_providers,
)
from .stage import add_usd_reference, ensure_physics_scene
from .performance import TimingStats

EXTENSIONS = (
    "omni.graph.nodes",
    "isaacsim.core.nodes",
    "isaacsim.ros2.bridge",
    "isaacsim.sensors.physics",
    "isaacsim.robot.wheeled_robots",
    "isaacsim.robot.policy.examples",
    "omni.kit.scripting",
    "omni.physx",
    "omni.replicator.core",
    "omni.isaac.quadruped",
)

PEDESTRIAN_EXTENSIONS = (
    "omni.anim.timeline",
    "omni.anim.navigation.core",
    "omni.anim.graph.core",
    "omni.anim.graph.bundle",
    "omni.anim.people",
)


def load_builtin_providers() -> None:
    """Import registration modules after SimulationApp has been constructed."""

    from .controllers import providers as controller_implementations
    from .ros import providers as ros_implementations
    from .sensors import providers as sensor_implementations
    from .spawn import providers as spawn_implementations

    # Keep explicit references so linters do not treat registration imports as dead.
    del (
        controller_implementations,
        ros_implementations,
        sensor_implementations,
        spawn_implementations,
    )


@dataclass
class RobotRuntime:
    context: RobotContext
    controller: Any
    sensors: Any
    ros: Any
    sensor_handles: list[SensorHandle]


class SimulationRunner:
    """Compose one validated scenario inside an active SimulationApp."""

    def __init__(self, simulation_app: Any, bundle: dict[str, Any], *, use_pedestrians: bool = False) -> None:
        self.simulation_app = simulation_app
        self.bundle = bundle
        self.use_pedestrians = use_pedestrians
        self.config_root = Path(bundle["config_root"])
        self.package_root = self.config_root.parent
        self.timeline: Any | None = None
        self.world: Any | None = None
        self.clock: Any | None = None
        self.robots: list[RobotRuntime] = []
        self.people: list[Any] = []
        self.pedestrian_navmesh: Any | None = None
        self.viewport: Any | None = None
        self.animation_clock: Any | None = None
        simulation = bundle["simulation"]
        self.physics_dt = float(simulation["physics"]["dt"])
        self.target_fps = float(simulation["timeline"]["target_fps"])
        self.play_delay = float(simulation["timeline"].get("play_delay", 0.0))
        self.ready_wall_time = 0.0
        self.start_wall_time: float | None = None
        self.play_started = False
        self.frame_count = 0
        self.episode_start_sim_time: float | None = None
        self.last_log_sim_time = 0.0
        self.timings = TimingStats()

    def _provider(self, registry, name: str):
        return registry.create(
            name,
            package_root=self.package_root,
            config_root=self.config_root,
        )

    def _enable_extensions(self) -> None:
        import carb
        import omni.kit.app

        if self.use_pedestrians and (
            self.bundle["pedestrians"].get("people") or self.bundle["pedestrians"].get("generation")
        ):
            from .animation_clock import CharacterClock

            self.animation_clock = CharacterClock(carb.settings.get_settings())
        manager = omni.kit.app.get_app().get_extension_manager()
        extensions = EXTENSIONS + (PEDESTRIAN_EXTENSIONS if self.use_pedestrians else ())
        for extension in extensions:
            manager.set_extension_enabled_immediate(extension, True)
        self.simulation_app.update()

    def _create_world(self) -> None:
        import carb
        import omni.usd
        from pxr import UsdGeom

        try:
            from isaacsim.core.api import World
        except ImportError:
            from omni.isaac.core import World

        simulation = self.bundle["simulation"]
        environment = self.bundle["environment"]
        stage_units = float(environment["stage"].get("meters_per_unit", 1.0))
        omni.usd.get_context().new_stage()
        self.simulation_app.update()
        self.world = World(
            stage_units_in_meters=stage_units,
            physics_dt=self.physics_dt,
            rendering_dt=float(simulation["rendering"]["dt"]),
            physics_prim_path="/World/PhysicsScene",
        )
        stage = omni.usd.get_context().get_stage()
        UsdGeom.SetStageMetersPerUnit(stage, stage_units)
        ensure_physics_scene(stage)

        asset = environment["asset"]
        environment_path = resolve_asset(asset["usd_path"], self.package_root)
        add_usd_reference(stage, asset["prim_path"], environment_path)
        self.simulation_app.update()
        intensity = simulation["rendering"].get("indirect_diffuse_intensity")
        if intensity is not None:
            settings = carb.settings.get_settings()
            settings.set_bool("/rtx/indirectDiffuse/enabled", True)
            settings.set_float("/rtx/indirectDiffuse/scalingFactor", float(intensity))
            carb.log_info(f"Indirect diffuse GI enabled; intensity={float(intensity):g}")
        carb.log_info(f"Loaded environment: {environment_path}")

    def _prepare_pedestrian_routes(self) -> None:
        """Plan fixed/random routes before spawning the robot and pedestrians."""
        if not self.use_pedestrians:
            return
        from .pedestrian_routes import prepare_pedestrian_routes

        self.pedestrian_navmesh = prepare_pedestrian_routes(
            self.bundle, self.package_root
        )
        if self.pedestrian_navmesh is not None:
            self.simulation_app.update()

    def _setup_clock(self) -> None:
        import omni.usd

        configured = []
        for item in self.bundle["robots"]:
            preset = item["preset"]
            clock = dict(preset["clock"])
            clock.setdefault("domain_id", preset["ros"].get("domain_id", 0))
            configured.append(clock)
        first = configured[0]
        comparable = {
            key: first.get(key) for key in ("provider", "topic", "queue_size", "domain_id")
        }
        for clock in configured[1:]:
            other = {key: clock.get(key) for key in comparable}
            if other != comparable:
                raise ValueError("All robots in one scenario must share the same clock settings")
        self.clock = self._provider(clock_providers, first["provider"])
        self.clock.setup(first, stage=omni.usd.get_context().get_stage())

    def _setup_robots(self) -> None:
        import omni.usd

        stage = omni.usd.get_context().get_stage()
        multiple_instances = len(self.bundle["robots"]) > 1
        for item in self.bundle["robots"]:
            preset = item["preset"]
            instance = item["instance"]
            composition = {
                "preset": preset,
                "instance": instance,
                "multiple_instances": multiple_instances,
            }
            spawner = self._provider(spawn_providers, preset["spawn"]["provider"])
            context = spawner.spawn(
                composition,
                stage=stage,
                world=self.world,
            )
            context.metadata["controller_config"] = dict(preset["controller"])
            self.simulation_app.update()

            controller = self._provider(
                controller_providers,
                preset["controller"]["provider"],
            )
            controller.prepare(context, preset["controller"])

            ros = self._provider(ros_providers, preset["ros"]["provider"])
            ros.setup_robot(context, preset["ros"])
            self.simulation_app.update()

            sensors = self._provider(
                sensor_providers,
                preset["sensors"]["provider"],
            )
            sensor_handles = sensors.setup(context, preset["sensors"])
            self.simulation_app.update()

            ros.setup_sensors(context, sensor_handles, preset["ros"])
            self.simulation_app.update()
            self.robots.append(
                RobotRuntime(
                    context=context,
                    controller=controller,
                    sensors=sensors,
                    ros=ros,
                    sensor_handles=sensor_handles,
                )
            )

    def _setup_pedestrians(self) -> None:
        if not self.use_pedestrians:
            return
        from .pedestrians import build_pedestrians

        self.people = build_pedestrians(
            self.world,
            self.bundle["pedestrians"],
            navmesh_prepared=self.pedestrian_navmesh is not None,
        )
        self.simulation_app.update()

    def setup(self) -> None:
        import carb
        import omni.usd
        from omni.timeline import get_timeline_interface

        load_builtin_providers()
        self._enable_extensions()
        self.timeline = get_timeline_interface()
        self._create_world()
        self._prepare_pedestrian_routes()
        self._setup_clock()
        self._setup_robots()
        self._setup_pedestrians()

        self.world.reset()
        for runtime in self.robots:
            runtime.controller.start_after_reset(runtime.context, self.world)

        viewport_config = self.bundle["simulation"].get("viewport", {})
        if viewport_config.get("mode", "follow") != "free":
            from .viewport import RobotViewport

            self.viewport = RobotViewport(
                self.robots[0].context,
                viewport_config,
            )
            self.viewport.update(float(self.world.current_time), force=True)

        self.timeline.set_time_codes_per_second(self.target_fps)
        self.timeline.commit_silently()
        self.timeline.stop()
        if self.animation_clock is not None:
            import omni.anim.graph.core as ag

            self.animation_clock.start(ag.acquire_interface(), omni.usd.get_context().get_stage_id())
        self.ready_wall_time = time.time()
        carb.log_info(
            f"Isaac Composer scenario {self.bundle['scenario']['name']} ready; "
            f"playback starts after {self.play_delay:g}s"
        )

    def _start_playback_when_ready(self) -> None:
        if self.play_started:
            return
        if time.time() - self.ready_wall_time < self.play_delay:
            return
        self.timeline.play()
        self.episode_start_sim_time = float(self.world.current_time)
        self.play_started = True
        self.start_wall_time = time.time()

    def step(self) -> None:
        self._start_playback_when_ready()
        if not self.play_started:
            self.simulation_app.update()
            return

        frame_started = time.perf_counter()
        for runtime in self.robots:
            runtime.controller.step(self.physics_dt)
        if self.viewport is not None:
            self.viewport.update(float(self.world.current_time))
        prepared_at = time.perf_counter()
        animation_time = float(self.world.current_time)
        self.world.step(render=True)
        if self.animation_clock is not None:
            self.animation_clock.advance(float(self.world.current_time) - animation_time, self.people)
        stepped_at = time.perf_counter()
        self.timings.record(
            controller_and_viewport=prepared_at - frame_started,
            world_step=stepped_at - prepared_at,
            total=time.perf_counter() - frame_started,
        )
        self.frame_count += 1
        elapsed_sim = float(self.world.current_time) - self.episode_start_sim_time
        if elapsed_sim - self.last_log_sim_time >= 1.0:
            self.last_log_sim_time = elapsed_sim
            elapsed = time.time() - self.start_wall_time
            real_fps = self.frame_count / elapsed if elapsed > 0.0 else 0.0
            print(
                f"SIM_PERFORMANCE frame={self.frame_count}, "
                f"sim_time={elapsed_sim:.2f}, "
                f"real_fps={real_fps:.2f}, "
                f"realtime_factor={elapsed_sim / elapsed if elapsed > 0.0 else 0.0:.2f}, "
                f"frame_wall_ms={self.timings.latest['total']:.1f}",
                flush=True,
            )

    def run(self, max_frames: int = 0, duration_sec: float = 0.0) -> str:
        if not math.isfinite(duration_sec) or duration_sec < 0:
            raise ValueError("duration_sec must be finite and non-negative")
        self.setup()
        while self.simulation_app.is_running():
            self.step()
            if max_frames > 0 and self.frame_count >= max_frames:
                return "frame_limit"
            if (
                duration_sec > 0
                and self.episode_start_sim_time is not None
                and float(self.world.current_time) - self.episode_start_sim_time >= duration_sec
            ):
                return "duration"
        return "closed"

    def shutdown(self) -> None:
        if self.timeline is not None:
            self.timeline.stop()
        if self.animation_clock is not None:
            self.animation_clock.close()
            self.animation_clock = None
        if self.world is not None and self.people:
            from .pedestrians import shutdown_pedestrians

            shutdown_pedestrians(self.world, self.people)
        self.people.clear()
        self.viewport = None
        for runtime in reversed(self.robots):
            runtime.controller.shutdown()
            runtime.ros.shutdown()
            runtime.sensors.shutdown()
        self.robots.clear()
        if self.clock is not None:
            self.clock.shutdown()
            self.clock = None


def _parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run an Isaac Composer scenario")
    parser.add_argument("scenario", help="Scenario preset name or YAML path")
    parser.add_argument(
        "--use-pedestrians", action=argparse.BooleanOptionalAction, default=False,
        help="Spawn the scenario's pedestrian preset; disabled by default",
    )
    for coordinate in ("x", "y", "z", "yaw"):
        parser.add_argument(
            f"--robot-{coordinate}", default="",
            help=f"First robot's {coordinate}; empty preserves the scenario pose (meters/radians)",
        )
    parser.add_argument(
        "--headless",
        action=argparse.BooleanOptionalAction,
        default=None,
        help="Override config/simulation application.headless",
    )
    parser.add_argument(
        "--max-frames",
        type=int,
        default=0,
        help="Stop after this many played frames; zero runs until shutdown",
    )
    parser.add_argument(
        "--duration-sec",
        type=float,
        default=0.0,
        help="Stop after this many simulated seconds of playback; zero disables",
    )
    parser.add_argument(
        "--seed", type=int, default=-1, help="Override scenario seed; -1 preserves preset"
    )
    parser.add_argument(
        "--generation-output-dir", default="", help="Store generated scenario artifacts here"
    )
    parser.add_argument(
        "--view",
        choices=("follow", "top_down", "free"),
        default=None,
        help="Viewport mode; follows the first robot by default",
    )
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> None:
    args = _parse_args(argv)
    bundle = load_scenario_bundle(args.scenario)
    override_first_robot_pose(
        bundle, x=args.robot_x, y=args.robot_y, z=args.robot_z, yaw=args.robot_yaw
    )
    if args.seed >= 0:
        bundle["pedestrians"]["seed"] = args.seed
    if args.generation_output_dir and bundle["pedestrians"].get("generation"):
        bundle["pedestrians"]["generation"]["output_dir"] = args.generation_output_dir
    simulation = bundle["simulation"]
    if args.view is not None:
        simulation.setdefault("viewport", {})["mode"] = args.view
    application = simulation.get("application", {})
    app_config = {
        "renderer": simulation["rendering"]["renderer"],
        "headless": (
            bool(args.headless)
            if args.headless is not None
            else bool(application.get("headless", False))
        ),
        "width": int(application.get("width", 1280)),
        "height": int(application.get("height", 720)),
    }

    try:
        from isaacsim import SimulationApp
    except ImportError as error:
        raise RuntimeError(
            "Run this entry point with the Python interpreter supplied by Isaac Sim"
        ) from error

    simulation_app = SimulationApp(app_config)
    simulation_app.update()
    runner = SimulationRunner(simulation_app, bundle, use_pedestrians=args.use_pedestrians)
    try:
        runner.run(max_frames=max(0, args.max_frames), duration_sec=args.duration_sec)
    except KeyboardInterrupt:
        pass
    except Exception:
        try:
            import carb

            carb.log_error(traceback.format_exc())
        except ImportError:
            traceback.print_exc()
        raise
    finally:
        try:
            runner.shutdown()
        finally:
            simulation_app.close()


if __name__ == "__main__":
    main()
