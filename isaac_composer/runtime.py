"""Isaac Sim scenario runner backed by the configured provider registries."""

from __future__ import annotations

import argparse
import time
import traceback
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from .assets import resolve_asset
from .config import load_scenario_bundle
from .models import RobotContext, SensorHandle
from .registry import (
    clock_providers,
    controller_providers,
    ros_providers,
    sensor_providers,
    spawn_providers,
)
from .stage import add_usd_reference, ensure_physics_scene


EXTENSIONS = (
    "omni.graph.nodes",
    "isaacsim.core.nodes",
    "isaacsim.ros2.bridge",
    "isaacsim.sensors.physics",
    "isaacsim.robot.wheeled_robots",
    "isaacsim.robot.policy.examples",
    "omni.kit.scripting",
    "omni.physx",
    "omni.anim.timeline",
    "omni.anim.navigation.core",
    "omni.anim.graph.core",
    "omni.anim.graph.bundle",
    "omni.anim.people",
    "omni.isaac.quadruped",
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

    def __init__(self, simulation_app: Any, bundle: dict[str, Any]) -> None:
        self.simulation_app = simulation_app
        self.bundle = bundle
        self.config_root = Path(bundle["config_root"])
        self.package_root = self.config_root.parent
        self.timeline: Any | None = None
        self.world: Any | None = None
        self.clock: Any | None = None
        self.robots: list[RobotRuntime] = []
        self.people: list[Any] = []
        simulation = bundle["simulation"]
        self.physics_dt = float(simulation["physics"]["dt"])
        self.target_fps = float(simulation["timeline"]["target_fps"])
        self.play_delay = float(simulation["timeline"].get("play_delay", 0.0))
        self.ready_wall_time = 0.0
        self.start_wall_time: float | None = None
        self.play_started = False
        self.frame_count = 0

    def _provider(self, registry, name: str):
        return registry.create(
            name,
            package_root=self.package_root,
            config_root=self.config_root,
        )

    def _enable_extensions(self) -> None:
        import omni.kit.app

        manager = omni.kit.app.get_app().get_extension_manager()
        for extension in EXTENSIONS:
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
        carb.log_info(f"Loaded environment: {environment_path}")

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
            key: first.get(key)
            for key in ("provider", "topic", "queue_size", "domain_id")
        }
        for clock in configured[1:]:
            other = {key: clock.get(key) for key in comparable}
            if other != comparable:
                raise ValueError(
                    "All robots in one scenario must share the same clock settings"
                )
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
        from .pedestrians import build_pedestrians

        self.people = build_pedestrians(self.world, self.bundle["pedestrians"])
        self.simulation_app.update()

    def setup(self) -> None:
        import carb
        import omni.usd
        from omni.timeline import get_timeline_interface

        load_builtin_providers()
        self._enable_extensions()
        self.timeline = get_timeline_interface()
        self._create_world()
        self._setup_clock()
        self._setup_robots()
        self._setup_pedestrians()

        self.world.reset()
        for runtime in self.robots:
            runtime.controller.start_after_reset(runtime.context, self.world)

        self.timeline.set_time_codes_per_second(self.target_fps)
        self.timeline.commit_silently()
        self.timeline.stop()
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
        self.play_started = True
        self.start_wall_time = time.time()

    def step(self) -> None:
        import carb

        self._start_playback_when_ready()
        if not self.play_started:
            self.simulation_app.update()
            return

        for runtime in self.robots:
            runtime.controller.step(self.physics_dt)
        self.world.step(render=True)
        self.frame_count += 1
        log_interval = max(1, round(1.0 / self.physics_dt))
        if self.frame_count % log_interval == 0:
            elapsed = time.time() - self.start_wall_time
            real_fps = self.frame_count / elapsed if elapsed > 0.0 else 0.0
            carb.log_info(
                f"frame={self.frame_count}, "
                f"sim_time={self.frame_count * self.physics_dt:.2f}, "
                f"real_fps={real_fps:.2f}"
            )

    def run(self, max_frames: int = 0) -> None:
        self.setup()
        while self.simulation_app.is_running():
            self.step()
            if max_frames > 0 and self.frame_count >= max_frames:
                break

    def shutdown(self) -> None:
        if self.timeline is not None:
            self.timeline.stop()
        if self.world is not None and self.people:
            from .pedestrians import shutdown_pedestrians

            shutdown_pedestrians(self.world, self.people)
        self.people.clear()
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
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> None:
    args = _parse_args(argv)
    bundle = load_scenario_bundle(args.scenario)
    simulation = bundle["simulation"]
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
    runner = SimulationRunner(simulation_app, bundle)
    try:
        runner.run(max_frames=max(0, args.max_frames))
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
        runner.shutdown()
        simulation_app.close()


if __name__ == "__main__":
    main()
