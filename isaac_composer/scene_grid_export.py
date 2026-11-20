"""Load an environment and export grid addresses using the existing native mesh."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import time
import subprocess
import sys
import traceback
import os

from .config import load_scenario_bundle


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("scenario", help="Scenario preset, e.g. warehouse_a2_random")
    parser.add_argument("--cell-size-m", type=float, default=2.0)
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument(
        "--scene-image",
        action="store_true",
        help="Also capture actual world RGB with grid IDs for Cosmos",
    )
    parser.add_argument("--worker", action="store_true", help=argparse.SUPPRESS)
    args = parser.parse_args(argv)
    bundle = load_scenario_bundle(args.scenario)
    package_root = Path(bundle["config_root"]).parent
    output = args.output_dir or package_root / "generated/scene_grid" / bundle["scenario"]["name"]
    output.mkdir(parents=True, exist_ok=True)
    status_path = output / "export_status.json"
    if not args.worker:
        status_path.write_text(json.dumps({"status": "running"}))
        command = [
            sys.executable,
            str(Path(__file__).resolve()),
            args.scenario,
            "--worker",
            "--cell-size-m",
            str(args.cell_size_m),
            "--output-dir",
            str(output),
        ]
        if args.scene_image:
            command.append("--scene-image")
        # Use package import for the child; direct module files have relative imports.
        command[1:2] = ["-m", "isaac_composer.scene_grid_export"]
        child_env = dict(os.environ)
        child_env["PYTHONPATH"] = (
            str(Path(__file__).resolve().parent.parent)
            + os.pathsep
            + child_env.get("PYTHONPATH", "")
        )
        process = subprocess.run(command, env=child_env)
        status = json.loads(status_path.read_text())
        if process.returncode or status.get("status") != "complete":
            raise RuntimeError(f"Scene export failed: {status}")
        return
    # Only load the environment and bake the pedestrian footprint. No episodes,
    # random actor routes, robot task goals or animation commands are generated.
    bundle["robot_task"]["enabled"] = False
    from isaacsim import SimulationApp

    app = SimulationApp({"headless": True, "width": 320, "height": 240})
    try:
        import omni.usd
        from .runtime import SimulationRunner
        from .pedestrian_routes import build_pedestrian_navmesh
        from .scene_grid import build_scene_grid, save_scene_grid

        runner = SimulationRunner(app, bundle)
        runner._enable_extensions()
        runner._create_world()
        deadline = time.monotonic() + 120
        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            if time.monotonic() > deadline:
                raise RuntimeError("Timed out loading environment assets")
            app.update()
        scene = build_pedestrian_navmesh(bundle, package_root)
        grid = build_scene_grid(scene, args.cell_size_m)
        save_scene_grid(grid, output, scene=scene)
        result = grid.to_dict()
        result["scenario"] = bundle["scenario"]["name"]
        result["environment"] = bundle["environment"]["name"]
        result["source_usd"] = bundle["environment"]["asset"]["usd_path"]
        (output / "grid_cells.json").write_text(json.dumps(result, indent=2, allow_nan=False))
        if args.scene_image:
            from .semantic_scenario.scene_input import capture_scene_image

            capture_scene_image(app, scene, grid, output)
        print(
            f"SCENE_GRID_EXPORTED {output.resolve()} | kept={len(grid.cells)} removed={result['removed_cell_count']}",
            flush=True,
        )
        status_path.write_text(json.dumps({"status": "complete"}))
    except Exception as error:
        traceback.print_exc()
        status_path.write_text(
            json.dumps(
                {
                    "status": "failed",
                    "error": str(error).encode("ascii", "backslashreplace").decode("ascii"),
                }
            )
        )
    finally:
        app.close()


if __name__ == "__main__":
    main()
