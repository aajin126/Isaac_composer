"""Fresh world-conditioned planning before each teleoperated episode."""

from pathlib import Path
import hashlib
import json
import os
import signal
import subprocess
import math

from ..config import load_scenario_bundle
from ..interpreter import default_isaac_python
from .cosmos.client import CosmosClient
from .pipeline import ScenarioPipeline
from .runtime.compiler import load_reviewed_plan
from .scene_input import load_context
from .schema import PlanError, save_json
from .variation import history_entry
from .reporting import print_episode_plan
from .spatial import DEFAULT_ROUTE_BAND_HALF_WIDTH_M, enrich_context
from ..pedestrian_placement import DEFAULT_ROUTE_BAND_FRACTION


def world_key(bundle, cell_size_m):
    root = Path(bundle["config_root"]).parent
    generation = bundle["pedestrians"].get("generation", {})
    description = {
        "environment": bundle["environment"],
        "navmesh": bundle["pedestrians"]["navmesh"],
        "scene": generation.get("scene", {}),
        "cell_size_m": cell_size_m,
    }
    stamps = {}
    for raw in [
        bundle["environment"]["asset"]["usd_path"],
        generation.get("scene", {}).get("occupancy_map"),
    ]:
        if raw:
            path = Path(raw)
            if not path.is_absolute():
                path = root / path
            if path.is_file():
                stat = path.stat()
                stamps[str(path.resolve())] = [stat.st_size, stat.st_mtime_ns]
    return hashlib.sha256(json.dumps([description, stamps], sort_keys=True).encode()).hexdigest()


class EpisodeScenarioPlanner:
    def __init__(
        self,
        scenario,
        output_root,
        base_url="http://172.17.0.2:8000",
        model=None,
        total_count=0,
        cell_size_m=2.0,
        isaac_python=None,
        context_dir=None,
        request="Invent a plausible human activity situation appropriate to this world.",
        max_revisions=2,
        robot_task=False,
        route_band_half_width_m=DEFAULT_ROUTE_BAND_HALF_WIDTH_M,
        route_band_fraction=DEFAULT_ROUTE_BAND_FRACTION,
    ):
        if (
            total_count < 0
            or not math.isfinite(cell_size_m)
            or cell_size_m <= 0
            or max_revisions < 0
            or not math.isfinite(route_band_half_width_m)
            or route_band_half_width_m <= 0
            or not math.isfinite(route_band_fraction)
            or not 0 <= route_band_fraction <= 1
        ):
            raise PlanError(
                "Population and revisions must be nonnegative; cell size and route band half-width must be finite and positive"
            )
        self.scenario, self.root = scenario, Path(output_root).resolve()
        self.base_url, self.model, self.total_count = base_url, model, total_count
        self.cell_size_m, self.isaac_python = cell_size_m, isaac_python or default_isaac_python()
        self.context_dir = Path(context_dir).expanduser().resolve() if context_dir else None
        self.request, self.max_revisions = request, max_revisions
        self.robot_task = robot_task
        self.route_band_half_width_m = route_band_half_width_m
        self.route_band_fraction = route_band_fraction
        self.active_world = None
        self.cached_context = None

    def _export_context(self, bundle, key):
        directory = self.root / "_scene_context" / key[:16]
        script = Path(bundle["config_root"]).parent / "scripts/export_scene_grid.py"
        command = [
            self.isaac_python,
            str(script),
            self.scenario,
            "--scene-image",
            "--cell-size-m",
            str(self.cell_size_m),
            "--output-dir",
            str(directory),
        ]
        print(f"SEMANTIC_WORLD_EXPORT environment={bundle['environment']['name']}", flush=True)
        child = subprocess.Popen(command, start_new_session=True)
        try:
            if child.wait():
                raise PlanError("Isaac world context export failed")
        finally:
            if child.poll() is None:
                try:
                    os.killpg(child.pid, signal.SIGTERM)
                    child.wait(timeout=25)
                except subprocess.TimeoutExpired:
                    os.killpg(child.pid, signal.SIGKILL)
                    child.wait()
                except ProcessLookupError:
                    pass
        return directory

    def _context(self, bundle):
        key = world_key(bundle, self.cell_size_m)
        if self.active_world != key:
            directory = self.context_dir or self._export_context(bundle, key)
            context = load_context(directory, self.request)
            if context["environment"] != bundle["environment"]["name"]:
                raise PlanError("Cosmos scene context does not match the episode world")
            if context["grid"].get("source_usd") != bundle["environment"]["asset"]["usd_path"]:
                raise PlanError("Cosmos scene context USD does not match the episode world")
            if abs(context["grid"]["cell_size_m"] - self.cell_size_m) > 1e-6:
                raise PlanError(
                    "Pre-exported scene grid cell size does not match the requested size"
                )
            self.cached_context, self.active_world = context, key
        return self.cached_context

    def _recent_scenarios(self, environment):
        history = []
        for approval in sorted(
            self.root.glob("episode_*/scenario/semantic_plan/approval.json"), reverse=True
        ):
            try:
                context, _, plan = load_reviewed_plan(approval.parent)
                if context["environment"] != environment:
                    continue
                story = json.loads((approval.parent / "global_scenario.json").read_text())
                index = int(approval.parents[2].name.removeprefix("episode_"))
                history.append(history_entry(index, story, plan))
                if len(history) == 3:
                    break
            except (ValueError, KeyError, OSError, TypeError):
                continue
        return history

    def prepare(self, index, episode_dir, seed):
        bundle = load_scenario_bundle(self.scenario)
        generation = bundle["pedestrians"].get("generation")
        if not generation or not bundle["pedestrians"]["social_navigation"]["enabled"]:
            raise PlanError("Cosmos episodes require generated pedestrians and social navigation")
        context = dict(self._context(bundle))
        context["episode"] = {"episode_index": index, "seed": seed}
        context["recent_scenarios"] = self._recent_scenarios(context["environment"])
        total = self.total_count or int(generation.get("count", 1))
        directory = Path(episode_dir) / "scenario/semantic_plan"
        if self.robot_task:
            # The real task requires native NavMesh. Finish planning inside the
            # already-needed episode simulator, after sampling that task.
            save_json(
                directory / "approval.json", {"approved": False, "status": "waiting_for_robot_task"}
            )
            save_json(
                directory / "planning_request.json",
                {
                    "context": context,
                    "base_url": self.base_url,
                    "model": self.model,
                    "seed": seed,
                    "episode_index": index,
                    "total_count": total,
                    "max_revisions": self.max_revisions,
                    "route_band_half_width_m": self.route_band_half_width_m,
                    "route_band_fraction": self.route_band_fraction,
                },
            )
            print(f"SEMANTIC_WAITING_FOR_ROBOT_TASK episode={index:06d}", flush=True)
            return {
                "mode": "cosmos",
                "plan_dir": str(directory.resolve()),
                "planning_pending": True,
                "environment": context["environment"],
                "world_sha256": self.active_world,
                "planning_seed": seed,
                "total_count": total,
            }
        context = enrich_context(
            context,
            seed,
            route_band_half_width_m=self.route_band_half_width_m,
            route_band_fraction=self.route_band_fraction,
        )
        client = CosmosClient(
            self.base_url, self.model, seed=seed, response_dir=directory / "model_responses"
        )
        print(
            f"SEMANTIC_EPISODE_PLAN episode={index:06d} seed={seed} environment={context['environment']}",
            flush=True,
        )
        ScenarioPipeline(client, self.max_revisions).run(context, total, directory, seed)
        _, participants, plan = load_reviewed_plan(directory)
        story = json.loads((directory / "global_scenario.json").read_text())
        print_episode_plan(index, story, participants, plan)
        metadata = {
            "mode": "cosmos",
            "plan_dir": str(directory.resolve()),
            "environment": context["environment"],
            "world_sha256": self.active_world,
            "model": client.model,
            "planning_seed": seed,
            "total_count": total,
            "cosmos_count": participants["cosmos_count"],
            "random_count": participants["random_count"],
        }
        save_json(directory / "episode_planning.json", metadata)
        return metadata
