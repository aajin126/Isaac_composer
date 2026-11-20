"""Finish deferred Cosmos planning with the actual native robot task."""

import json
from pathlib import Path
from concurrent.futures import Future
from threading import Thread
from queue import Queue, Empty

from ..cosmos.client import CosmosClient
from ..pipeline import ScenarioPipeline
from ..reporting import print_episode_plan
from ..schema import save_json
from ..spatial import DEFAULT_ROUTE_BAND_HALF_WIDTH_M, enrich_context
from ..scene_input import render_robot_context
from ...pedestrian_placement import DEFAULT_ROUTE_BAND_FRACTION


def finish_pending_plan(directory, task, update_app=None, scene=None):
    directory = Path(directory)
    request = json.loads((directory / "planning_request.json").read_text())
    context = enrich_context(
        request["context"],
        request["seed"],
        task,
        route_band_half_width_m=request.get(
            "route_band_half_width_m", DEFAULT_ROUTE_BAND_HALF_WIDTH_M
        ),
        route_band_fraction=request.get("route_band_fraction", DEFAULT_ROUTE_BAND_FRACTION),
    )
    context["scene_image"] = render_robot_context(context, directory / "robot_scene.png")
    client = CosmosClient(
        request["base_url"],
        request["model"],
        seed=request["seed"],
        response_dir=directory / "model_responses",
    )
    native_jobs = Queue()

    def native_check(context, people, plan):
        from .path_validation import inspect_native_paths

        if update_app is None:
            return inspect_native_paths(scene, context, people, plan)
        result = Future()
        native_jobs.put((context, people, plan, result))
        return result.result()

    print(
        f"SEMANTIC_ROBOT_CONTEXT start={task['spawn_world'][:2]} goal={task['goal_world'][:2]} "
        f"reference={context['spatial_guidance']['robot']['reference_path_source']} "
        f"route_band_half_width_m={context['spatial_guidance']['robot']['route_band_half_width_m']}",
        flush=True,
    )

    def run():
        return ScenarioPipeline(
            client,
            request["max_revisions"],
            native_validator=native_check if scene is not None else None,
        ).run(context, request["total_count"], directory, request["seed"])

    if update_app is None:
        plan = run()
    else:
        # Only model requests, PIL and JSON run in the worker. USD/native
        # navigation stay on the main thread, and the UI keeps responding.
        future = Future()

        def worker():
            try:
                future.set_result(run())
            except BaseException as error:
                future.set_exception(error)

        Thread(target=worker, daemon=True, name="semantic_planning").start()
        while not future.done():
            try:
                native_context, people, native_plan, result = native_jobs.get_nowait()
            except Empty:
                pass
            else:
                try:
                    from .path_validation import inspect_native_paths

                    result.set_result(
                        inspect_native_paths(scene, native_context, people, native_plan)
                    )
                except BaseException as error:
                    result.set_exception(error)
            update_app()
        plan = future.result()
    people = json.loads((directory / "participant_plan.json").read_text())
    story = json.loads((directory / "global_scenario.json").read_text())
    print_episode_plan(request["episode_index"], story, people, plan)
    save_json(
        directory / "episode_planning.json",
        {
            "mode": "cosmos",
            "planning_pending": False,
            "model": client.model,
            "planning_seed": request["seed"],
            "robot_context_source": "actual_native_episode_task",
            "total_count": request["total_count"],
            "cosmos_count": people["cosmos_count"],
            "random_count": people["random_count"],
        },
    )
