#!/usr/bin/env python3
"""Generate and review plans through Cosmos; does not start Isaac Sim."""

from run_scenario import _add_package_to_path

_add_package_to_path()

import argparse
import json
from pathlib import Path

from isaac_composer.semantic_scenario.cosmos.client import CosmosClient
from isaac_composer.semantic_scenario.pipeline import ScenarioPipeline
from isaac_composer.semantic_scenario.scene_input import load_context
from isaac_composer.semantic_scenario.spatial import DEFAULT_ROUTE_BAND_HALF_WIDTH_M, enrich_context


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--context-dir", type=Path, required=True)
    parser.add_argument("--total-count", type=int, default=10)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--base-url", default="http://172.17.0.2:8000")
    parser.add_argument("--model")
    parser.add_argument(
        "--robot-task-json",
        type=Path,
        help="An actual native robot_task.json for robot-relative planning",
    )
    parser.add_argument(
        "--route-band-half-width-m", type=float, default=DEFAULT_ROUTE_BAND_HALF_WIDTH_M
    )
    parser.add_argument(
        "--structured-output",
        action="store_true",
        help="Use schema decoding when supported by the server",
    )
    parser.add_argument("--seed", type=int, default=43)
    parser.add_argument("--max-revisions", type=int, default=2)
    parser.add_argument("--max-tokens", type=int, default=4096)
    parser.add_argument(
        "--request",
        default="Generate plausible daily activities appropriate to the visible environment.",
    )
    args = parser.parse_args()
    if args.max_revisions < 0:
        parser.error("--max-revisions must be nonnegative")
    context = load_context(args.context_dir, args.request)
    task = json.loads(args.robot_task_json.read_text()) if args.robot_task_json else None
    context = enrich_context(
        context, args.seed, task, route_band_half_width_m=args.route_band_half_width_m
    )
    client = CosmosClient(
        args.base_url,
        args.model,
        max_tokens=args.max_tokens,
        structured_output=args.structured_output,
        response_dir=args.output_dir / "model_responses",
    )
    ScenarioPipeline(client, args.max_revisions).run(
        context, args.total_count, args.output_dir, args.seed
    )
    print(f"SEMANTIC_PLAN_REVIEWED {args.output_dir.resolve()}", flush=True)


if __name__ == "__main__":
    main()
