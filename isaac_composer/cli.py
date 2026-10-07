"""Command-line checks that do not start Isaac Sim."""

from __future__ import annotations

import argparse

from .config import load_scenario_bundle
from .navigation import build_navigation_plan


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Validate an Isaac Composer scenario and all its presets"
    )
    parser.add_argument("scenario", help="Scenario name or YAML path")
    args = parser.parse_args()
    bundle = load_scenario_bundle(args.scenario)
    scenario = bundle["scenario"]
    navigation = build_navigation_plan(bundle)
    robots = ",".join(
        item["instance"]["instance_name"] for item in bundle["robots"]
    )
    print(
        f"valid scenario={scenario['name']} "
        f"environment={bundle['environment']['name']} "
        f"pedestrians={len(bundle['pedestrians']['people'])} "
        f"robots={robots} "
        f"navigation={'enabled' if navigation.enabled else 'disabled'} "
        f"nav_instances={len(navigation.instances)}"
    )


if __name__ == "__main__":
    main()
