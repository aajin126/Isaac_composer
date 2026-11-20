"""Execute an actual reviewed Cosmos plan in Isaac; parent checks durable evidence."""

import argparse
import json
import math
from pathlib import Path
import subprocess
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from isaac_composer import runtime
from isaac_composer.semantic_scenario.runtime.compiler import load_reviewed_plan


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--plan-dir", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--duration-sec", type=float, default=180)
    parser.add_argument("--worker", action="store_true", help=argparse.SUPPRESS)
    args = parser.parse_args()
    root = args.output_dir.resolve()
    if not args.worker:
        root.mkdir(parents=True, exist_ok=True)
        completed = subprocess.run(
            [
                sys.executable,
                str(Path(__file__).resolve()),
                "--worker",
                "--plan-dir",
                str(args.plan_dir.resolve()),
                "--output-dir",
                str(root),
                "--duration-sec",
                str(args.duration_sec),
            ]
        )
        if completed.returncode:
            raise SystemExit(completed.returncode)
        report = json.loads((root / "report.json").read_text())
        evidence = json.loads((root / "native_evidence.json").read_text())
        assert report["status"] == "complete", report
        assert evidence["semantic_scenario"]["completed"], evidence
        _, participants, _ = load_reviewed_plan(args.plan_dir)
        assert len(evidence["people"]) == participants["total_count"]
        assert (
            sum(p["planner_source"] == "cosmos" for p in evidence["people"])
            == participants["cosmos_count"]
        )
        events = [
            json.loads(line)
            for line in (root / "scenario/execution_events.jsonl").read_text().splitlines()
        ]
        assert any(e["event"] == "activity_completed" for e in events), events
        print("NATIVE_SEMANTIC_VERIFIED", root, flush=True)
        return

    context, participants, plan = load_reviewed_plan(args.plan_dir)
    original_load = runtime.load_scenario_bundle
    original_step = runtime.SimulationRunner.step
    original_shutdown = runtime.SimulationRunner.shutdown
    observed_lookaround = set()
    initial_positions = {}

    def load(name):
        bundle = original_load(name)
        bundle["robots"][0]["preset"]["sensors"]["items"] = []
        bundle["simulation"]["application"].update(width=320, height=240)
        cells = {c["id"]: c["target_world"] for c in context["grid"]["cells"]}
        starts = {}
        for scene in plan["scenes"]:
            for actor in scene["actor_plans"]:
                starts.setdefault(actor["actor_id"], cells[actor["start_cell"]])
        # Fixed A2 stays far from actor starts; this test sends no robot actions.
        pose = max(
            cells.values(), key=lambda p: min(math.dist(p[:2], s[:2]) for s in starts.values())
        )
        bundle["robots"][0]["instance"]["pose"]["position"] = [pose[0], pose[1], 0.65]
        return bundle

    def step(self):
        original_step(self)
        for index, person in enumerate(self.people):
            initial_positions.setdefault(index, person.state.position.tolist())
            if getattr(person, "_semantic_lookaround", False):
                observed_lookaround.add(index)

    def run(self, max_frames=0, duration_sec=0):
        self.setup()
        while self.simulation_app.is_running():
            self.step()
            executor = getattr(self.people[0], "_semantic_executor", None) if self.people else None
            if executor is not None and executor.diagnostics()["completed"]:
                return "semantic_complete"
            if (
                self.episode_start_sim_time is not None
                and self.world.current_time - self.episode_start_sim_time >= duration_sec
            ):
                return "duration"
        return "closed"

    def shutdown(self):
        if self.people:
            executor = self.people[0]._semantic_executor
            config = self.bundle["pedestrians"]["people"]
            evidence = {
                "semantic_scenario": executor.diagnostics(),
                "lookaround_observed_actor_ids": sorted(observed_lookaround),
                "people": [
                    {
                        "actor_id": i,
                        "start": initial_positions.get(i),
                        "end": p.state.position.tolist(),
                        "planner_source": c["planner_source"],
                    }
                    for i, (p, c) in enumerate(zip(self.people, config))
                ],
            }
            (root / "native_evidence.json").write_text(json.dumps(evidence, indent=2))
        original_shutdown(self)

    runtime.load_scenario_bundle = load
    runtime.SimulationRunner.step = step
    runtime.SimulationRunner.run = run
    runtime.SimulationRunner.shutdown = shutdown
    runtime.main(
        [
            "warehouse_a2_random",
            "--headless",
            "--robot-task",
            "false",
            "--seed",
            "43",
            "--semantic-plan-dir",
            str(args.plan_dir.resolve()),
            "--duration-sec",
            str(args.duration_sec),
            "--generation-output-dir",
            str(root / "scenario"),
            "--run-report",
            str(root / "report.json"),
        ]
    )


if __name__ == "__main__":
    main()
