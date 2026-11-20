"""Bounded staged generation and semantic review with inspectable artifacts."""

from __future__ import annotations

from pathlib import Path
import random
import copy
import shutil

from .cosmos.client import CosmosError
from .cosmos.planner import ScenarioPlanner
from .cosmos.reviewer import ScenarioReviewer
from .cosmos.local_revision import LocalScenarioRevision
from .schema import (
    PlanError,
    PlanningRejected,
    LocalRepairFailed,
    save_json,
    validate_global,
    validate_staffing,
    validate_participants,
    validate_scene_plan,
    validate_review,
)
from .scene_input import render_route_preview
from .runtime.compiler import plan_digest
from .variation import story_key, motion_key
from .reporting import print_story
from .navigation_quality import inspect_navigation
from .cosmos.actor_repair import replace_field
from .cosmos.route_repair import RouteCellRepair


class ScenarioPipeline:
    def __init__(self, client, max_revisions=2, stage_retries=2, native_validator=None):
        self.client = client
        self.planner, self.reviewer = ScenarioPlanner(client), ScenarioReviewer(client)
        self.local_revision = LocalScenarioRevision(client)
        self.max_revisions, self.stage_retries = max_revisions, stage_retries
        self.native_validator = native_validator

    def _native_check(self, context, people, plan):
        if self.native_validator is None:
            return
        for _ in range(24):
            problem = self.native_validator(context, people, plan)
            if problem is None:
                context["native_paths_verified"] = True
                return
            aid, sid = problem["actor_id"], problem["scene_id"]
            actor = next(
                a
                for s in plan["scenes"]
                if s["scene_id"] == sid
                for a in s["actor_plans"]
                if a["actor_id"] == aid
            )
            chosen = RouteCellRepair(self.client).choose(
                aid, str(problem["field_path"]), problem["available_cells"], None
            )
            replace_field(actor, problem["field_path"], chosen)
            validate_scene_plan(plan, people, context["grid"])
            print(
                f"SEMANTIC_NATIVE_REPAIR actor={aid} scene={sid} field={problem['field_path']} new={chosen}",
                flush=True,
            )
        raise LocalRepairFailed("Native path correction limit exceeded")

    def _stage(self, name, make, validate, directory):
        error = None
        repair = None
        for attempt in range(self.stage_retries + 1):
            print(f"SEMANTIC_STAGE {name} attempt={attempt+1}", flush=True)
            result = None
            try:
                result = make(repair)
                save_json(directory / f"{name}_attempt_{attempt+1}.json", result)
                validate(result)
                save_json(directory / f"{name}.json", result)
                return result
            except (PlanError, CosmosError, KeyError, TypeError, AttributeError) as exc:
                error = str(exc).encode("ascii", "backslashreplace").decode("ascii")
                print(
                    f"SEMANTIC_STAGE_ERROR stage={name} attempt={attempt+1} reason={error}",
                    flush=True,
                )
                save_json(
                    directory / f"{name}_error_{attempt+1}.json", {"stage": name, "error": error}
                )
                if isinstance(exc, LocalRepairFailed):
                    raise
                repair = {"validation_error": error}
                if isinstance(exc, CosmosError) and exc.response_text:
                    repair["rejected_response"] = exc.response_text[:8000]
                elif result is not None:
                    repair["rejected_response"] = result
        raise PlanningRejected(f"Stage {name} failed after bounded retries: {error}")

    def run(self, context, total_count, output_dir, seed=43):
        if type(total_count) is not int or total_count < 1:
            raise PlanError("total_count must be a positive integer")
        directory = Path(output_dir).resolve()
        directory.mkdir(parents=True, exist_ok=True)
        # Invalidate stale approval even if preparing new inputs fails.
        save_json(directory / "approval.json", {"approved": False, "status": "planning"})
        context = copy.deepcopy(context)
        context["placement_population_count"] = total_count
        for key, name in (("scene_image", "grid_scene.png"), ("world_image", "world_scene.png")):
            if key in context:
                source = Path(context[key]).resolve()
                target = directory / name
                if source != target:
                    shutil.copy2(source, target)
                context[key] = str(target)
        save_json(directory / "scene_context.json", context)
        recent = context.get("recent_scenarios", [])

        def check_story(document):
            validate_global(document)
            if any(story_key(document) == previous.get("story_sha256") for previous in recent):
                raise PlanError(
                    "Duplicate storyline from a previous episode; invent a different situation"
                )

        def check_plan(document):
            validate_scene_plan(document, participants, context["grid"])
            if any(motion_key(document) == previous.get("motion_sha256") for previous in recent):
                raise PlanError(
                    "Duplicate activity routes from a previous episode; create a fresh scene plan"
                )

        def create_cast(repair):
            # Global drafting needs the prior story and critique, not the full
            # grid routes/actor plans whose JSON shape belongs to another stage.
            story_feedback = {
                key: repair[key]
                for key in ("previous_global_scenario", "semantic_review")
                if repair and key in repair
            }
            story = self._stage(
                "global_scenario",
                lambda e: self.planner.draft_global_scenario(
                    context,
                    {**story_feedback, "format_repair": e} if e else story_feedback or None,
                ),
                check_story,
                directory,
            )
            print_story(story)
            staffing = self._stage(
                "staffing",
                lambda e: self.planner.estimate_participants(story, total_count, e),
                lambda d: validate_staffing(d, total_count),
                directory,
            )
            selected = sorted(
                random.Random(seed).sample(range(total_count), staffing["cosmos_count"])
            )
            random_ids = sorted(set(range(total_count)) - set(selected))
            participants = self._stage(
                "participant_plan",
                lambda e: self.planner.assign_roles(
                    story, staffing, selected, random_ids, e, context
                ),
                lambda d: validate_participants(d, total_count, selected),
                directory,
            )
            return story, participants

        feedback = None
        redraft = True
        for revision in range(self.max_revisions + 1):
            print(f"SEMANTIC_REVISION round={revision+1}/{self.max_revisions+1}", flush=True)
            if redraft:
                story, participants = create_cast(feedback)
            else:

                def check_revision(document):
                    validate_participants(
                        document["participant_plan"],
                        total_count,
                        [p["actor_id"] for p in participants["participants"]],
                    )
                    validate_scene_plan(
                        document["scene_plan"], document["participant_plan"], context["grid"]
                    )

                corrected = self._stage(
                    "local_revision",
                    lambda e: self.local_revision.revise(
                        context, story, participants, plan, feedback["semantic_review"]
                    ),
                    check_revision,
                    directory,
                )
                participants, plan = corrected["participant_plan"], corrected["scene_plan"]
                check_plan(plan)
                save_json(directory / "participant_plan.json", participants)
                save_json(directory / "scene_plan.json", plan)
            save_json(directory / f"global_scenario_{revision:02d}.json", story)
            save_json(directory / f"participant_plan_{revision:02d}.json", participants)
            if redraft:
                plan = self._stage(
                    "scene_plan",
                    lambda e: self.planner.generate_scene_plan(
                        context,
                        story,
                        participants,
                        {"validation_error": e, "prior_feedback": feedback} if e else feedback,
                    ),
                    check_plan,
                    directory,
                )
            for _ in range(12):
                self._native_check(context, participants, plan)
                early_quality = inspect_navigation(context, participants, plan)
                if early_quality["passed"] or any(
                    "field_path" not in i for i in early_quality["issues"]
                ):
                    break
                for issue in early_quality["issues"]:
                    if not issue["available_cells"]:
                        raise LocalRepairFailed(
                            "No feasible moving destination in the robot travel band"
                        )
                    actor = next(
                        a
                        for s in plan["scenes"]
                        if s["scene_id"] == issue["scene_id"]
                        for a in s["actor_plans"]
                        if a["actor_id"] == issue["actor_id"]
                    )
                    chosen = RouteCellRepair(self.client).choose(
                        issue["actor_id"], str(issue["field_path"]), issue["available_cells"], None
                    )
                    replace_field(actor, issue["field_path"], chosen)
                    print(
                        f"SEMANTIC_MOVEMENT_REPAIR actor={issue['actor_id']} goal={chosen}",
                        flush=True,
                    )
            else:
                raise LocalRepairFailed("Moving traffic correction limit exceeded")
            save_json(directory / "scene_plan.json", plan)
            save_json(directory / f"scene_plan_{revision:02d}.json", plan)
            image = render_route_preview(
                context, plan, directory / f"planned_routes_{revision:02d}.png"
            )
            quality = inspect_navigation(context, participants, plan)
            save_json(directory / f"navigation_quality_{revision:02d}.json", quality)
            save_json(directory / "navigation_quality.json", quality)
            if quality["passed"]:
                review = self._stage(
                    "review",
                    lambda e: self.reviewer.review(context, story, participants, plan, image, e),
                    lambda d: validate_review(d, participants),
                    directory,
                )
            else:
                review = {"passed": False, "issues": quality["issues"]}
                print(
                    f"SEMANTIC_NAVIGATION_REPAIR actors={[i['actor_id'] for i in review['issues']]}",
                    flush=True,
                )
            save_json(directory / f"review_{revision:02d}.json", review)
            if review["passed"]:
                save_json(
                    directory / "approval.json",
                    {
                        "approved": True,
                        "status": "semantically_reviewed",
                        "revision": revision,
                        "model": self.client.model,
                        "seed": seed,
                        "digest_version": 2,
                        "content_sha256": plan_digest(
                            story,
                            participants,
                            plan,
                            context["grid"],
                            {key: context.get(key) for key in ("robot_task", "spatial_guidance")},
                        ),
                        "execution_validation": (
                            "Native paths verified during planning and checked again when loading"
                            if context.get("native_paths_verified")
                            else "Native paths are checked when loading into Isaac Sim"
                        ),
                    },
                )
                return plan
            feedback = {
                "previous_plan": plan,
                "previous_global_scenario": story,
                "semantic_review": review,
            }
            redraft = any(
                issue.get("actor_id") is None or issue.get("scope") == "global_scenario"
                for issue in review["issues"]
            )
        save_json(directory / "approval.json", {"approved": False, "status": "review_failed"})
        raise PlanningRejected(
            "Cosmos review rejected the plan after the revision limit; no execution authorized"
        )
