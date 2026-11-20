"""Semantic pipeline contracts and arrival-driven execution, without Isaac."""

import copy
import json
from types import SimpleNamespace

import numpy as np
import pytest
from PIL import Image

from isaac_composer.semantic_scenario.cosmos.client import (
    CosmosClient,
    CosmosError,
    parse_json_answer,
)
from isaac_composer.semantic_scenario.pipeline import ScenarioPipeline
from isaac_composer.semantic_scenario.runtime.compiler import (
    compile_scene_plan,
    load_reviewed_plan,
    build_mixed_people,
    save_execution_inputs,
    animation_query_scene,
)
from isaac_composer.semantic_scenario.runtime.executor import SceneExecutor
from isaac_composer.semantic_scenario.schema import (
    PlanError,
    PlanningRejected,
    english_json,
    validate_staffing,
    validate_scene_plan,
    validate_participants,
    validate_global,
    validate_review,
)
from isaac_composer.semantic_scenario.scene_input import load_context
from isaac_composer.pedestrian_routes import PedestrianRoutePlanner


class Floor:
    def project_world(self, p):
        return p

    def is_allowed_world(self, p):
        return True

    def shortest_path(self, start, end):
        return [start, end]


class Person:
    def __init__(self):
        self._target_speed = 1.0
        self._current_point_index = 99

    def update_target_position(self, path, walk_speed):
        self.path, self._target_speed = path, walk_speed


@pytest.fixture
def sample(tmp_path):
    image = tmp_path / "grid_scene.png"
    Image.new("RGB", (600, 200), "gray").save(image)
    grid = {
        "environment": "warehouse",
        "bounds_xy": [0, 0, 6, 2],
        "cell_size_m": 2,
        "cells": [
            {"id": f"{col}1", "target_world": [x, 1, 0], "bounds_xy": [x - 1, 0, x + 1, 2]}
            for col, x in zip("ABC", [1, 3, 5])
        ],
    }
    context = {
        "environment": "warehouse",
        "user_request": "Plan warehouse activities.",
        "grid": grid,
        "valid_cells": ["A1", "B1", "C1"],
        "scene_image": str(image),
    }
    story = {
        "scenario_id": "inspection",
        "summary": "Inspect two storage areas.",
        "events": [{"description": "Workers inspect adjacent storage areas."}],
    }
    staff = {
        "total_count": 3,
        "minimum_cosmos_count": 2,
        "cosmos_count": 2,
        "random_count": 1,
        "justification": "Two inspectors and one background pedestrian.",
    }
    people = {
        **staff,
        "participants": [
            {"actor_id": i, "role": "inspector", "purpose": "Inspect storage"} for i in [0, 1]
        ],
        "random_actor_ids": [2],
    }
    plan = {
        "scenes": [
            {
                "scene_id": "inspection",
                "description": "Inspect adjacent areas together.",
                "start_condition": {"type": "all_participants_arrived"},
                "actor_plans": [
                    {
                        "actor_id": i,
                        "start_cell": start,
                        "subgoals": [
                            {"cell_id": target, "action": "LookAround", "duration_s": 0.4}
                        ],
                        "goal_cell": target,
                    }
                    for i, start, target in [(0, "A1", "B1"), (1, "C1", "C1")]
                ],
            }
        ]
    }
    return context, story, staff, people, plan


def test_population_half_rounds_up_and_rejects_boolean_ids(sample):
    _, _, staff, people, _ = sample
    validate_staffing(staff, 3)
    with pytest.raises(PlanError, match="between 2"):
        validate_staffing({**staff, "cosmos_count": 1, "random_count": 2}, 3)
    with pytest.raises(PlanError):
        validate_participants({**people, "random_actor_ids": [True]}, 3, [0, 1])


@pytest.mark.parametrize("total,count", [(10, 5), (10, 10), (7, 4), (1, 1)])
def test_staffing_derives_all_population_fields_from_the_model_chosen_count(sample, total, count):
    from isaac_composer.semantic_scenario.cosmos.planner import ScenarioPlanner

    _, story, *_ = sample
    client = FakeClient([{"option_id": f"cast_{count - (total + 1) // 2 + 1:03d}"}])
    result = ScenarioPlanner(client).estimate_participants(story, total)
    assert result == {
        "total_count": total,
        "minimum_cosmos_count": (total + 1) // 2,
        "cosmos_count": count,
        "random_count": total - count,
        "justification": f"Cosmos selected {count} planned pedestrians and {total - count} background pedestrians.",
    }
    validate_staffing(result, total)
    assert [o["cosmos_count"] for o in client.calls[0][1]["staffing_options"]] == list(
        range((total + 1) // 2, total + 1)
    )
    assert "option_id" in client.calls[0][1]["output_task"]


def test_staffing_drops_contradictory_derived_fields_and_echoed_feedback(sample):
    from isaac_composer.semantic_scenario.cosmos.planner import ScenarioPlanner

    _, story, *_ = sample
    response = {
        "option_id": "cast_001",
        "cosmos_count": 9,
        "justification": "Five people perform the activities.",
        "random_count": 0,
        "total_count": 5,
        "minimum_cosmos_count": 0,
        "maximum_cosmos_count": 99,
        "repair_feedback": {"validation_error": "Previous failure."},
    }
    result = ScenarioPlanner(FakeClient([response])).estimate_participants(story, 10)
    assert result["cosmos_count"] == 5 and result["random_count"] == 5
    assert result["total_count"] == 10 and result["minimum_cosmos_count"] == 5
    assert "repair_feedback" not in result and "maximum_cosmos_count" not in result
    assert response["random_count"] == 0


@pytest.mark.parametrize("choice", [None, True, "5", 5.0, "cast_000", "cast_999", []])
def test_staffing_rejects_missing_or_invalid_decisions_instead_of_inventing_counts(sample, choice):
    from isaac_composer.semantic_scenario.cosmos.planner import ScenarioPlanner

    _, story, *_ = sample
    client = FakeClient([{"option_id": choice}])
    with pytest.raises(PlanError, match="Choose option_id"):
        ScenarioPlanner(client).estimate_participants(story, 10)


def test_repeated_invalid_staffing_is_a_recoverable_candidate_rejection(sample, tmp_path):
    _, story, *_ = sample
    client = FakeClient([{"cosmos_count": 3}] * 3)
    pipeline = ScenarioPipeline(client)
    with pytest.raises(PlanningRejected, match="Stage staffing failed"):
        pipeline._stage(
            "staffing",
            lambda feedback: pipeline.planner.estimate_participants(story, 10, feedback),
            lambda value: validate_staffing(value, 10),
            tmp_path,
        )
    assert len(client.calls) == 3
    for _, payload, _ in client.calls:
        assert all(o["cosmos_count"] >= 5 for o in payload["staffing_options"])
        assert "repair_feedback" not in payload
        assert "rejected_response" not in payload


def test_english_json_rejects_non_english_values_and_keys():
    for document in ({"role": "검수"}, {"역할": "inspection"}):
        with pytest.raises(PlanError, match="English"):
            english_json(document)


def test_parser_normalizes_english_typography_before_english_validation():
    raw = json.dumps(
        {
            "summary": "Workers\u2019 checks\u2014then wait\u2026",
            "events": [{"description": "Look\u00a0around near \u201cshelves\u201d."}],
        }
    )
    result = parse_json_answer(raw)
    english_json(result)
    assert result["summary"] == "Workers' checks-then wait..."
    assert result["events"][0]["description"] == 'Look around near "shelves".'


def test_parser_preserves_non_english_words_and_reports_their_field():
    result = parse_json_answer(
        json.dumps(
            {
                "events": [{"description": "Ensure smooth \u4ea4\u63a5 between loads."}],
            }
        )
    )
    assert "\u4ea4\u63a5" in result["events"][0]["description"]
    with pytest.raises(PlanError, match=r"U\+4EA4 at \$\.events\[0\]\.description"):
        english_json(result)


def test_typography_normalization_cannot_silently_merge_keys():
    with pytest.raises(CosmosError, match="valid JSON"):
        parse_json_answer(json.dumps({"worker\u2019s": 1, "worker's": 2}))


def test_unknown_cell_missing_actor_and_overlapping_scene_ownership(sample):
    context, _, _, people, plan = sample
    bad = copy.deepcopy(plan)
    bad["scenes"][0]["actor_plans"][0]["goal_cell"] = "Z99"
    with pytest.raises(PlanError, match="Unknown"):
        validate_scene_plan(bad, people, context["grid"])
    bad = copy.deepcopy(plan)
    bad["scenes"][0]["actor_plans"].pop()
    with pytest.raises(PlanError, match="Every Cosmos"):
        validate_scene_plan(bad, people, context["grid"])
    bad = copy.deepcopy(plan)
    other = copy.deepcopy(plan["scenes"][0])
    other["scene_id"] = "next_inspection"
    bad["scenes"].append(other)
    with pytest.raises(PlanError, match="explicit dependency"):
        validate_scene_plan(bad, people, context["grid"])
    other["start_condition"] = {"type": "after_scene", "scene_id": "inspection"}
    validate_scene_plan(bad, people, context["grid"])


def test_native_compiler_rejects_snapping_and_unreachable_paths(sample):
    context, _, _, people, plan = sample

    class Snapped(Floor):
        def project_world(self, p):
            return [p[0] + 0.02, p[1], p[2]]

    with pytest.raises(PlanError, match="valid center"):
        compile_scene_plan(Snapped(), context, people, plan)

    class Disconnected(Floor):
        def shortest_path(self, start, end):
            return [start]

    with pytest.raises(PlanError, match="both requested endpoints"):
        compile_scene_plan(Disconnected(), context, people, plan)


def test_idle_only_actor_can_be_spawned_without_invented_walk(sample):
    context, _, _, people, plan = sample
    compiled = compile_scene_plan(Floor(), context, people, plan)
    planner = PedestrianRoutePlanner(Floor(), {"speed_range_mps": [1, 1], "min_route_length_m": 1})
    samples = iter([[1, 5, 0], [5, 5, 0]])
    planner._sample = lambda: next(samples)
    roster = build_mixed_people(compiled, planner)
    assert [p["planner_source"] for p in roster] == ["cosmos", "cosmos", "random"]
    assert roster[1]["pose"]["position"] == [5, 1, 0]
    assert roster[1]["pose"]["yaw"] == 0
    assert not roster[0]["repeat"]


def test_synchronized_dwell_waits_for_arrival_and_resets_path_index(sample, tmp_path):
    context, _, _, people, plan = sample
    compiled = compile_scene_plan(Floor(), context, people, plan)
    compiled["repeat_semantic_plan"] = False  # Inspect a single execution of the barrier.
    actors = [Person(), Person(), Person()]
    executor = SceneExecutor(Floor(), actors, compiled, tmp_path)
    positions = np.array([[1, 1, 0], [5, 1, 0], [1, 5, 0]], float)
    for _ in range(20):
        executor.step(0.1, positions)
    assert not any(e["event"] == "activity_started" for e in executor.events)
    assert actors[0]._current_point_index == 0
    assert actors[0]._target_speed == 1
    positions[0] = [3, 1, 0]
    for _ in range(10):
        executor.step(0.1, positions)
    assert executor.diagnostics()["completed"]
    starts = [e for e in executor.events if e["event"] == "activity_started"]
    assert len(starts) == 2 and starts[0]["time_s"] == starts[1]["time_s"]
    assert all(p._target_speed == 0 for p in actors[:2])
    executor.close()
    events = [
        json.loads(line) for line in (tmp_path / "execution_events.jsonl").read_text().splitlines()
    ]
    assert events[-1]["completed"] is True


def test_dwell_pauses_when_actor_is_displaced(sample):
    context, _, _, people, plan = sample
    plan["scenes"][0]["actor_plans"][0]["subgoals"][0]["duration_s"] = 5
    executor = SceneExecutor(
        Floor(), [Person(), Person(), Person()], compile_scene_plan(Floor(), context, people, plan)
    )
    positions = np.array([[3, 1, 0], [5, 1, 0], [1, 5, 0]], float)
    for _ in range(5):
        executor.step(0.1, positions)
    state = executor.scenes["inspection"]["actors"][0]
    dwell = state["dwell_s"]
    positions[0] = [1, 1, 0]
    for _ in range(30):
        executor.step(0.1, positions)
    assert state["dwell_s"] == dwell
    assert executor.action(0) is None


class FakeClient:
    model = "test_cosmos"

    def __init__(self, answers):
        self.answers, self.calls = iter(answers), []

    def complete(self, instruction, payload, images=(), schema=None, temperature=0.3):
        self.calls.append((instruction, copy.deepcopy(payload), list(images)))
        return copy.deepcopy(next(self.answers))


def _scene_answers(plan):
    outline = {
        "scenes": [
            {
                **{k: v for k, v in scene.items() if k != "actor_plans"},
                "actor_ids": [actor["actor_id"] for actor in scene["actor_plans"]],
            }
            for scene in plan["scenes"]
        ]
    }
    answers, seen = [outline], set()
    for scene in plan["scenes"]:
        for actor in scene["actor_plans"]:
            if actor["actor_id"] not in seen:
                answers.append({"start_cell": actor["start_cell"]})
                seen.add(actor["actor_id"])
            answers.append(actor)
    return answers


def test_pipeline_story_precedes_staffing_review_revision_and_hash_guard(sample, tmp_path):
    context, story, staff, people, plan = sample
    # Seed 1 selects actor IDs 0 and 2 for N=3; use the deterministic IDs.
    people["participants"][1]["actor_id"] = 2
    people["random_actor_ids"] = [1]
    plan["scenes"][0]["actor_plans"][1]["actor_id"] = 2
    failed = {
        "passed": False,
        "issues": [
            {
                "actor_id": 0,
                "problem": "Visit is too short.",
                "requested_change": "Extend the visit.",
            }
        ],
    }
    fixed = copy.deepcopy(plan)
    fixed["scenes"][0]["actor_plans"][0]["subgoals"][0]["duration_s"] = 8

    def staged(scene_plan):
        return _scene_answers(scene_plan)

    client = FakeClient(
        [
            story,
            {"option_id": "cast_001"},
            {"participants": people["participants"]},
            *staged(plan),
            failed,
            {
                "changes": [
                    {"scene_id": "inspection", "path": ["subgoals", 0, "duration_s"], "value": 8}
                ]
            },
            {"passed": True, "issues": []},
        ]
    )
    output = tmp_path / "plan"
    ScenarioPipeline(client).run(context, 3, output, seed=1)
    assert "total_count" not in client.calls[0][1]
    assert client.calls[1][1]["staffing_options"][0]["cosmos_count"] == 2
    scene_calls = [
        p for instruction, p, _ in client.calls if instruction.startswith("Plan the scene timeline")
    ]
    assert len(scene_calls) == 1
    local_calls = [
        p
        for instruction, p, _ in client.calls
        if instruction.startswith("Correct ONLY the named actor")
    ]
    assert len(local_calls) == 1 and local_calls[0]["actor_id"] == 0
    assert local_calls[0]["review_issues"] == failed["issues"]
    review_calls = [
        images
        for instruction, _, images in client.calls
        if instruction.startswith("Independently review")
    ]
    assert all(len(images) == 1 for images in review_calls)
    assert load_reviewed_plan(output)[2] == fixed
    fixed["scenes"][0]["description"] = "Edited after review."
    (output / "scene_plan.json").write_text(json.dumps(fixed))
    with pytest.raises(PlanError, match="changed after"):
        load_reviewed_plan(output)


def test_failed_new_generation_invalidates_previous_approval(sample, tmp_path):
    context, *_ = sample
    (tmp_path / "approval.json").write_text('{"approved":true}')
    with pytest.raises(PlanError, match="bounded retries"):
        ScenarioPipeline(FakeClient([{}]), stage_retries=0).run(context, 3, tmp_path)
    assert json.loads((tmp_path / "approval.json").read_text())["approved"] is False


def test_role_generation_keeps_population_in_code_and_requests_only_roles(sample):
    from isaac_composer.semantic_scenario.cosmos.planner import ScenarioPlanner

    _, story, staff, people, _ = sample
    client = FakeClient([{"participants": people["participants"]}])
    result = ScenarioPlanner(client).assign_roles(story, staff, [0, 1], [2])
    assert result == people
    assert "population" not in client.calls[0][1]
    assert client.calls[0][1]["selected_actor_ids"] == [0, 1]
    result["justification"] = "Edited after generation."
    assert staff["justification"] != result["justification"]


def test_role_generation_rejects_repeated_population_or_wrong_ids(sample):
    from isaac_composer.semantic_scenario.cosmos.planner import ScenarioPlanner

    _, story, staff, people, _ = sample
    with pytest.raises(PlanError, match="ONLY participants"):
        ScenarioPlanner(FakeClient([people])).assign_roles(story, staff, [0, 1], [2])
    bad = copy.deepcopy(people["participants"])
    bad[1]["actor_id"] = 2
    with pytest.raises(PlanError, match="selected actor IDs"):
        ScenarioPlanner(FakeClient([{"participants": bad}])).assign_roles(story, staff, [0, 1], [2])


def test_broken_role_json_reports_location_and_cannot_be_partially_accepted():
    malformed = '{"participants": [{"actor_id":1}, "random_actor_ids": []}}'
    with pytest.raises(CosmosError, match=r"line 1, column"):
        parse_json_answer(malformed)


def test_wrapped_review_correction_is_lossless_and_keeps_the_failed_decision(sample):
    from isaac_composer.semantic_scenario.cosmos.reviewer import ScenarioReviewer

    context, story, _, people, plan = sample
    response = {
        "passed": False,
        "issues": [
            {
                "actor_id": 0,
                "scope": "roles",
                "problem": "The role has no concrete action.",
                "requested_change": {
                    "actor_id": 0,
                    "scope": "roles",
                    "change": "Wait beside the other worker.",
                },
            }
        ],
    }
    raw = copy.deepcopy(response)
    reviewer = ScenarioReviewer(FakeClient([response]))
    review = reviewer.review(context, story, people, plan, context["scene_image"])
    validate_review(review, people)
    assert review["passed"] is False
    assert review["issues"][0]["requested_change"] == "Wait beside the other worker."
    assert response == raw


def test_semantic_review_uses_readable_grid_and_declares_only_actual_code_checks(sample):
    from isaac_composer.semantic_scenario.cosmos.reviewer import ScenarioReviewer

    context, story, _, people, plan = sample
    client = FakeClient([{"passed": True, "issues": []}])
    ScenarioReviewer(client).review(context, story, people, plan, "route_overlay.png")
    _, payload, images = client.calls[0]
    assert images == [context["scene_image"]]
    assert payload["verified_by_code"]["all_cell_ids_valid"] is True
    assert payload["verified_by_code"]["native_paths_already_verified"] is False
    assert payload["navigation_validation_phase"] == "simulator_load"


@pytest.mark.parametrize(
    "wrapper",
    [
        {"actor_id": 1, "change": "Wait beside the other worker."},
        {"scope": "scenes", "change": "Wait beside the other worker."},
        {"change": "Wait beside the other worker.", "other_correction": "Leave the area."},
        {"change": ""},
        {"change": None},
        {},
    ],
)
def test_ambiguous_review_wrappers_trigger_repair_with_the_original_response(sample, wrapper):
    from isaac_composer.semantic_scenario.cosmos.reviewer import ScenarioReviewer

    context, story, _, people, plan = sample
    response = {
        "passed": False,
        "issues": [
            {
                "actor_id": 0,
                "scope": "roles",
                "problem": "The role has no concrete action.",
                "requested_change": wrapper,
            }
        ],
    }
    with pytest.raises(CosmosError, match=r"issues\[0\].requested_change") as caught:
        ScenarioReviewer(FakeClient([response])).review(
            context, story, people, plan, context["scene_image"]
        )
    assert json.loads(caught.value.response_text) == response


def test_empty_review_correction_is_not_filled_in_or_treated_as_approval(sample):
    _, _, _, people, _ = sample
    response = {
        "passed": False,
        "issues": [
            {
                "actor_id": 0,
                "problem": "No action specified.",
                "requested_change": "",
            }
        ],
    }
    with pytest.raises(PlanError, match=r"issues\[0\].requested_change.*STRING"):
        validate_review(response, people)
    assert response["passed"] is False


def test_global_json_repair_sees_the_rejected_answer_and_keeps_the_full_contract(sample, tmp_path):
    context, story, *_ = sample
    malformed = (
        '{"scenario_id":"test","summary":"Workers meet.","events":[{"description":"Wait."],{}]}'
    )
    client = CosmosClient("http://example.test:8000", model="cosmos")
    answers = iter([malformed, json.dumps(story)])
    requests = []

    def request(path, body):
        requests.append(body)
        return {"choices": [{"finish_reason": "stop", "message": {"content": next(answers)}}]}

    client._request = request
    pipeline = ScenarioPipeline(client)
    result = pipeline._stage(
        "global_scenario",
        lambda repair: pipeline.planner.draft_global_scenario(
            context, {"format_repair": repair} if repair else None
        ),
        validate_global,
        tmp_path,
    )
    assert result == story
    text = requests[1]["messages"][1]["content"][-1]["text"]
    payload = json.loads(text.split("\nINPUT:\n", 1)[1])
    repair = payload["repair_feedback"]["format_repair"]
    assert repair["rejected_response"] == malformed
    assert "Expecting ',' delimiter" in repair["validation_error"]
    assert "ALL required output fields" in text
    assert "WITHIN THIS SPECIFIC ENVIRONMENT" in text


def test_missing_global_events_are_returned_to_the_model_for_repair(sample, tmp_path):
    context, story, *_ = sample
    missing = {"scenario_id": "test", "summary": "Workers meet."}
    client = FakeClient([missing, story])
    pipeline = ScenarioPipeline(client)
    assert (
        pipeline._stage(
            "global_scenario",
            lambda repair: pipeline.planner.draft_global_scenario(context, repair),
            validate_global,
            tmp_path,
        )
        == story
    )
    assert client.calls[1][1]["repair_feedback"]["rejected_response"] == missing


def test_parser_rejects_partial_reasoning_and_accepts_answer_wrappers():
    assert parse_json_answer('<think>Reasoning</think><answer>{"passed":true}</answer>') == {
        "passed": True
    }
    assert parse_json_answer('```json\n{"passed":true}\n```')["passed"]
    with pytest.raises(CosmosError, match="unfinished"):
        parse_json_answer('<think>{"passed":true}')


def test_client_uses_fresh_context_and_rejects_truncation(tmp_path):
    client = CosmosClient("http://example.test:8000", model="cosmos")
    calls = []

    def request(path, body):
        calls.append(body)
        return {"choices": [{"finish_reason": "length", "message": {"content": '{"passed":true}'}}]}

    client._request = request
    with pytest.raises(CosmosError, match="max_tokens"):
        client.complete("Review independently.", {"scene": "warehouse"})
    assert client.base_url.endswith("/v1")
    assert [m["role"] for m in calls[0]["messages"]] == ["system", "user"]


def test_failed_scene_export_cannot_reuse_stale_image(sample, tmp_path):
    (tmp_path / "export_status.json").write_text('{"status":"failed"}')
    with pytest.raises(PlanError, match="did not complete"):
        load_context(tmp_path)


def test_failed_input_preparation_invalidates_previous_approval(sample, tmp_path):
    context, *_ = sample
    context["scene_image"] = str(tmp_path / "missing.png")
    output = tmp_path / "old_plan"
    output.mkdir()
    (output / "approval.json").write_text('{"approved":true}')
    with pytest.raises(FileNotFoundError):
        ScenarioPipeline(FakeClient([])).run(context, 3, output)
    assert json.loads((output / "approval.json").read_text())["approved"] is False


def test_episode_copy_keeps_scene_images_and_rejects_recursive_output(sample, tmp_path):
    context, *_ = sample
    source = tmp_path / "plan_source"
    source.mkdir()
    (source / "scene_plan.json").write_text('{"scenes":[]}')
    episode = tmp_path / "episode/scenario"
    save_execution_inputs(episode, source, context, {"source": "cosmos"})
    copied = json.loads((episode / "semantic_plan/scene_context.json").read_text())
    assert copied["scene_image"] != context["scene_image"]
    assert Image.open(copied["scene_image"]).size == (600, 200)
    with pytest.raises(PlanError, match="outside"):
        save_execution_inputs(source / "episode", source, context, {})


def test_parser_rejects_duplicate_keys_instead_of_discarding_plan_fields():
    with pytest.raises(CosmosError, match="valid JSON"):
        parse_json_answer('{"passed":false,"passed":true}')


def test_outline_cannot_silently_drop_a_cosmos_actor(sample):
    context, story, _, people, _ = sample
    from isaac_composer.semantic_scenario.cosmos.scene_planner import ScenePlanner

    client = FakeClient([{"scenes": [{"scene_id": "incomplete", "actor_ids": [0]}]}])
    with pytest.raises(PlanError, match="every Cosmos"):
        ScenePlanner(client).generate(context, story, people)
    assert '"actor_ids":[0, 1]' in client.calls[0][0]


def test_actor_repairs_keep_valid_peers_and_explicit_remaining_candidates(sample):
    from isaac_composer.semantic_scenario.cosmos.scene_planner import ScenePlanner

    context, story, _, people, plan = sample
    first, second = copy.deepcopy(plan["scenes"][0]["actor_plans"])
    outline = {
        "scenes": [
            {
                "scene_id": "inspection",
                "description": "Inspect adjacent storage areas.",
                "start_condition": {"type": "episode_started"},
                "actor_ids": [0, 1],
            }
        ]
    }
    invalid_goal = {**second, "goal_cell": "B3"}
    client = FakeClient(
        [
            outline,
            {"start_cell": "A1"},
            first,
            {"start_cell": "C1"},
            invalid_goal,
            {"cell_id": "C1"},
        ]
    )
    result = ScenePlanner(client).generate(context, story, people)
    assert result["scenes"][0]["actor_plans"] == [first, second]
    assert len(client.calls) == 6
    instruction, payload, _ = client.calls[-2]
    assert "NEVER invent a cell ID" in instruction
    assert payload["allowed_start_cells"] == ["C1"]
    selection = [
        p for instruction, p, _ in client.calls if instruction.startswith("Choose an initial")
    ]
    assert selection[1]["available_start_cells"] == ["B1", "C1"]
    assert payload["valid_cells"] == ["A1", "B1", "C1"]
    assert payload["selected_start_cell"] == "C1"
    correction = client.calls[-1][1]
    assert correction["field"] == "goal_cell" and correction["available_cells"] == [
        "A1",
        "B1",
        "C1",
    ]
    assert "B3" not in json.dumps(correction)
    assert len([c for c in client.calls if c[0].startswith("Plan ONLY")]) == 2


def test_start_selector_only_sees_remaining_cells_even_during_repairs():
    from isaac_composer.semantic_scenario.cosmos.start_selector import StartSelector

    client = FakeClient([{"start_cell": "B15"}, {"start_cell": "B1"}])
    assert StartSelector(client).choose(5, ["A1", "B1"]) == "B1"
    for _, payload, images in client.calls:
        assert payload["available_start_cells"] == ["A1", "B1"]
        assert "B15" not in json.dumps(payload)
        assert not images
        assert "valid_cells" not in payload and "previous_actor_plans" not in payload


def test_start_selector_does_not_replace_a_persistently_invalid_model_choice():
    from isaac_composer.semantic_scenario.cosmos.start_selector import StartSelector

    client = FakeClient([{"start_cell": "B15"}] * 3)
    with pytest.raises(PlanError, match="Actor 5 start selection failed"):
        StartSelector(client).choose(5, ["A1", "B1"])
    assert len(client.calls) == 3


def test_later_scenes_do_not_request_a_second_initial_spawn(sample):
    from isaac_composer.semantic_scenario.cosmos.scene_planner import ScenePlanner

    context, story, _, people, plan = sample
    later = copy.deepcopy(plan["scenes"][0])
    later["scene_id"] = "later"
    later["start_condition"] = {"type": "after_scene", "scene_id": "inspection"}
    plan["scenes"].append(later)
    client = FakeClient(_scene_answers(plan))
    result = ScenePlanner(client).generate(context, story, people)
    validate_scene_plan(result, people, context["grid"])
    selections = [
        p for instruction, p, _ in client.calls if instruction.startswith("Choose an initial")
    ]
    assert len(selections) == 2
    assert selections[1]["available_start_cells"] == ["B1", "C1"]


def test_actor_schema_excludes_undefined_queues_and_pins_front_and_start():
    from isaac_composer.semantic_scenario.cosmos.contracts import actor_route_contract

    cells = ["A1", "B1", "C1"]
    schema = actor_route_contract(cells, 3, ["C1"], {}, "episode_started")
    assert schema["properties"]["actor_id"] == {"const": 3}
    assert schema["properties"]["start_cell"]["enum"] == ["C1"]
    assert len(schema["properties"]["subgoals"]["items"]["anyOf"]) == 3
    queues = {"inspection": {"spots": ["A1", "B1"]}}
    schema = actor_route_contract(cells, 3, ["C1"], queues, "episode_started")
    step = schema["properties"]["subgoals"]["items"]["anyOf"][3]["properties"]
    assert step["cell_id"] == {"const": "A1"}
    assert step["queue_id"] == {"const": "inspection"}


def test_invalid_actor_response_is_rejected_even_after_prompt_repair(sample):
    from isaac_composer.semantic_scenario.cosmos.scene_planner import ScenePlanner

    context, story, _, people, plan = sample
    actors = copy.deepcopy(plan["scenes"][0]["actor_plans"])
    outline = {
        "scenes": [
            {
                "scene_id": "inspection",
                "description": "Inspect storage.",
                "start_condition": {"type": "episode_started"},
                "actor_ids": [0, 1],
            }
        ]
    }
    actors[1]["goal_cell"] = "B3"
    client = FakeClient(
        [
            outline,
            {"start_cell": "A1"},
            actors[0],
            {"start_cell": "C1"},
            actors[1],
            {"cell_id": "B3"},
            {"cell_id": "B3"},
            {"cell_id": "B3"},
        ]
    )
    with pytest.raises(PlanningRejected, match="field goal_cell replacement failed"):
        ScenePlanner(client).generate(context, story, people)
    assert len([c for c in client.calls if c[0].startswith("Plan ONLY")]) == 2


def test_subgoal_repair_changes_only_the_invalid_cell_and_keeps_activity_details():
    from isaac_composer.semantic_scenario.cosmos.route_repair import RouteCellRepair

    actor = {
        "actor_id": 7,
        "start_cell": "A1",
        "subgoals": [
            {"cell_id": "J1", "action": "LookAround", "duration_s": 17},
            {"cell_id": "B1", "action": "Idle", "duration_s": 4},
        ],
        "goal_cell": "C1",
    }
    expected = copy.deepcopy(actor)
    expected["subgoals"][0]["cell_id"] = "B1"
    client = FakeClient([{"cell_id": "B1"}])
    RouteCellRepair(client).repair(actor, ["A1", "B1", "C1"])
    assert actor == expected
    assert client.calls[0][1]["preceding_cell"] == "A1"
    assert "J1" not in json.dumps(client.calls[0][1])


def test_repeated_invalid_destination_reuses_the_same_cosmos_choice():
    from isaac_composer.semantic_scenario.cosmos.route_repair import RouteCellRepair

    actor = {
        "actor_id": 7,
        "start_cell": "A1",
        "subgoals": [{"cell_id": "J1", "action": "Idle", "duration_s": 5}],
        "goal_cell": "J1",
    }
    client = FakeClient([{"cell_id": "B1"}])
    RouteCellRepair(client).repair(actor, ["A1", "B1", "C1"])
    assert actor["subgoals"] == [{"cell_id": "B1", "action": "Idle", "duration_s": 5}]
    assert actor["goal_cell"] == "B1"
    assert len(client.calls) == 1


def test_idle_root_drift_allowance_keeps_planning_bounds_and_obstacles():
    scene = Floor()
    scene.allowed_region_xy = (-9.5, -11.4, 8.5, 17)
    scene.occupancy_region = object()
    scene.navmesh = object()
    scene.agent_radius = 0.3
    scene.routing_radius_m = 0.35
    query = animation_query_scene(scene, 0.02)
    assert query.allowed_region_xy == (-9.52, -11.42, 8.52, 17.02)
    assert scene.allowed_region_xy == (-9.5, -11.4, 8.5, 17)
    assert query.navmesh is scene.navmesh
    assert query.occupancy_region is scene.occupancy_region
    assert query.agent_radius == 0.3 and query.routing_radius_m == 0.35


def test_native_queue_schema_requires_real_slots_and_front_service(sample):
    context, _, _, people, plan = sample
    scene = plan["scenes"][0]
    scene["start_condition"] = {"type": "episode_started"}
    plan["queues"] = [{"queue_id": "inspection", "spots": ["B1", "C1"]}]
    for actor in scene["actor_plans"]:
        actor["goal_cell"] = "A1"
        actor["subgoals"] = [
            {
                "cell_id": "B1",
                "action": "Queue",
                "queue_id": "inspection",
                "service_action": "LookAround",
                "duration_s": 1,
            }
        ]
    validate_scene_plan(plan, people, context["grid"])
    compiled = compile_scene_plan(Floor(), context, people, plan)
    assert compiled["queues"][0]["positions"] == [[3.0, 1.0, 0.0], [5.0, 1.0, 0.0]]
    scene["start_condition"] = {"type": "all_participants_arrived"}
    with pytest.raises(PlanError, match="front together"):
        validate_scene_plan(plan, people, context["grid"])
    scene["start_condition"] = {"type": "episode_started"}
    plan["queues"][0]["spots"] = ["B1"]
    with pytest.raises(PlanError, match="enough spots"):
        validate_scene_plan(plan, people, context["grid"])


def test_previous_story_and_routes_trigger_new_generation_not_id_renaming(sample, tmp_path):
    from isaac_composer.semantic_scenario.variation import history_entry

    context, story, staff, people, plan = sample
    context["recent_scenarios"] = [history_entry(0, story, plan)]
    novel_story = copy.deepcopy(story)
    novel_story["summary"] = "Coworkers meet after completing their rounds."
    people["participants"][1]["actor_id"] = 2
    people["random_actor_ids"] = [1]
    plan["scenes"][0]["actor_plans"][1]["actor_id"] = 2
    changed = copy.deepcopy(plan)
    changed["scenes"][0]["actor_plans"][0]["goal_cell"] = "A1"

    def stages(value):
        return _scene_answers(value)

    client = FakeClient(
        [
            story,
            novel_story,
            {"option_id": "cast_001"},
            {"participants": people["participants"]},
            *stages(plan),
            *stages(changed),
            {"passed": True, "issues": []},
        ]
    )
    output = tmp_path / "new_episode"
    ScenarioPipeline(client, max_revisions=0).run(context, 3, output, seed=1)
    assert (
        "Duplicate storyline"
        in json.loads((output / "global_scenario_error_1.json").read_text())["error"]
    )
    assert (
        "Duplicate activity routes"
        in json.loads((output / "scene_plan_error_1.json").read_text())["error"]
    )
    assert client.calls[0][1]["recent_scenarios"][0]["scenario_id"] == story["scenario_id"]
    assert load_reviewed_plan(output)[2] == changed


def test_review_can_repair_story_and_cast_instead_of_only_routes(sample, tmp_path):
    context, story, staff, people, plan = sample
    people["participants"][1]["actor_id"] = 2
    people["random_actor_ids"] = [1]
    plan["scenes"][0]["actor_plans"][1]["actor_id"] = 2
    repaired = copy.deepcopy(story)
    repaired["summary"] = "Coworkers wait and coordinate without moving scene props."
    rejected = {
        "passed": False,
        "issues": [
            {
                "actor_id": None,
                "scope": "global_scenario",
                "problem": "The story needs a different activity.",
                "requested_change": "Use a feasible coordination event.",
            }
        ],
    }
    wrapped = copy.deepcopy(rejected)
    wrapped["issues"][0]["requested_change"] = {
        "actor_id": None,
        "scope": "global_scenario",
        "change": rejected["issues"][0]["requested_change"],
    }

    def stages(value):
        return _scene_answers(value)

    client = FakeClient(
        [
            story,
            {"option_id": "cast_001"},
            {"participants": people["participants"]},
            *stages(plan),
            wrapped,
            repaired,
            {"option_id": "cast_001"},
            {"participants": people["participants"]},
            *stages(plan),
            {"passed": True, "issues": []},
        ]
    )
    directory = tmp_path / "story_repair"
    ScenarioPipeline(client, max_revisions=1).run(context, 3, directory, seed=1)
    stories = [p for prompt, p, _ in client.calls if prompt.startswith("Invent a coherent")]
    assert len(stories) == 2 and "total_count" not in stories[1]
    assert stories[1]["repair_feedback"]["semantic_review"] == rejected
    assert "previous_plan" not in stories[1]["repair_feedback"]
    assert stories[1]["repair_feedback"]["previous_global_scenario"] == story
    assert json.loads((directory / "global_scenario_01.json").read_text()) == repaired
    assert json.loads((directory / "approval.json").read_text())["revision"] == 1
    assert json.loads((directory / "review_00.json").read_text()) == rejected


@pytest.mark.parametrize(
    "path,bad",
    [
        (["actor_id"], 99),
        (["subgoals", 0, "action"], "Inspect"),
        (["subgoals", 0, "duration_s"], "soon"),
        (["subgoals", 0, "duration_s"], True),
        (["subgoals"], None),
        (["subgoals", 0], "not an activity"),
    ],
)
def test_actor_repair_corrects_any_invalid_field_without_rewriting_valid_fields(sample, path, bad):
    from isaac_composer.semantic_scenario.cosmos.actor_repair import ActorPlanRepair, replace_field

    context, story, _, people, plan = sample
    expected = copy.deepcopy(plan["scenes"][0]["actor_plans"][0])
    parent = expected
    for key in path:
        parent = parent[key]
    client = FakeClient([{"value": parent}])
    actor = copy.deepcopy(expected)
    replace_field(actor, path, bad)
    ActorPlanRepair(client).repair(
        actor, 0, context["valid_cells"], {}, "all_participants_arrived", ["A1"], {"story": story}
    )
    assert actor == expected
    assert len(client.calls) == 1
    assert client.calls[0][1]["field_path"] == path


@pytest.mark.parametrize(
    "field,bad,correct",
    [
        ("queue_id", "missing_queue", "inspection"),
        ("service_action", "Inspect", "LookAround"),
        ("cell_id", "C1", "B1"),
        ("duration_s", -3, 5),
    ],
)
def test_queue_repair_preserves_the_other_activity_fields(field, bad, correct):
    from isaac_composer.semantic_scenario.cosmos.actor_repair import ActorPlanRepair

    expected = {
        "actor_id": 7,
        "start_cell": "C1",
        "goal_cell": "A1",
        "subgoals": [
            {
                "cell_id": "B1",
                "action": "Queue",
                "queue_id": "inspection",
                "service_action": "LookAround",
                "duration_s": 5,
            }
        ],
    }
    actor = copy.deepcopy(expected)
    actor["subgoals"][0][field] = bad
    client = FakeClient([{"value": correct}])
    ActorPlanRepair(client).repair(
        actor,
        7,
        ["A1", "B1", "C1"],
        {"inspection": {"spots": ["B1", "C1"]}},
        "episode_started",
        ["C1"],
        {},
    )
    assert actor == expected
    assert client.calls[0][1]["field_path"] == ["subgoals", 0, field]


def test_persistently_invalid_field_is_bounded_without_rewriting_the_actor(sample):
    from isaac_composer.semantic_scenario.cosmos.actor_repair import ActorPlanRepair

    context, _, _, _, plan = sample
    actor = copy.deepcopy(plan["scenes"][0]["actor_plans"][0])
    actor["subgoals"][0]["duration_s"] = -5
    original = copy.deepcopy(actor)
    client = FakeClient([{"value": -1}] * 3)
    with pytest.raises(PlanningRejected, match="replacement failed"):
        ActorPlanRepair(client).repair(
            actor, 0, context["valid_cells"], {}, "all_participants_arrived", ["A1"], {}
        )
    assert actor == original
    assert len(client.calls) == 3


def test_actor_semantic_correction_targets_one_scene_and_preserves_every_peer(sample):
    from isaac_composer.semantic_scenario.cosmos.local_revision import LocalScenarioRevision

    context, story, _, people, plan = sample
    later = copy.deepcopy(plan["scenes"][0])
    later["scene_id"] = "later"
    later["start_condition"] = {"type": "after_scene", "scene_id": "inspection"}
    plan["scenes"].append(later)
    original = copy.deepcopy(plan)
    expected = copy.deepcopy(plan)
    expected["scenes"][1]["actor_plans"][0]["subgoals"][0]["duration_s"] = 12
    issue = {
        "actor_id": 0,
        "scope": "scenes",
        "problem": "The later visit is too short.",
        "requested_change": "Wait twelve seconds during the later visit.",
    }
    client = FakeClient(
        [{"changes": [{"scene_id": "later", "path": ["subgoals", 0, "duration_s"], "value": 12}]}]
    )
    result = LocalScenarioRevision(client).revise(
        context, story, people, plan, {"passed": False, "issues": [issue]}
    )
    assert result["scene_plan"] == expected
    assert result["participant_plan"] == people
    assert plan == original
    assert all(s["actor_plan"]["actor_id"] == 0 for s in client.calls[0][1]["actor_scene_plans"])


def test_local_semantic_revision_rejects_other_actor_edits_and_is_atomic(sample):
    from isaac_composer.semantic_scenario.cosmos.local_revision import LocalScenarioRevision

    context, story, _, people, plan = sample
    original = copy.deepcopy(plan)
    client = FakeClient(
        [{"changes": [{"scene_id": "inspection", "path": ["actor_id"], "value": 1}]}] * 3
    )
    with pytest.raises(PlanningRejected, match="semantic correction failed"):
        LocalScenarioRevision(client).revise(
            context,
            story,
            people,
            plan,
            {"issues": [{"actor_id": 0, "problem": "Visit is too short."}]},
        )
    assert plan == original
    assert len(client.calls) == 3


def test_role_issue_only_revises_the_named_role_and_saves_a_reviewable_plan(sample, tmp_path):
    context, story, _, people, plan = sample
    people["participants"][1]["actor_id"] = 2
    people["random_actor_ids"] = [1]
    plan["scenes"][0]["actor_plans"][1]["actor_id"] = 2
    review = {
        "passed": False,
        "issues": [
            {
                "actor_id": 0,
                "scope": "roles",
                "problem": "Role wording does not describe the activity.",
                "requested_change": "Use the role storage inspector.",
            }
        ],
    }
    client = FakeClient(
        [
            story,
            {"option_id": "cast_001"},
            {"participants": people["participants"]},
            *_scene_answers(plan),
            review,
            {"changes": [{"scene_id": None, "path": ["role"], "value": "storage inspector"}]},
            {"passed": True, "issues": []},
        ]
    )
    ScenarioPipeline(client).run(context, 3, tmp_path, seed=1)
    _, accepted_people, accepted_plan = load_reviewed_plan(tmp_path)
    expected = json.loads((tmp_path / "participant_plan_00.json").read_text())
    expected["participants"][0]["role"] = "storage inspector"
    assert accepted_people == expected
    assert accepted_plan == plan
    assert len([c for c in client.calls if c[0].startswith("Plan the scene timeline")]) == 1
    assert len([c for c in client.calls if c[0].startswith("Plan ONLY")]) == 2


def test_action_case_is_normalized_without_requesting_a_new_behavior(sample):
    from isaac_composer.semantic_scenario.cosmos.actor_repair import ActorPlanRepair

    context, _, _, _, plan = sample
    expected = copy.deepcopy(plan["scenes"][0]["actor_plans"][0])
    actor = copy.deepcopy(expected)
    actor["subgoals"][0]["action"] = "lookaround"
    client = FakeClient([])
    ActorPlanRepair(client).repair(
        actor, 0, context["valid_cells"], {}, "all_participants_arrived", ["A1"], {}
    )
    assert actor == expected
    assert not client.calls


def test_excess_subgoals_are_selected_by_cosmos_with_kept_details_unchanged(sample):
    from isaac_composer.semantic_scenario.cosmos.actor_repair import ActorPlanRepair

    context, _, _, _, plan = sample
    actor = copy.deepcopy(plan["scenes"][0]["actor_plans"][0])
    actor["subgoals"] = [
        {"cell_id": "B1", "action": "LookAround", "duration_s": duration}
        for duration in range(1, 6)
    ]
    expected = copy.deepcopy(actor)
    expected["subgoals"] = expected["subgoals"][1:]
    client = FakeClient([{"value": [1, 2, 3, 4]}])
    ActorPlanRepair(client).repair(
        actor, 0, context["valid_cells"], {}, "all_participants_arrived", ["A1"], {}
    )
    assert actor == expected
    assert client.calls[0][1]["output_examples"] == [{"value": [0, 1, 2, 3]}]
    assert actor["subgoals"][0]["duration_s"] == 2


def test_field_format_repair_only_sends_the_malformed_answer_and_a_concrete_example(sample):
    from isaac_composer.semantic_scenario.cosmos.actor_repair import ActorPlanRepair

    context, _, _, _, plan = sample
    actor = copy.deepcopy(plan["scenes"][0]["actor_plans"][0])
    actor["subgoals"][0]["duration_s"] = -1
    malformed = '{"value":8}}'

    class FormatClient(FakeClient):
        def complete(self, instruction, payload, images=(), schema=None):
            self.calls.append((instruction, copy.deepcopy(payload), list(images)))
            if len(self.calls) == 1:
                raise CosmosError("Extra data", response_text=malformed)
            return {"value": 8}

    client = FormatClient([])
    ActorPlanRepair(client).repair(
        actor, 0, context["valid_cells"], {}, "all_participants_arrived", ["A1"], {}
    )
    second = client.calls[1][1]
    assert second["rejected_response"] == malformed
    assert second["output_examples"] == [{"value": 1}]
    assert "current_actor_plan" not in second
    assert actor["subgoals"][0]["duration_s"] == 8


def test_review_examples_show_both_decisions_and_do_not_repeat_a_rejected_plan(sample):
    from isaac_composer.semantic_scenario.cosmos.reviewer import ScenarioReviewer

    context, story, _, people, plan = sample
    client = FakeClient([{"passed": True, "issues": []}])
    ScenarioReviewer(client).review(
        context,
        story,
        people,
        plan,
        context["scene_image"],
        {"validation_error": "Extra data", "rejected_response": "echoed full plan"},
    )
    payload = client.calls[0][1]
    examples = payload["output_examples"]
    assert {example["passed"] for example in examples} == {True, False}
    for example in examples:
        validate_review(example, people)
        english_json(example)
    assert payload["format_repair_feedback"] == {"validation_error": "Extra data"}
    assert list(payload)[-1] == "output_task"


def test_local_failure_does_not_restart_the_stage_or_rebuild_valid_peers(tmp_path):
    from isaac_composer.semantic_scenario.schema import LocalRepairFailed

    calls = []

    def make(repair):
        calls.append(repair)
        raise LocalRepairFailed("Actor 7 correction failed")

    with pytest.raises(PlanningRejected, match="Actor 7 correction failed"):
        ScenarioPipeline(FakeClient([]))._stage("scene_plan", make, lambda result: result, tmp_path)
    assert calls == [None]


def test_invalid_behavior_is_not_repeated_in_the_allowed_value_selection_prompt(sample):
    from isaac_composer.semantic_scenario.cosmos.actor_repair import ActorPlanRepair

    context, _, _, _, plan = sample
    actor = copy.deepcopy(plan["scenes"][0]["actor_plans"][0])
    actor["subgoals"][0]["action"] = "Inspect"
    client = FakeClient([{"value": "Queue"}, {"value": "LookAround"}])
    ActorPlanRepair(client).repair(
        actor, 0, context["valid_cells"], {}, "all_participants_arrived", ["A1"], {}
    )
    for _, payload, _ in client.calls:
        assert payload["available_values"] == ["Idle", "LookAround"]
        assert "Inspect" not in json.dumps(payload)
        assert "Queue" not in json.dumps(payload)
    assert actor["subgoals"][0]["action"] == "LookAround"


def test_robot_navigation_pause_budget_is_repaired_without_changing_destinations(sample):
    from isaac_composer.semantic_scenario.cosmos.actor_repair import ActorPlanRepair

    context, _, _, _, plan = sample
    actor = copy.deepcopy(plan["scenes"][0]["actor_plans"][0])
    actor["subgoals"][0]["duration_s"] = 45
    expected = copy.deepcopy(actor)
    expected["subgoals"][0]["duration_s"] = 3
    client = FakeClient([{"value": 3}])
    ActorPlanRepair(client).repair(
        actor,
        0,
        context["valid_cells"],
        {},
        "all_participants_arrived",
        ["A1"],
        {
            "spatial_guidance": {
                "ordinary_pause_max_s": 4,
                "ordinary_pause_budget_s_per_actor_scene": 6,
            }
        },
    )
    assert actor == expected
    assert max(client.calls[0][1]["available_values"]) == 4


def test_pass_through_waypoints_do_not_wait_for_a_stationary_animation(sample):
    context, _, _, people, plan = sample
    scene = plan["scenes"][0]
    scene["start_condition"] = {"type": "episode_started"}
    scene["actor_plans"] = scene["actor_plans"][:1]
    people["participants"] = people["participants"][:1]
    actor = scene["actor_plans"][0]
    actor["subgoals"] = [{"cell_id": "B1", "action": "GoTo", "duration_s": 0}]
    actor["goal_cell"] = "C1"
    compiled = compile_scene_plan(Floor(), context, people, plan)
    walkers = [Person(), Person(), Person()]
    executor = SceneExecutor(Floor(), walkers, compiled)
    executor.step(0.1, [[1, 1, 0], [5, 1, 0], [1, 5, 0]])
    executor.step(0.1, [[3, 1, 0], [5, 1, 0], [1, 5, 0]])
    assert executor.scenes["inspection"]["actors"][0]["index"] == 1
    assert not any(e["event"] == "activity_started" for e in executor.events)
    assert walkers[0].path[-1] == [5, 1, 0]


def test_native_failed_destination_repairs_only_that_field(sample):
    context, _, _, people, plan = sample
    original = copy.deepcopy(plan)
    checks = []

    def native(context, people, candidate):
        checks.append(copy.deepcopy(candidate))
        if len(checks) == 1:
            return {
                "actor_id": 0,
                "scene_id": "inspection",
                "field_path": ["goal_cell"],
                "available_cells": ["A1", "C1"],
            }
        return None

    client = FakeClient([{"cell_id": "C1"}])
    ScenarioPipeline(client, native_validator=native)._native_check(context, people, plan)
    expected = copy.deepcopy(original)
    expected["scenes"][0]["actor_plans"][0]["goal_cell"] = "C1"
    assert plan == expected
    assert len(checks) == 2
    assert client.calls[0][1]["available_cells"] == ["A1", "C1"]
    assert context["native_paths_verified"] is True


def test_native_repair_candidates_exclude_unreachable_targets(sample):
    from isaac_composer.semantic_scenario.runtime.path_validation import inspect_native_paths

    context, _, _, people, plan = sample

    class LimitedFloor(Floor):
        def shortest_path(self, start, end):
            if end[0] == 3:
                raise RuntimeError("Path leaves the collection region")
            return [start, end]

    problem = inspect_native_paths(LimitedFloor(), context, people, plan)
    assert problem["actor_id"] == 0
    assert problem["field_path"] == ["subgoals", 0, "cell_id"]
    assert problem["available_cells"] == ["C1"]


def test_approval_also_protects_the_robot_task_given_to_cosmos(sample, tmp_path):
    context, story, _, people, plan = sample
    context["robot_task"] = {"spawn_world": [0, 0, 0.65], "goal_world": [5, 1, 0]}
    people["participants"][1]["actor_id"] = 2
    people["random_actor_ids"] = [1]
    plan["scenes"][0]["actor_plans"][1]["actor_id"] = 2
    client = FakeClient(
        [
            story,
            {"option_id": "cast_001"},
            {"participants": people["participants"]},
            *_scene_answers(plan),
            {"passed": True, "issues": []},
        ]
    )
    ScenarioPipeline(client).run(context, 3, tmp_path, seed=1)
    load_reviewed_plan(tmp_path)
    saved = json.loads((tmp_path / "scene_context.json").read_text())
    saved["robot_task"]["goal_world"] = [1, 0, 0]
    (tmp_path / "scene_context.json").write_text(json.dumps(saved))
    with pytest.raises(PlanError, match="changed after"):
        load_reviewed_plan(tmp_path)


def test_individual_role_generation_keeps_previous_roles_when_one_response_is_bad():
    from isaac_composer.semantic_scenario.cosmos.roles import RolePlanner

    client = FakeClient(
        [
            {"role": "crossing coworker", "purpose": "Walk across the aisle."},
            {"participants": []},
            {"role": "departing coworker", "purpose": "Leave after the brief meeting."},
        ]
    )
    roles = RolePlanner(client).generate(
        {"summary": "Colleagues cross paths."},
        [0, 4],
        {"spatial_guidance": {"activity_goal": "Create purposeful foot traffic."}},
    )
    assert [a["actor_id"] for a in roles["participants"]] == [0, 4]
    assert [p["requested_actor_id"] for _, p, _ in client.calls] == [0, 4, 4]
    assert client.calls[2][1]["other_assigned_roles"] == roles["participants"][:1]


def test_unambiguous_single_document_array_wrapper_is_normalized_without_guessing():
    assert parse_json_answer('[{"passed":true,"issues":[]}]') == {"passed": True, "issues": []}
    with pytest.raises(CosmosError, match="JSON object"):
        parse_json_answer('[{"passed":true},{"passed":false}]')


def test_creative_story_stage_does_not_reuse_previous_cell_or_actor_assignments(sample):
    from isaac_composer.semantic_scenario.cosmos.planner import ScenarioPlanner
    from isaac_composer.semantic_scenario.spatial import enrich_context

    context, story, _, _, _ = sample
    context = enrich_context(context, 17)
    context["recent_scenarios"] = [
        {
            "scenario_id": "previous",
            "summary": "An inspection round.",
            "events": [{"description": "Wait.", "actor_ids": [99], "locations": ["north_west"]}],
        }
    ]
    response = copy.deepcopy(story)
    response["events"][0]["actor_ids"] = [99]
    response["events"][0]["locations"] = ["north_west"]
    client = FakeClient([response])
    result = ScenarioPlanner(client).draft_global_scenario(context)
    assert result == story
    instruction, payload, images = client.calls[0]
    assert images == []
    assert payload["recent_scenarios"] == [
        {"scenario_id": "previous", "summary": "An inspection round."}
    ]
    assert "99" not in json.dumps(payload)
    assert "spatial_guidance" not in payload
    assert payload["recent_scenarios"][0]["summary"] == "An inspection round."
