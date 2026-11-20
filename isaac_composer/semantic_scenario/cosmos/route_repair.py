"""Repair only invalid destination IDs while preserving the accepted route."""

from ..schema import PlanError, LocalRepairFailed
from .client import CosmosError
from .contracts import obj
from .output_examples import with_examples

PROMPT = """Choose a replacement route cell for this pedestrian.
One destination field is invalid or missing. Choose ONLY one exact ID from available_cells.
The preceding valid cell is supplied for continuity. Return ONLY {"cell_id":"a supplied ID"}.
Do not rewrite the route, actor, activities, durations, or other pedestrians."""


class RouteCellRepair:
    def __init__(self, client):
        self.client = client

    def choose(self, actor_id, field, cells, previous):
        canonical = {c.casefold(): c for c in cells}
        for attempt in range(3):
            payload = {"actor_id": actor_id, "field": field, "available_cells": list(cells)}
            if previous in cells:
                payload["preceding_cell"] = previous
            if attempt:
                payload["repair_instruction"] = "Copy one cell_id exactly from available_cells."
            try:
                response = self.client.complete(
                    PROMPT,
                    with_examples(payload, {"cell_id": cells[0]}),
                    schema=obj({"cell_id": {"type": "string", "enum": list(cells)}}),
                )
                cell = response.get("cell_id")
                if isinstance(cell, str):
                    cell = canonical.get(cell.strip().casefold())
                if cell not in cells:
                    raise PlanError("Replacement cell_id must belong to available_cells")
                return cell
            except (PlanError, CosmosError, AttributeError, TypeError) as error:
                print(
                    f"SEMANTIC_CELL_REPAIR actor={actor_id} field={field} reason={error}",
                    flush=True,
                )
                if attempt == 2:
                    raise LocalRepairFailed(
                        f"Actor {actor_id} field {field} replacement failed: {error}"
                    ) from error

    def repair(self, actor, cells):
        """Only replace invalid IDs; the model owns every replacement choice."""
        aid = actor["actor_id"]
        previous = actor.get("start_cell")
        replacements = {}

        def replacement(original, field):
            # Repeated references to the same bad ID describe one destination.
            # Reuse Cosmos's choice so repairing the goal does not split it.
            if isinstance(original, str) and original in replacements:
                return replacements[original]
            chosen = self.choose(aid, field, cells, previous)
            if isinstance(original, str):
                replacements[original] = chosen
            return chosen

        for index, step in enumerate(actor.get("subgoals", [])):
            if not isinstance(step, dict):
                continue  # The regular validator reports non-cell shape errors.
            original = step.get("cell_id")
            if original not in cells and step.get("action") != "Queue":
                step["cell_id"] = replacement(original, f"subgoals[{index}].cell_id")
                print(
                    f"SEMANTIC_CELL_REPLACED actor={aid} field=subgoals[{index}].cell_id "
                    f"old={original} new={step['cell_id']}",
                    flush=True,
                )
            previous = step.get("cell_id")
        original = actor.get("goal_cell")
        if original not in cells:
            actor["goal_cell"] = replacement(original, "goal_cell")
            print(
                f"SEMANTIC_CELL_REPLACED actor={aid} field=goal_cell "
                f"old={original} new={actor['goal_cell']}",
                flush=True,
            )
