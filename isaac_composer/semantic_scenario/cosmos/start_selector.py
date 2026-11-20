"""Let Cosmos reserve one initial start from the shrinking candidate list."""

from ..schema import PlanError, LocalRepairFailed
from .client import CosmosError
from .contracts import obj
from .output_examples import with_examples

PROMPT = """Choose an initial start cell for this pedestrian.
Choose ONLY one exact ID from available_start_cells. Return ONLY a JSON object
with the key start_cell and the selected ID as its string value.
Previously reserved cells have been removed. Never invent or reuse an unlisted ID."""


class StartSelector:
    def __init__(self, client):
        self.client = client

    def choose(self, actor_id, available, guidance=None):
        if not available:
            raise PlanError("Not enough distinct valid cells for the initial spawns")
        schema = obj({"start_cell": {"type": "string", "enum": list(available)}})
        canonical = {cell.casefold(): cell for cell in available}
        for attempt in range(3):
            # No full cell list, image labels, old routes or rejected cell IDs:
            # the only location IDs in this request are still available starts.
            payload = {"actor_id": actor_id, "available_start_cells": list(available)}
            if guidance:
                payload["placement_context"] = guidance
            if attempt:
                payload["repair_instruction"] = (
                    "The previous selection was invalid. Copy one ID exactly from "
                    "available_start_cells. Return only start_cell."
                )
            try:
                response = self.client.complete(
                    PROMPT, with_examples(payload, {"start_cell": available[0]}), schema=schema
                )
                cell = response.get("start_cell")
                if isinstance(cell, str):
                    cell = canonical.get(cell.strip().casefold())
                if cell not in available:
                    raise PlanError("start_cell must be selected from available_start_cells")
                print(f"SEMANTIC_START_SELECTED actor={actor_id} cell={cell}", flush=True)
                return cell
            except (PlanError, CosmosError, AttributeError, TypeError) as error:
                print(f"SEMANTIC_START_REPAIR actor={actor_id} reason={error}", flush=True)
                if attempt == 2:
                    raise LocalRepairFailed(
                        f"Actor {actor_id} start selection failed: {error}"
                    ) from error
