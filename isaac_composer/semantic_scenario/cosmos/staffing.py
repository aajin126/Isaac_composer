"""Cosmos selects a valid population allocation instead of calculating counts."""

from ..schema import PlanError, validate_staffing
from .contracts import staffing_contract
from .output_examples import with_examples

PROMPT = """Choose one staffing option for the supplied human activity story.
Each supplied option already satisfies the population budget and minimum planned cast.
Choose the option whose planned cast can perform the main activity and meaningful
supporting pedestrian roles. Reuse people across sequential events where appropriate.
Return ONLY a JSON object with option_id, copied exactly from staffing_options.
Do not output a raw headcount, justification, or a copy of the input."""


class StaffingPlanner:
    def __init__(self, client):
        self.client = client

    def generate(self, story, total, feedback=None):
        if type(total) is not int or total < 1:
            raise PlanError("total_count must be a positive integer")
        minimum = (total + 1) // 2
        options = [
            {"option_id": f"cast_{index:03d}", "cosmos_count": count, "random_count": total - count}
            for index, count in enumerate(range(minimum, total + 1), 1)
        ]
        by_id = {option["option_id"]: option for option in options}
        payload = {"global_scenario": story, "staffing_options": options}
        if feedback:
            # Invalid prior counts are deliberately not repeated as examples.
            payload["repair_instruction"] = (
                "The previous answer did not select a supplied option. "
                "Copy one option_id exactly from staffing_options."
            )
        payload["output_task"] = (
            "Choose one of these IDs: "
            + ", ".join(by_id)
            + '. Return only {"option_id":"the chosen supplied ID"}.'
        )
        response = self.client.complete(
            PROMPT,
            with_examples(payload, {"option_id": options[0]["option_id"]}),
            schema=staffing_contract(options),
        )
        selected = response.get("option_id") if isinstance(response, dict) else None
        if not isinstance(selected, str) or selected not in by_id:
            raise PlanError(
                "Choose option_id from staffing_options; do not return a raw cosmos_count"
            )
        option = by_id[selected]
        document = {
            "total_count": total,
            "minimum_cosmos_count": minimum,
            "cosmos_count": option["cosmos_count"],
            "random_count": option["random_count"],
            "justification": (
                f"Cosmos selected {option['cosmos_count']} planned pedestrians and "
                f"{option['random_count']} background pedestrians."
            ),
        }
        validate_staffing(document, total)
        print(
            f"SEMANTIC_STAFFING_SELECTED option={selected} "
            f"cosmos={option['cosmos_count']} random={option['random_count']} total={total}",
            flush=True,
        )
        return document
