"""Concrete JSON format examples, never fallback planning decisions."""

import copy

from ..schema import PlanError, english_json


def with_examples(payload, *examples):
    """Keep the response shape at the end, after potentially long input plans."""
    english_json(list(examples))
    instruction = payload.get("output_task", "Return only the requested JSON object.")
    return {
        **{k: v for k, v in payload.items() if k != "output_task"},
        "output_examples": list(examples),
        "output_task": instruction
        + (
            " The output_examples show complete JSON response formats only. "
            "Make your own choices for this request; do not echo INPUT or output_examples, "
            "and do not add markdown, reasoning, comments or extra keys."
        ),
    }


def schema_example(schema):
    """Illustrate one valid value of a field, without using it as a result."""
    if "const" in schema:
        return copy.deepcopy(schema["const"])
    if "enum" in schema:
        if not schema["enum"]:
            raise PlanError("The field has no allowed replacement values")
        return copy.deepcopy(schema["enum"][0])
    if "anyOf" in schema:
        return schema_example(schema["anyOf"][0])
    kind = schema.get("type")
    if kind == "object":
        return {key: schema_example(schema["properties"][key]) for key in schema["required"]}
    if kind == "array":
        return [schema_example(schema["items"]) for _ in range(max(1, schema.get("minItems", 0)))]
    if kind in ("number", "integer"):
        return max(schema.get("minimum", 0), min(5, schema.get("maximum", 5)))
    if kind == "boolean":
        return True
    return "English text"
