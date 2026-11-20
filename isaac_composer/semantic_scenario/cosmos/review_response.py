"""Adapt unambiguous model wrappers to the canonical review response."""

import copy

from ..schema import PlanError


def normalize_review_response(response):
    """Unwrap a correction sentence without changing a review decision or target."""
    document = copy.deepcopy(response)
    if not isinstance(document, dict) or not isinstance(document.get("issues"), list):
        return document  # The regular review validator reports structural errors.
    for index, issue in enumerate(document["issues"]):
        if not isinstance(issue, dict):
            continue
        change = issue.get("requested_change")
        if not isinstance(change, dict):
            continue
        field = f"issues[{index}].requested_change"
        if "change" not in change or set(change) - {"actor_id", "scope", "change"}:
            raise PlanError(f"{field} must be a correction STRING; unsupported object wrapper")
        for key in ("actor_id", "scope"):
            if key in change and (
                type(change[key]) is not type(issue.get(key)) or change[key] != issue.get(key)
            ):
                raise PlanError(
                    f"{field}.{key} conflicts with the enclosing issue; correct the target"
                )
        sentence = change["change"]
        if not isinstance(sentence, str) or not sentence.strip():
            raise PlanError(f"{field}.change must be a nonempty correction STRING")
        issue["requested_change"] = sentence
    return document
