"""Timed pedestrian actions, relative to the end of the start delay."""

from __future__ import annotations

import math
from typing import Any


def validate_motion_events(events: list[dict[str, Any]]) -> list[dict[str, Any]]:
    if not isinstance(events, list):
        raise ValueError("Pedestrian events must be a list")
    normalized = []
    for event in events:
        if not isinstance(event, dict) or event.get("action") not in {"stop", "speed"}:
            raise ValueError("Pedestrian events require action='stop' or action='speed'")
        item = dict(event)
        for key in ("time_s", "duration_s"):
            value = float(item[key])
            if not math.isfinite(value) or value < 0 or (key == "duration_s" and value == 0):
                raise ValueError(f"Invalid pedestrian event {key}: {value}")
            item[key] = value
        if item["action"] == "speed":
            speed = float(item["speed_mps"])
            if not math.isfinite(speed) or speed < 0:
                raise ValueError(f"Invalid pedestrian event speed_mps: {speed}")
            item["speed_mps"] = speed
        normalized.append(item)
    return sorted(normalized, key=lambda item: item["time_s"])


def scheduled_speed(events: list[dict[str, Any]], elapsed: float, default: float) -> float:
    speed = default
    for event in events:
        if event["time_s"] <= elapsed < event["time_s"] + event["duration_s"]:
            speed = 0.0 if event["action"] == "stop" else event["speed_mps"]
    return speed
