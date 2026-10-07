"""Shared data passed between spawn, controller, sensor, and ROS providers."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any


@dataclass(frozen=True)
class LinkContext:
    """A robot-relative mounting link and its corresponding ROS frame."""

    name: str
    prim_path: str
    frame_id: str


@dataclass
class RobotContext:
    """Robot information that downstream providers can use without robot names."""

    name: str
    root_prim_path: str
    articulation_prim_path: str | None = None
    links: dict[str, LinkContext] = field(default_factory=dict)
    runtime_handle: Any | None = None
    metadata: dict[str, Any] = field(default_factory=dict)

    def require_link(self, alias: str) -> LinkContext:
        try:
            return self.links[alias]
        except KeyError as error:
            available = ", ".join(sorted(self.links)) or "<none>"
            raise KeyError(
                f"Robot {self.name!r} has no link alias {alias!r}; "
                f"available aliases: {available}"
            ) from error


@dataclass
class SensorHandle:
    """Resources created or discovered for one sensor instance."""

    name: str
    sensor_type: str
    mount_prim_path: str
    sensor_prim_path: str
    frame_id: str
    topic: str | None = None
    render_product: Any | None = None
    runtime_handle: Any | None = None
    metadata: dict[str, Any] = field(default_factory=dict)
