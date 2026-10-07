"""Interface implemented by USD-reference and policy-based spawners."""

from __future__ import annotations

from typing import Any, Protocol

from isaac_composer.models import RobotContext


class SpawnProvider(Protocol):
    def spawn(
        self,
        config: dict[str, Any],
        *,
        stage: Any,
        world: Any | None,
    ) -> RobotContext:
        """Create the robot and return paths/resources for other providers."""
        ...
