"""Controller lifecycle kept separate from robot-name dispatch."""

from __future__ import annotations

from typing import Any, Protocol

from isaac_composer.models import RobotContext


class ControllerProvider(Protocol):
    def prepare(self, robot: RobotContext, config: dict[str, Any]) -> None:
        """Prepare resources before the first world reset."""
        ...

    def start_after_reset(self, robot: RobotContext, world: Any) -> None:
        """Register callbacks or initialize policy state after world reset."""
        ...

    def step(self, step_size: float) -> None:
        """Advance controllers that are not driven by a physics callback."""
        ...

    def shutdown(self) -> None:
        """Release callbacks and runtime resources."""
        ...
