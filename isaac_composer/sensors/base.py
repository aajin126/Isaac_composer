"""Attach/detach interface for generated, existing, and embedded sensors."""

from __future__ import annotations

from typing import Any, Protocol

from isaac_composer.models import RobotContext, SensorHandle


class SensorProvider(Protocol):
    def setup(
        self,
        robot: RobotContext,
        config: dict[str, Any],
    ) -> list[SensorHandle]:
        """Create or discover all configured sensors."""
        ...

    def detach(self, sensor: SensorHandle) -> None:
        """Remove publisher resources before removing sensor USD prims."""
        ...

    def shutdown(self) -> None:
        """Release every sensor managed by this provider."""
        ...
