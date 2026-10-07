"""ROS interfaces that support both generated and embedded ActionGraphs."""

from __future__ import annotations

from typing import Any, Protocol

from isaac_composer.models import RobotContext, SensorHandle


class RosProvider(Protocol):
    def setup_robot(
        self,
        robot: RobotContext,
        config: dict[str, Any],
    ) -> None:
        """Create or validate the robot-level ROS bridge."""
        ...

    def setup_sensors(
        self,
        robot: RobotContext,
        sensors: list[SensorHandle],
        config: dict[str, Any],
    ) -> None:
        """Create publishers and TF for generated/existing sensors."""
        ...

    def shutdown(self) -> None:
        ...


class ClockProvider(Protocol):
    def setup(self, config: dict[str, Any], *, stage: Any) -> None:
        """Create, validate, or disable the single simulation clock."""
        ...

    def shutdown(self) -> None:
        ...
