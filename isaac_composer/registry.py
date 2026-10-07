"""Small explicit registries used instead of robot-name conditionals."""

from __future__ import annotations

from collections.abc import Callable
from typing import Any, Generic, TypeVar


T = TypeVar("T")


class RegistryError(LookupError):
    """Raised for missing or duplicate provider registrations."""


class ProviderRegistry(Generic[T]):
    def __init__(self, kind: str):
        self.kind = kind
        self._providers: dict[str, Callable[..., T]] = {}

    def register(self, name: str):
        key = self._normalize(name)

        def decorator(factory: Callable[..., T]) -> Callable[..., T]:
            if key in self._providers:
                raise RegistryError(
                    f"{self.kind} provider {key!r} is already registered"
                )
            self._providers[key] = factory
            return factory

        return decorator

    def get(self, name: str) -> Callable[..., T]:
        key = self._normalize(name)
        try:
            return self._providers[key]
        except KeyError as error:
            available = ", ".join(self.names()) or "<none>"
            raise RegistryError(
                f"Unknown {self.kind} provider {key!r}; available: {available}"
            ) from error

    def create(self, name: str, *args: Any, **kwargs: Any) -> T:
        return self.get(name)(*args, **kwargs)

    def names(self) -> tuple[str, ...]:
        return tuple(sorted(self._providers))

    @staticmethod
    def _normalize(name: str) -> str:
        key = str(name).strip().lower().replace("-", "_")
        if not key:
            raise RegistryError("Provider name must not be empty")
        return key


spawn_providers: ProviderRegistry[Any] = ProviderRegistry("spawn")
controller_providers: ProviderRegistry[Any] = ProviderRegistry("controller")
sensor_providers: ProviderRegistry[Any] = ProviderRegistry("sensor")
ros_providers: ProviderRegistry[Any] = ProviderRegistry("ROS")
clock_providers: ProviderRegistry[Any] = ProviderRegistry("clock")
