"""Resolve package-owned and Isaac Sim server assets at runtime."""

from __future__ import annotations

import os
from pathlib import Path


REMOTE_PREFIXES = ("omniverse://", "http://", "https://")


def get_isaac_assets_root() -> str:
    """Return the asset root configured by the active Isaac Sim installation."""

    try:
        from isaacsim.storage.native import get_assets_root_path
    except ImportError:
        from omni.isaac.core.utils.nucleus import get_assets_root_path

    root = get_assets_root_path()
    if not root:
        raise RuntimeError("Isaac Sim assets root is unavailable")
    return str(root).rstrip("/")


def resolve_asset(path_value: str, package_root: str | Path) -> str:
    """Resolve an Isaac server path or package-share-relative local path."""

    value = os.path.expandvars(os.path.expanduser(str(path_value)))
    if value.startswith(REMOTE_PREFIXES):
        return value
    if value.startswith("isaac_assets://"):
        relative = value.removeprefix("isaac_assets://").lstrip("/")
        return f"{get_isaac_assets_root()}/{relative}"
    if value.startswith("/Isaac/"):
        return get_isaac_assets_root() + value

    path = Path(value)
    if not path.is_absolute():
        path = Path(package_root) / path
    resolved = path.resolve()
    if not resolved.is_file():
        raise FileNotFoundError(f"Local asset does not exist: {resolved}")
    return str(resolved)


def as_reference_url(resolved_path: str) -> str:
    """Convert a local path to a USD file URL while preserving remote URLs."""

    if resolved_path.startswith(REMOTE_PREFIXES):
        return resolved_path
    return Path(resolved_path).resolve().as_uri()
