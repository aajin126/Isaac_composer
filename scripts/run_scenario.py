#!/usr/bin/env python3
"""Bootstrap the installed package under Isaac Sim's Python interpreter."""

from __future__ import annotations

import sys
from pathlib import Path


def _add_package_to_path() -> None:
    script_path = Path(__file__).resolve()
    source_root = script_path.parent.parent
    if (source_root / "isaac_composer").is_dir():
        sys.path.insert(0, str(source_root))
        return

    # Installed layout: <prefix>/share/isaac_composer/scripts/run_scenario.py
    prefix = script_path.parents[3]
    for site_packages in prefix.glob("lib/python*/site-packages"):
        sys.path.insert(0, str(site_packages))


_add_package_to_path()

from isaac_composer.runtime import main  # noqa: E402


if __name__ == "__main__":
    main()
