#!/usr/bin/env python3
"""Isaac Python entry point; grid computation lives in the package."""

from run_scenario import _add_package_to_path

_add_package_to_path()

from isaac_composer.scene_grid_export import main

if __name__ == "__main__":
    main()
