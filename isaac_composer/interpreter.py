"""Locate the same Isaac Python for context export and episode simulation."""

import os
from pathlib import Path


def default_isaac_python():
    candidates = [os.environ.get("ISAACSIM_PYTHON", "")]
    # Keep ROS on its system interpreter; use the existing torch-enabled
    # interpreter only inside the simulator child process.
    conda_python = Path.home() / "anaconda3/envs/env_isaacsim/bin/python"
    if conda_python.is_file():
        wrapper = Path(__file__).resolve().parents[1] / "scripts/isaac_python.sh"
        if not wrapper.is_file():
            try:
                from ament_index_python.packages import get_package_share_directory

                wrapper = Path(get_package_share_directory("isaac_composer")) / "scripts/isaac_python.sh"
            except (ImportError, LookupError):
                pass
        if wrapper.is_file():
            candidates.append(str(wrapper))
    package_path = os.environ.get("isaac_sim_package_path", "")
    if package_path:
        candidates.append(str(Path(package_path) / "python.sh"))
    candidates.extend(
        [str(Path.home() / "develop/isaacsim/python.sh"), str(Path.home() / "isaacsim/python.sh")]
    )
    return next((path for path in candidates if path and Path(path).is_file()), "python3")
