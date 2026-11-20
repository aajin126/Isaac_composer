#!/usr/bin/env bash
set -eo pipefail

# Use ROS's system setuptools, avoiding incompatible pip user installations.
workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../.." && pwd)"
cd -- "$workspace_dir"
export PYTHONNOUSERSITE=1
source /opt/ros/humble/setup.bash
set -u
exec /usr/bin/colcon build --symlink-install "$@"
