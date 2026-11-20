#!/usr/bin/env bash
# Run Isaac Sim with the installed torch-enabled Python, independently of ROS.
set -e

isaac_python_exe="${PYTHONEXE:-$HOME/anaconda3/envs/env_isaacsim/bin/python}"
isaac_sim_root=""
for isaac_root_candidate in "${isaac_sim_package_path:-}" "$HOME/develop/isaacsim" "$HOME/isaacsim"; do
    if [[ -f "$isaac_root_candidate/python.sh" ]]; then
        isaac_sim_root="$isaac_root_candidate"
        break
    fi
done
if [[ ! -x "$isaac_python_exe" || ! -f "$isaac_sim_root/python.sh" ]]; then
    echo "Isaac Sim or env_isaacsim Python was not found." >&2
    exit 1
fi
export PYTHONEXE="$isaac_python_exe"
# Isaac Sim's bundled torch needs the CUDA libraries shipped in ml_archive.
# Set their search paths for every invocation, including from a fresh ROS shell.
isaac_ml_bundle="$isaac_sim_root/exts/omni.isaac.ml_archive/pip_prebundle"
isaac_torch_library_path=""
for isaac_lib_dir in "$isaac_ml_bundle/cusparselt/lib" "$isaac_ml_bundle"/nvidia/*/lib; do
    if [[ -d "$isaac_lib_dir" ]]; then
        isaac_torch_library_path="${isaac_torch_library_path:+$isaac_torch_library_path:}$isaac_lib_dir"
    fi
done
if [[ -n "$isaac_torch_library_path" ]]; then
    export LD_LIBRARY_PATH="$isaac_torch_library_path${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
fi
# python.sh's conda warning concerns its default bundled interpreter. We have
# explicitly selected the external Python; do not change the parent ROS shell.
unset CONDA_PREFIX CONDA_DEFAULT_ENV
exec "$isaac_sim_root/python.sh" "$@"
