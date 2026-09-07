#!/usr/bin/env bash
set -euo pipefail

# Resolve the repository independently of the caller's current directory.
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "$SCRIPT_DIR/.." && pwd)"

# Keep machine-specific installation paths outside the tracked repository.
ISAACSIM_ROOT="${UNIVTAC_ISAACSIM_ROOT:?Set UNIVTAC_ISAACSIM_ROOT to the Isaac Sim 6.0.1 directory}"
CUDA_ROOT="${UNIVTAC_CUDA_ROOT:-$ISAACSIM_ROOT/kit/python/lib/python3.12/site-packages/nvidia/cu13}"
ISAACSIM_PYTHON="$ISAACSIM_ROOT/python.sh"

if [[ ! -x "$ISAACSIM_PYTHON" ]]; then
    echo "Isaac Sim 6 python.sh is not executable: $ISAACSIM_PYTHON" >&2
    echo "Set UNIVTAC_ISAACSIM_ROOT to the Isaac Sim 6.0.1 RC directory." >&2
    exit 2
fi

if [[ ! -d "$CUDA_ROOT/bin" ]]; then
    echo "Isaac Sim CUDA toolkit directory is missing: $CUDA_ROOT" >&2
    echo "Set UNIVTAC_CUDA_ROOT to the Python 3.12 nvidia/cu13 package directory." >&2
    exit 2
fi

# Export the resolved paths so multiprocessing workers inherit the same runtime.
export UNIVTAC_ISAACSIM_ROOT="$ISAACSIM_ROOT"
export UNIVTAC_CUDA_ROOT="$CUDA_ROOT"
export CUDA_HOME="$CUDA_ROOT"
export PATH="$CUDA_ROOT/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"

# Prevent ROS 2 or an old Conda environment from injecting incompatible Python packages.
unset ROS_DISTRO AMENT_PREFIX_PATH PYTHONPATH COLCON_PREFIX_PATH

# UniVTAC resolves tasks, assets, configs, and policy modules relative to its root.
cd "$REPO_ROOT"
exec "$ISAACSIM_PYTHON" "$@"
