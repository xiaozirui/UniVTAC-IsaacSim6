#!/usr/bin/env bash
# Modified from the UniVTAC project.
# Changes in this derivative project include compatibility adaptations
# for Isaac Sim 6.0.1 and Isaac Lab 3.0.0.

set -euo pipefail

REPO_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

TASK_NAME=${1:?Usage: bash parallel_collect.sh TASK [CONFIG] [GPU_LIST] [WORKERS]}
CONFIG_NAME=${2:-"demo"}
GPU=${3:-0}
NUM_PROCESSES=${4:-3}

exec "$REPO_ROOT/scripts/run_isaacsim6.sh" scripts/parallel_collect_data.py \
    "$TASK_NAME" "$CONFIG_NAME" \
    --gpu "$GPU" \
    --workers "$NUM_PROCESSES"
