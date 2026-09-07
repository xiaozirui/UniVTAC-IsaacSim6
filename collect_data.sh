#!/usr/bin/env bash
# Modified from the UniVTAC project.
# Changes in this derivative project include compatibility adaptations
# for Isaac Sim 6.0.1 and Isaac Lab 3.0.0.

set -euo pipefail

REPO_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ISAACSIM_ROOT="${UNIVTAC_ISAACSIM_ROOT:?Set UNIVTAC_ISAACSIM_ROOT to the Isaac Sim 6.0.1 directory}"

TASK_NAME=${1:?Usage: bash collect_data.sh TASK CONFIG [GPU] [START_SEED] [MAX_SEED] [EPISODE_NUM]}
CONFIG_NAME=${2:?Usage: bash collect_data.sh TASK CONFIG [GPU] [START_SEED] [MAX_SEED] [EPISODE_NUM]}
GPU=${3:-0}
START_SEED=${4:--1}
MAX_SEED=${5:--1}
EPISODE=${6:--1}
VIZ=${UNIVTAC_VIZ:-none}

exec "$REPO_ROOT/scripts/run_isaacsim6.sh" scripts/collect_data.py \
    "$TASK_NAME" "$CONFIG_NAME" \
    --start_seed "$START_SEED" \
    --max_seed "$MAX_SEED" \
    --episode_num "$EPISODE" \
    --gpu "$GPU" \
    --viz "$VIZ" \
    --kit_args="--ext-folder=$ISAACSIM_ROOT/extsDeprecated"
