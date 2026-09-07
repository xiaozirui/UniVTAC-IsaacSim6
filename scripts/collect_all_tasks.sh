#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0

# Run the standard UniVTAC data-collection tasks one at a time.  The next task
# is started only after the current Isaac Sim process has exited.
# Usage: bash scripts/collect_all_tasks.sh

set -uo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "$SCRIPT_DIR/.." && pwd)"

TASKS=(
    grasp_classify
    lift_bottle
    lift_can
    insert_HDMI
    insert_hole
    insert_tube
    pull_out_key
    put_bottle_in_shelf
)

# Shared collection settings.  Environment variables make intentional batch
# overrides possible without duplicating or editing eight commands.
CONFIG="${UNIVTAC_BATCH_CONFIG:-task_config/demo.yml}"
EPISODE_NUM="${UNIVTAC_BATCH_EPISODE_NUM:-1}"
START_SEED="${UNIVTAC_BATCH_START_SEED:-0}"
MAX_SEED="${UNIVTAC_BATCH_MAX_SEED:-20}"
GPU="${UNIVTAC_BATCH_GPU:-0}"
VIZ="${UNIVTAC_BATCH_VIZ:-none}"

if [[ "$CONFIG" != /* ]]; then
    CONFIG="$REPO_ROOT/$CONFIG"
fi

if [[ ! -f "$CONFIG" ]]; then
    echo "Task configuration not found: $CONFIG" >&2
    exit 2
fi

if [[ -z "${UNIVTAC_ISAACSIM_ROOT:-}" ]]; then
    echo "Set UNIVTAC_ISAACSIM_ROOT to the Isaac Sim 6.0.1 directory." >&2
    exit 2
fi

RUN_ID="$(date +'%Y%m%d_%H%M%S')"
RUN_DIR="$REPO_ROOT/data/_batch_runs/$RUN_ID"
SUMMARY="$RUN_DIR/summary.tsv"
mkdir -p "$RUN_DIR"
printf 'task\texit_code\tlog\n' > "$SUMMARY"

echo "UniVTAC serial data collection"
echo "Tasks: ${#TASKS[@]} (strictly sequential)"
echo "Config: $CONFIG"
echo "episode_num=$EPISODE_NUM start_seed=$START_SEED max_seed=$MAX_SEED gpu=$GPU viz=$VIZ"
echo "Batch logs: $RUN_DIR"

failed_tasks=()

trap 'echo; echo "Collection queue interrupted." >&2; exit 130' INT TERM

cd "$REPO_ROOT" || exit 2
for index in "${!TASKS[@]}"; do
    task="${TASKS[$index]}"
    log_file="$RUN_DIR/$task.log"

    echo
    echo "[$((index + 1))/${#TASKS[@]}] Starting $task"

    set +o pipefail
    bash "$SCRIPT_DIR/run_isaacsim6.sh" \
        "$SCRIPT_DIR/collect_data.py" \
        "$task" "$CONFIG" \
        --episode_num "$EPISODE_NUM" \
        --start_seed "$START_SEED" \
        --max_seed "$MAX_SEED" \
        --gpu "$GPU" \
        --viz "$VIZ" \
        2>&1 | tee "$log_file"
    task_status="${PIPESTATUS[0]}"
    set -o pipefail

    printf '%s\t%s\t%s\n' "$task" "$task_status" "$log_file" >> "$SUMMARY"
    if (( task_status == 0 )); then
        echo "[$((index + 1))/${#TASKS[@]}] Finished $task"
    else
        echo "[$((index + 1))/${#TASKS[@]}] $task failed with exit code $task_status; continuing." >&2
        failed_tasks+=("$task")
    fi
done

echo
echo "Collection queue finished. Summary: $SUMMARY"
if (( ${#failed_tasks[@]} > 0 )); then
    echo "Failed tasks: ${failed_tasks[*]}" >&2
    exit 1
fi

echo "All tasks completed successfully."
