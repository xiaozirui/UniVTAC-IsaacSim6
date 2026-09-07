#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0

# Run the standard UniVTAC data-collection tasks one at a time.  The next task
# is started only after the current Isaac Sim process has exited.

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

usage() {
    cat <<'EOF'
Usage:
  bash scripts/collect_all_tasks.sh \
    --config task_config/demo.yml \
    --episode_num 1 \
    --start_seed 0 \
    --max_seed 20 \
    --gpu 0 \
    --viz none

The supplied settings are applied to all eight tasks. Tasks are always run
strictly sequentially; this script never launches multiple Isaac Sim instances
at the same time.
EOF
}

die() {
    echo "Error: $*" >&2
    echo >&2
    usage >&2
    exit 2
}

require_value() {
    if (( $# < 2 )) || [[ "$2" == --* ]]; then
        die "$1 requires a value."
    fi
}

CONFIG=""
EPISODE_NUM=""
START_SEED=""
MAX_SEED=""
GPU=""
VIZ=""

while (( $# > 0 )); do
    case "$1" in
        --config|--yaml)
            require_value "$@"
            CONFIG="$2"
            shift 2
            ;;
        --episode_num)
            require_value "$@"
            EPISODE_NUM="$2"
            shift 2
            ;;
        --start_seed)
            require_value "$@"
            START_SEED="$2"
            shift 2
            ;;
        --max_seed)
            require_value "$@"
            MAX_SEED="$2"
            shift 2
            ;;
        --gpu)
            require_value "$@"
            GPU="$2"
            shift 2
            ;;
        --viz)
            require_value "$@"
            VIZ="$2"
            shift 2
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            die "unknown argument: $1"
            ;;
    esac
done

[[ -n "$CONFIG" ]] || die "--config is required."
[[ -n "$EPISODE_NUM" ]] || die "--episode_num is required."
[[ -n "$START_SEED" ]] || die "--start_seed is required."
[[ -n "$MAX_SEED" ]] || die "--max_seed is required."
[[ -n "$GPU" ]] || die "--gpu is required."
[[ -n "$VIZ" ]] || die "--viz is required."

if [[ ! "$EPISODE_NUM" =~ ^[0-9]+$ ]] || (( EPISODE_NUM <= 0 )); then
    die "--episode_num must be a positive integer."
fi
if [[ ! "$START_SEED" =~ ^[0-9]+$ ]]; then
    die "--start_seed must be a non-negative integer."
fi
if [[ ! "$MAX_SEED" =~ ^[0-9]+$ ]]; then
    die "--max_seed must be a non-negative integer."
fi
if (( START_SEED > MAX_SEED )); then
    die "--start_seed must not be greater than --max_seed."
fi

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
