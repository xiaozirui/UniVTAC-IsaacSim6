# Modified from the UniVTAC project.
# Changes in this derivative project include compatibility adaptations
# for Isaac Sim 6.0.1 and Isaac Lab 3.0.0.

import os
import sys
import time
import yaml
import json
import torch
import argparse
import traceback
from pathlib import Path
from typing import TYPE_CHECKING, Literal

from isaaclab.app import AppLauncher

sys.path.append('.')

# add argparse arguments
parser = argparse.ArgumentParser(
    description="Collect data"
)
parser.add_argument(
    "task",
    type=str,
    help="Task file name",
)
parser.add_argument(
    "config",
    type=str,
    help="Config file name",
    default="demo.yml"
)
parser.add_argument(
    "--episode_num",
    type=int,
    default=-1,
)
parser.add_argument(
    "--start_seed",
    type=int,
    default=-1,
)
parser.add_argument(
    "--max_seed",
    type=int,
    default=-1,
)
parser.add_argument(
    "--gpu",
    type=str,
    default=None,
)
parser.add_argument(
    "--keep_open",
    action="store_true",
    help="Keep the Isaac Sim window open after collection finishes (GUI mode only).",
)

AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
if args_cli.gpu is not None:
    os.environ['CUDA_VISIBLE_DEVICES'] = args_cli.gpu

args_cli.enable_cameras = True
args_cli.num_envs = 1

def get_config(file, default_root:Path, type:Literal['yaml', 'json']):
    if type == 'yaml':
        if file.endswith('.yml') or file.endswith('.yaml'):
            file = Path(file)
        else:
            file = default_root / f'{file}.yml'
        with open(file, 'r') as f:
            config = yaml.load(f.read(), Loader=yaml.FullLoader)
        return config, file
    else:
        if file.endswith('.json'):
            file = Path(file)
        else:
            file = default_root / f'{file}.json'
        with open(file, 'r') as f:
            config = json.load(f)
        return config, file

task_config, task_config_file = get_config(
    args_cli.config,
    default_root=Path(__file__).parent.parent / 'task_config',
    type='yaml'
)

# launch omniverse app, must done before importing anything from omni.isaac
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import importlib
if TYPE_CHECKING:
    from envs._base_task import BaseTask, BaseTaskCfg

log_path = Path('./log')
def log(msg):
    global log_path
    log_path.parent.mkdir(parents=True, exist_ok=True)

    msg = f"[{time.strftime(r'%Y-%m-%d %H:%M:%S')}] {msg}"
    with open(log_path, 'a') as f:
        f.write(msg + '\n')
    print(msg)

def run(task: 'BaseTask', episode_num, use_seed, start_seed, max_seed):
    seed = 0
    suc_map_path = task.save_root / 'suc_map.txt'
    if suc_map_path.exists():
        with open(suc_map_path, 'r', encoding='utf-8') as suc_map_file:
            suc_map = [token for token in suc_map_file.read().strip().split() if token]
    else:
        suc_map = []

    completed_successes = 0
    session_successes = 0
    attempt_count = 0

    if start_seed != -1:
        seed = start_seed
        log(f"Starting from seed {seed}.")
    elif use_seed:
        completed_successes = sum(token == '1' for token in suc_map)
        seed = len(suc_map)
        log(
            f"Use seed map with {completed_successes} successful episodes. "
            f"Starting from seed {seed}."
        )

    def record_seed_result(result_token: str):
        # Keep the token index equal to the actual seed, even for a non-zero explicit start.
        if seed >= len(suc_map):
            suc_map.extend('?' for _ in range(seed + 1 - len(suc_map)))
        suc_map[seed] = result_token

        # Replace atomically so an interrupted write does not truncate the resume map.
        tmp_path = suc_map_path.with_name(f'.{suc_map_path.name}.{os.getpid()}.tmp')
        try:
            with open(tmp_path, 'w', encoding='utf-8') as suc_map_file:
                suc_map_file.write(' '.join(suc_map))
                suc_map_file.flush()
                os.fsync(suc_map_file.fileno())
            os.replace(tmp_path, suc_map_path)
        finally:
            tmp_path.unlink(missing_ok=True)

    mean_steps = 0.0
    while completed_successes < episode_num and (max_seed == -1 or seed <= max_seed):
        attempt_count += 1
        try:
            start_t = time.perf_counter()
            task.reset(seed=seed)
            task.play_once()
            cost_t = time.perf_counter() - start_t
        except Exception:
            log(f"[{completed_successes:<3d}] Seed {seed} failed with error: {traceback.format_exc()}")
            record_seed_result('0')
            task.clean_cache(mean_steps=mean_steps, result='error')
        else:
            if task.plan_success and task.check_success() and not task.check_early_stop():
                task.save_to_hdf5()
                log(f"[{completed_successes:<3d}] Seed {seed} success in {cost_t:.2f} s.\n"
                    f"steps: {task.step_count:<5d}, save frames: {task.save_count:<5d}.\n")
                completed_successes += 1
                session_successes += 1
                record_seed_result('1')
                mean_steps = (
                    ((session_successes - 1) * mean_steps + task.step_count)
                    / session_successes
                )
                task.clean_cache(mean_steps=mean_steps, result='success')
            else:
                log(f"[{completed_successes:<3d}] Seed {seed} failed in {cost_t:.2f} s.\n"
                    f"Plan {task.plan_success}, Check {task.check_success()}")
                record_seed_result('0')
                task.clean_cache(mean_steps=mean_steps, result='fail')

        seed += 1

    success_rate = session_successes / attempt_count * 100 if attempt_count else 0.0
    log(
        f'Complete collection: target progress {completed_successes}/{episode_num}; '
        f'this run {session_successes}/{attempt_count} successful ({success_rate:.2f}%).'
    )

    if args_cli.keep_open:
        requested_visualizers = args_cli.visualizer or []
        if isinstance(requested_visualizers, str):
            requested_visualizers = [requested_visualizers]
        gui_requested = 'kit' in requested_visualizers
        if gui_requested:
            log("Collection finished. Keeping the GUI open; close the window or press Ctrl+C to exit.")
            task.sim.stop()
            try:
                while simulation_app.is_running():
                    simulation_app.update()
            except KeyboardInterrupt:
                log("Received Ctrl+C; closing Isaac Sim.")
        else:
            log("Ignoring --keep_open because Isaac Sim is running without a GUI.")

def main():
    global args_cli, task_config, task_config_file, log_path
    task_file_name = args_cli.task

    episode_num = task_config.get("episode_num", -1)
    if args_cli.episode_num != -1:
        episode_num = args_cli.episode_num
    start_seed = task_config.get("start_seed", -1)
    if args_cli.start_seed != -1:
        start_seed = args_cli.start_seed
    max_seed = task_config.get("max_seed", -1)
    if args_cli.max_seed != -1:
        max_seed = args_cli.max_seed

    task_config.update({
        "episode_num": episode_num,
        "start_seed": start_seed,
        "max_seed": max_seed,
    })

    task_module = importlib.import_module(f"envs.{task_file_name}")
    env_cfg:'BaseTaskCfg' = task_module.TaskCfg()
    env_cfg.tactile_sensor_type = task_config.get('sensor_type', 'gsmini')
    env_cfg.save_dir = Path(task_config.get("save_dir", "./data")) / task_file_name / task_config_file.stem
    env_cfg.decimation = task_config.get("decimation", env_cfg.decimation)
    env_cfg.save_frequency = task_config.get("save_frequency", env_cfg.save_frequency)
    env_cfg.video_frequency = task_config.get("video_frequency", env_cfg.video_frequency)
    env_cfg.render_frequency = task_config.get("render_frequency", env_cfg.render_frequency)
    env_cfg.obs_data_type = task_config.get("observations", {})
    env_cfg.random_texture = task_config.get("random_texture", False)

    video_resolution = task_config.get("video_resolution")
    if video_resolution is not None:
        if len(video_resolution) != 2 or any(not isinstance(value, int) or value <= 0 for value in video_resolution):
            raise ValueError("video_resolution must be [width, height] with positive integer values.")
        camera_width, camera_height = video_resolution
        if camera_width % 2 or camera_height % 2:
            raise ValueError("video_resolution width and height must be even for MPEG-4 output.")
        for camera_cfg in env_cfg.cameras:
            camera_cfg.width = camera_width
            camera_cfg.height = camera_height
        env_cfg.video_size = (camera_width * 2 + camera_height // 2, camera_height)

    env_cfg.scene.num_envs = 1

    init_start = time.perf_counter()
    task = None
    try:
        task = task_module.Task(env_cfg, mode='collect')
        init_cost = time.perf_counter() - init_start

        log_path = task.save_root / f"{time.strftime(r'%Y-%m-%d_%H:%M:%S')}.log"
        log(f"Task Name: {task_file_name}")
        log(f"Config Name: {task_config_file.stem}")
        log(f"Task Config: \n{json.dumps(task_config, ensure_ascii=False, indent=4)}\n{'-' * 20}\n")
        log(f"Env Config: \n{env_cfg}\n{'-' * 20}\n")
        log(f"Init cost {init_cost:.2f} seconds, devices: {os.environ.get('CUDA_VISIBLE_DEVICES')}")
        run(
            task,
            episode_num=episode_num,
            use_seed=task_config.get("use_seed", True),
            start_seed=start_seed,
            max_seed=max_seed,
        )
    finally:
        if task is not None:
            task.close()
        simulation_app.close()

if __name__ == "__main__":
    main()
