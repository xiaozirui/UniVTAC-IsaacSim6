# Modifications

This file records the source changes published by the unofficial UniVTAC-IsaacSim6 derivative. It does not transfer copyright in upstream contributions.

Comparison reference: upstream UniVTAC `05bcd3edb92237107efa40105292a24f1a9fd761` (closest measured tree; not a proven Git parent).

## Environment and task execution

| Path | Category | Contribution |
|---|---|---|
| `envs/_base_task.py` | Isaac Lab 3 migration | Application lifecycle, scene setup, rendering, observations, collection, and reset behavior |
| `envs/_global.py` | Release portability | External licensed asset-root configuration |
| `envs/collect.py` | Camera migration | Isaac Lab 3 camera/quaternion behavior |
| `envs/grasp_classify.py` | Camera/task migration | Camera boundary and task execution |
| `envs/insert_HDMI.py` | Task robustness | Grasp/insertion trajectory and physical success checks |
| `envs/insert_hole.py` | Task robustness | Grasp/insertion sequence and complete trajectory capture |
| `envs/insert_tube.py` | Task robustness | Grasp, collision, insertion, and success behavior |
| `envs/lift_bottle.py` | Task robustness | Adaptive grasp and lift behavior |
| `envs/lift_can.py` | Task robustness | Contact/friction and lift success |
| `envs/pull_out_key.py` | Task robustness | Grasp closure, friction, extraction, and success behavior |
| `envs/put_bottle_in_shelf.py` | Task robustness | Grasp, shelf-safe approach, release, and stability checks |
| `envs/robot/*` | Runtime compatibility | Isaac Lab 3 robot state and cuRobo integration boundary |
| `envs/sensors/*` | Sensor compatibility | Camera/tactile data handling and render synchronization |
| `envs/utils/*` | API compatibility | Actor/view, pose, transform, data, and task primitive migration |

## Runtime and data collection

| Path | Category | Contribution |
|---|---|---|
| `collect_data.sh` | Portable launcher | Route collection through the external Isaac Sim 6 runtime |
| `parallel_collect.sh` | Collection orchestration | Isaac Sim 6 parallel collection entry point |
| `scripts/collect_data.py` | Data collection | Launch, interruption, bounded seeds, clean shutdown, and video handling |
| `scripts/parallel_collect_data.py` | Parallel collection | Process lifecycle, task arguments, and output handling |
| `scripts/replay.py` | Replay compatibility | Isaac Lab 3 launch and recorded observation compatibility |
| `scripts/run_isaacsim6.sh` | New runtime launcher | External simulator/CUDA environment configuration |
| `scripts/visualize.py` | Data compatibility | Observation and video visualization |
| `task_config/*` | Task configuration | Demonstration, contact, and bounded validation settings |

## Intentionally not published here

The complete development checkout also contains local dependency compatibility work. It is excluded from this source-only history until the relevant third-party redistribution and provenance questions are resolved. Policy and encoder content is also excluded because it was not modified by this migration.
