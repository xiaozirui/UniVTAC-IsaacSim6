# UniVTAC-IsaacSim6

**An unofficial compatibility and research extension of [UniVTAC](https://github.com/univtac/UniVTAC) for NVIDIA Isaac Sim 6.0.1 and Isaac Lab 3.0.0.**

[English](README.md) | [简体中文](README_zh-CN.md)

> [!IMPORTANT]
> This is a source-only public release. Third-party source trees, robot and sensor assets, models, calibration data, datasets, binaries, and media are deliberately excluded until their redistribution terms are verified. This repository is not a self-contained simulator installation.

## Project Overview

UniVTAC-IsaacSim6 ports UniVTAC manipulation-task environments and data collection to the Isaac Sim 6 / Isaac Lab 3 runtime. It preserves upstream attribution while providing a clean base for continued visuo-tactile manipulation research.

The complete internal development checkout has executed all eight listed tasks successfully in at least one smoke case. This public repository contains the project-authored compatibility and task logic, not unresolved third-party packages or assets.

## Relationship to UniVTAC

- Upstream: <https://github.com/univtac/UniVTAC>
- Paper: Chen et al., *UniVTAC: A Unified Simulation Platform for Visuo-Tactile Manipulation Data Generation, Learning, and Benchmarking*
- Closest measured upstream comparison snapshot: `05bcd3edb92237107efa40105292a24f1a9fd761`

This is an unofficial derivative project, not an official UniVTAC release, successor, or NVIDIA-endorsed implementation. The local project began as an independent snapshot, so the comparison commit is not claimed as a proven Git parent.

The upstream README and root LICENSE currently disagree about the license name. This project conservatively preserves the upstream root Apache License 2.0 and its redistribution requirements without making a broader legal conclusion.

## Why This Port Exists

Isaac Sim 6 and Isaac Lab 3 changed application launch, camera and pose APIs, scene/view behavior, rendering synchronization, and interfaces used by TacEx/UIPC. This port adapts those boundaries while retaining the original manipulation-task organization.

## Version Compatibility

| Code line | Isaac Sim | Isaac Lab | Python |
|---|---:|---:|---:|
| Upstream `main` (`0dafa102`) | 4.5.0 | 2.1.1 | 3.10 |
| Upstream `isaac51` (`d541e556`) | 5.1.0 | 2.3.0 | 3.11 |
| UniVTAC-IsaacSim6 | 6.0.1 RC | 3.0.0 beta 2 patch 1 plus compatibility work | 3.12 |

## Implemented Contributions and Innovation

The implemented contribution is more than a version-number update:

- A compatibility boundary for Isaac Lab 3 application lifecycle, scene cloning, pose/quaternion conventions, camera APIs, and visualization modes.
- Explicit UIPC-to-Fabric/render synchronization so tactile-marker and RTX observations advance with the physics state.
- Contact-aware task stabilization: grasp-closing behavior, friction/contact tuning, collision-safe motion, and recovery workspaces adapted to the Isaac Sim 6 runtime.
- Physical success checks and task-specific trajectories for insertion, extraction, lifting, and shelf placement instead of relying only on nominal motion completion.
- Complete-trajectory capture, deterministic seed controls, bounded failure handling, clean shutdown, and isolated validation outputs for research data collection.
- Portable launch configuration that keeps simulator installations and machine paths outside the repository.

These are engineering and task-execution contributions established by the code and local validation. This repository does not claim a new learned-policy architecture where none has yet been implemented.

## Research Innovation Direction

This port is designed as the lower layer of a longer research program:

```text
UniVTAC upstream
    -> Isaac Sim 6 / Isaac Lab 3 compatibility
    -> stable contact-rich task execution
    -> reproducible visuo-tactile data collection
    -> tactile representation and VLA research
    -> new project-owned manipulation algorithms
```

Planned research includes tactile representations, contact-aware VLA policies, cross-modal alignment between vision/action/touch, failure-aware data curation, and new closed-loop manipulation algorithms. These are clearly marked as future directions rather than completed results.

## Installation

Obtain all dependencies and assets separately from their authorized distributors:

1. Install NVIDIA Isaac Sim 6.0.1 outside this repository.
2. Install a compatible Isaac Lab 3 environment.
3. Obtain compatible TacEx/UIPC and cuRobo revisions separately.
4. Supply licensed UniVTAC-compatible robot, sensor, object, scene, and texture assets outside this repository.
5. Configure local paths:

```bash
export UNIVTAC_ISAACSIM_ROOT=/path/to/isaac-sim
export UNIVTAC_ASSETS_ROOT=/path/to/licensed-univtac-assets
bash scripts/run_isaacsim6.sh -c "import isaaclab; print('Isaac Lab import OK')"
```

The external compatibility patches are not part of this first public source release. See [PUBLIC_RELEASE_SCOPE.md](docs/PUBLIC_RELEASE_SCOPE.md).

## Supported Tasks

| Task | Internal smoke status | Seed |
|---|---|---:|
| `grasp_classify` | success | 0 |
| `lift_bottle` | success | 0 |
| `lift_can` | success | 0 |
| `insert_HDMI` | success | 0 |
| `insert_hole` | success | 1 |
| `insert_tube` | success | 0 |
| `pull_out_key` | success | 0 |
| `put_bottle_in_shelf` | success | 0 |

These are single-case smoke results, not robustness or benchmark claims. Reproduction requires compatible external dependencies and licensed assets.

## Data Collection

```bash
bash scripts/run_isaacsim6.sh scripts/collect_data.py \
  lift_can task_config/demo.yml \
  --episode_num 1 --start_seed 0 --max_seed 0 --gpu 0 --viz none
```

Generated datasets, logs, videos, checkpoints, and evaluation outputs are excluded from Git.

## Current Status and Roadmap

- Public now: task environments, collection/runtime source, task configs, bilingual documentation, attribution, and audit records.
- External for now: assets, TacEx/UIPC, cuRobo, Isaac Lab patches, models, calibration data, and binaries.
- Next: publish reproducible dependency patches after license review, add an asset provenance manifest, broaden seed testing, and add project-owned tactile/VLA algorithms under a separate `research/` tree.

## Upstream Comparison

See [UPSTREAM_COMPARISON.md](docs/UPSTREAM_COMPARISON.md) and [MODIFICATIONS.md](MODIFICATIONS.md).

## Acknowledgements

This project contains code derived from UniVTAC. Original UniVTAC authors retain copyright in their original contributions. NVIDIA Isaac Sim, Isaac Lab, TacEx, UIPC/libuipc, cuRobo, and separately obtained assets remain subject to their respective licenses. See [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md).

## Citation

```bibtex
@article{chen2026univtac,
  title={UniVTAC: A Unified Simulation Platform for Visuo-Tactile Manipulation Data Generation, Learning, and Benchmarking},
  author={Chen, Baijun and Wan, Weijie and Chen, Tianxing and Guo, Xianda and Xu, Congsheng and Qi, Yuanyang and Zhang, Haojie and Wu, Longyan and Xu, Tianling and Li, Zixuan and others},
  journal={arXiv preprint arXiv:2602.10093},
  year={2026}
}
```

## License and Disclaimer

The root [LICENSE](LICENSE) preserves the complete Apache License 2.0 text found in upstream UniVTAC. It does not license separately obtained dependencies or assets. UniVTAC-IsaacSim6 is provided without warranty and is not an official UniVTAC or NVIDIA project.
