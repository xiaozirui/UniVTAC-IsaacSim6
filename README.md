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

## Technical Contributions

The implemented scope comprises the following system and task-level contributions:

- **Simulator compatibility layer.** Migration of application lifecycle, scene cloning, pose and quaternion conventions, camera interfaces, and visualization control to the Isaac Sim 6 / Isaac Lab 3 execution model.
- **Synchronized visuo-tactile observations.** Explicit coordination of UIPC state updates, Fabric synchronization, and rendering so tactile-marker observations and RTX camera frames correspond to the same simulated state.
- **Contact-rich task stabilization.** Runtime-specific adaptation of gripper closure, contact and friction parameters, collision-aware approach motion, and per-episode recovery workspaces.
- **Physics-grounded task verification.** Task-specific success criteria for lifting, insertion, extraction, and shelf placement based on object state and geometric relations rather than motion completion alone.
- **Research-grade data collection.** Complete trajectory recording, deterministic seed control, bounded failure handling, controlled shutdown, and isolated validation outputs.
- **Portable runtime boundary.** Environment-based configuration that separates project source from simulator installations, machine paths, generated data, and external assets.

These claims are restricted to behavior represented in the published source and the reported single-case validation. No new learned-policy architecture is claimed as an implemented result in this release.

## Proposed Algorithmic Research Framework

The compatibility layer is intended to support a contact-aware visuo-tactile policy architecture in which geometric planning provides a nominal action and tactile feedback supplies closed-loop residual correction. This section defines a research design, not a completed or benchmarked algorithm.

### Multimodal state representation

At time step `t`, the policy receives synchronized visual observations `oᵛ_t`, tactile RGB/marker/depth observations `oᵗ_t`, robot proprioception `q_t`, and an optional language instruction `l`. Modality-specific encoders produce a shared token sequence:

```text
z_t = Fuse(E_v(oᵛ_t), E_t(oᵗ_t), E_p(q_t), E_l(l)).
```

The tactile encoder is expected to preserve local marker displacement and contact geometry instead of reducing touch to a binary contact flag. Temporal alignment is enforced at the collection layer so that cross-modal learning does not absorb simulator-induced observation skew.

### Contact-phase-conditioned control

A contact-state estimator predicts contact state `c_t` and manipulation phase `p_t`, such as free-space approach, initial contact, constrained manipulation, or release. The controller combines a nominal motion-planning action with a learned residual:

```text
a_t = a_plan,t + g(c_t, p_t) · Δa_θ(z_≤t, c_t, p_t),
```

where `g` limits learned corrections outside contact-sensitive phases. This hybrid formulation is designed to retain the geometric reliability of cuRobo-style planning while enabling tactile correction for grasp closure, insertion alignment, slip, and contact-force imbalance.

### Failure-aware data and recovery loop

Successful trajectories provide task demonstrations, while failed seeds are retained as structured hard negatives. Physics-based success checks supervise a recovery policy that may regrasp, retract, realign, or replan. The resulting dataset associates observations and actions with contact phase, failure mode, and recovery outcome rather than storing only successful terminal labels.

### Learning objectives

A candidate training objective combines action prediction, contact-state estimation, cross-modal alignment, and outcome prediction:

```text
L = λ_a L_action + λ_c L_contact + λ_x L_cross-modal + λ_s L_success.
```

The modular objective permits controlled ablations of vision-only, touch-only, early-fusion, late-fusion, and language-conditioned variants without changing the task interface.

### Evaluation methodology

The proposed framework should be evaluated through multi-seed task success, contact-phase accuracy, recovery success, visual/tactile ablations, sensor and texture randomization, execution latency, and transfer across tactile sensors and manipulation tasks. Future algorithm implementations will be placed in a separate `research/` tree and reported independently from the compatibility results.

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
