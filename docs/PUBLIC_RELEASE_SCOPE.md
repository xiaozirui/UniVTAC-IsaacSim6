# Public Release Scope

This branch is intentionally limited to the UniVTAC-IsaacSim6 task-environment and data-collection source contribution.

## Included

- `envs/`: manipulation environments, robot/sensor integration boundaries, and task utilities;
- `task_config/`: demonstration, contact, and validation configuration;
- collection, replay, visualization, and Isaac Sim 6 launcher scripts;
- bilingual project documentation, attribution, comparison, and audit records;
- the upstream root Apache License 2.0 text.

## Excluded

- all `assets/` content;
- all `third_party/` source and gitlinks;
- policy and encoder trees unchanged by this port;
- models, weights, calibration arrays, datasets, videos, textures, USD/mesh assets, binaries, build output, and local environments;
- local dependency commits for Isaac Lab, TacEx/UIPC, and cuRobo.

The excluded development components are not licensed by the root project license merely because they appeared in an upstream checkout. Users must obtain authorized dependencies and assets separately.

## Runtime boundary

Set `UNIVTAC_ISAACSIM_ROOT` to the external Isaac Sim installation and `UNIVTAC_ASSETS_ROOT` to an external, appropriately licensed asset tree. This first source release is intended to document and share the compatibility contribution; it is not an out-of-the-box runnable distribution.
