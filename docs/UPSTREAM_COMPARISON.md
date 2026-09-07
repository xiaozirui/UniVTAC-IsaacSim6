# Upstream Comparison

Audit date: 2026-09-07 (Asia/Shanghai)

- Upstream repository: <https://github.com/univtac/UniVTAC>
- Inspected upstream `main`: `0dafa10262e22f486f160a55d6f11aeab12d8e7b`
- Inspected upstream `isaac51`: `d541e5568227ca3b66104d294f63c80acad7c52c`
- Closest measured source snapshot: `05bcd3edb92237107efa40105292a24f1a9fd761`

The public release uses a clean source-only history because the local development checkout began as an independent snapshot and bundled assets/dependencies could not all be cleared for redistribution. The closest snapshot is a comparison reference, not a claimed Git parent.

| Code line | Isaac Sim | Isaac Lab | Python |
|---|---:|---:|---:|
| Upstream `main` | 4.5.0 | 2.1.1 | 3.10 |
| Upstream `isaac51` | 5.1.0 | 2.3.0 | 3.11 |
| UniVTAC-IsaacSim6 | 6.0.1 RC | 3.0.0 beta 2 patch 1 plus compatibility work | 3.12 |

Published changes concentrate on `envs`, task execution, task configuration, runtime launch, and data collection. Policy and encoder content was unchanged relative to the closest measured upstream source and is therefore not presented as a contribution. See the root `MODIFICATIONS.md` for the file-level inventory.

Known limitations include external dependency/asset setup, incomplete broad-seed and multi-environment validation, and incomplete policy training/evaluation regression coverage.
