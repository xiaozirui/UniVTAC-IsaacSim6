# UniVTAC-IsaacSim6

**面向 NVIDIA Isaac Sim 6.0.1 与 Isaac Lab 3.0.0 的非官方 [UniVTAC](https://github.com/univtac/UniVTAC) 兼容迁移与科研扩展项目。**

[English](README.md) | [简体中文](README_zh-CN.md)

> [!IMPORTANT]
> 这是仅含源码的公开版本。第三方源码树、机器人和传感器资产、模型、标定数据、数据集、二进制及媒体文件，在再分发条款确认前均被有意排除。因此本仓库不是完整的模拟器安装包。

## 项目概述

UniVTAC-IsaacSim6 将 UniVTAC manipulation task 环境和数据采集迁移到 Isaac Sim 6 / Isaac Lab 3，并在保留上游归属的基础上，为后续视触觉操作研究提供清晰、长期维护的代码基础。

完整内部开发环境已经为下列八个任务各运行过至少一个成功案例。公开仓库只包含本项目的兼容迁移与任务逻辑，不包含授权范围尚未确认的第三方包和资产。

## 与 UniVTAC 的关系

- 上游仓库：<https://github.com/univtac/UniVTAC>
- 原论文：Chen et al., *UniVTAC: A Unified Simulation Platform for Visuo-Tactile Manipulation Data Generation, Learning, and Benchmarking*
- 经目录树相似度测得的最近上游参考：`05bcd3edb92237107efa40105292a24f1a9fd761`

本项目是非官方衍生项目，不是 UniVTAC 官方发布、官方后继版本或 NVIDIA 官方实现。本地项目由独立快照初始化，因此不会把上述比较提交声称为能够由 Git 证明的父提交。

上游 README 与根 LICENSE 对许可证名称的表述不一致。本项目采取保守方式，保留上游根目录完整的 Apache License 2.0 及其再分发要求，不作超出仓库证据的法律推断。

## 为什么需要这个迁移

Isaac Sim 6 和 Isaac Lab 3 改变了应用启动、相机与位姿 API、scene/view 行为、渲染同步以及 TacEx/UIPC 使用的多个接口。本项目在保留原任务组织结构的同时迁移这些边界。

## 版本兼容性

| 代码线 | Isaac Sim | Isaac Lab | Python |
|---|---:|---:|---:|
| 上游 `main`（`0dafa102`） | 4.5.0 | 2.1.1 | 3.10 |
| 上游 `isaac51`（`d541e556`） | 5.1.0 | 2.3.0 | 3.11 |
| UniVTAC-IsaacSim6 | 6.0.1 RC | 3.0.0 beta 2 patch 1 加兼容修改 | 3.12 |

## 技术贡献

当前已经实现并由源码支持的系统与任务级贡献包括：

- **仿真兼容层。** 将应用生命周期、scene cloning、位姿与四元数约定、相机接口和可视化控制迁移至 Isaac Sim 6 / Isaac Lab 3 的执行机制。
- **同步视触觉观测。** 显式协调 UIPC 状态更新、Fabric 同步与渲染过程，使触觉 marker 观测和 RTX 相机帧对应同一仿真状态。
- **接触丰富任务稳定化。** 针对新运行时适配夹爪闭合、接触与摩擦参数、避碰接近运动以及 episode 级恢复 workspace。
- **基于物理状态的任务判定。** 针对提升、插入、拔出与货架放置任务，根据物体状态和几何关系判断成功，而非仅依据运动序列是否执行结束。
- **面向科研的数据采集机制。** 支持完整轨迹记录、确定性 seed、失败次数边界、受控退出与隔离验证输出。
- **可移植运行边界。** 通过环境配置将项目源码与模拟器安装、机器路径、生成数据和外部资产解耦。

上述表述仅覆盖公开源码中能够验证的行为和已报告的单案例测试。本版本不将尚未实现的学习策略架构描述为已完成成果。

## 拟议算法研究框架

本兼容层面向 contact-aware 视触觉策略研究：几何规划器提供名义动作，触觉反馈提供闭环残差修正。以下内容属于后续算法设计，而非已经完成或经过 benchmark 的算法结果。

### 多模态状态表征

在时间步 `t`，策略接收同步的视觉观测 `oᵛ_t`、触觉 RGB/marker/depth 观测 `oᵗ_t`、机器人本体状态 `q_t` 以及可选语言指令 `l`。各模态编码器生成共享 token 序列：

```text
z_t = Fuse(E_v(oᵛ_t), E_t(oᵗ_t), E_p(q_t), E_l(l)).
```

触觉编码器将保留局部 marker 位移和接触几何，而非将触觉简化为二值接触信号。数据采集层负责时间同步，避免跨模态模型学习由仿真更新错位产生的伪相关。

### 接触阶段条件控制

接触状态估计器预测接触状态 `c_t` 与操作阶段 `p_t`，例如自由空间接近、初始接触、受约束操作和释放。控制器将运动规划动作与学习得到的残差组合：

```text
a_t = a_plan,t + g(c_t, p_t) · Δa_θ(z_≤t, c_t, p_t),
```

其中 `g` 用于限制非接触敏感阶段的学习修正。该混合结构旨在保留 cuRobo 类几何规划的可靠性，同时利用触觉反馈修正夹爪闭合、插入对准、滑移和接触力不平衡。

### 失败感知数据与恢复闭环

成功轨迹构成任务示范，失败 seed 则作为结构化 hard negative 保留。基于物理状态的成功判定为恢复策略提供监督，使系统能够执行重新抓取、回撤、重新对准或重新规划。数据集由此同时记录接触阶段、失败模式和恢复结果，而不只保留成功终止标签。

### 学习目标

候选训练目标联合动作预测、接触状态估计、跨模态对齐与结果预测：

```text
L = λ_a L_action + λ_c L_contact + λ_x L_cross-modal + λ_s L_success.
```

模块化目标便于在不修改任务接口的前提下，对 vision-only、touch-only、early fusion、late fusion 以及 language-conditioned 方案进行受控消融。

### 评测方法

拟议框架应从多 seed 任务成功率、接触阶段识别、恢复成功率、视觉/触觉消融、传感器与纹理随机化、执行时延以及跨传感器和跨任务迁移等维度评测。后续算法实现将进入独立 `research/` 目录，并与兼容迁移结果分别报告。

## 安装

请从获得授权的官方发布渠道单独获取依赖和资产：

1. 在仓库外安装 NVIDIA Isaac Sim 6.0.1。
2. 安装兼容的 Isaac Lab 3 环境。
3. 单独获取兼容的 TacEx/UIPC 与 cuRobo revision。
4. 在仓库外准备已获许可的机器人、传感器、物体、场景和纹理资产。
5. 配置路径：

```bash
export UNIVTAC_ISAACSIM_ROOT=/path/to/isaac-sim
export UNIVTAC_ASSETS_ROOT=/path/to/licensed-univtac-assets
bash scripts/run_isaacsim6.sh -c "import isaaclab; print('Isaac Lab import OK')"
```

第一版公开源码不包含外部依赖兼容补丁，详见 [PUBLIC_RELEASE_SCOPE.md](docs/PUBLIC_RELEASE_SCOPE.md)。

## 支持任务

| 任务 | 内部 smoke 状态 | Seed |
|---|---|---:|
| `grasp_classify` | 成功 | 0 |
| `lift_bottle` | 成功 | 0 |
| `lift_can` | 成功 | 0 |
| `insert_HDMI` | 成功 | 0 |
| `insert_hole` | 成功 | 1 |
| `insert_tube` | 成功 | 0 |
| `pull_out_key` | 成功 | 0 |
| `put_bottle_in_shelf` | 成功 | 0 |

这些是单案例 smoke 结果，不代表 benchmark 或大范围鲁棒性结论。复现需要兼容的外部依赖和已获许可资产。

## 数据采集

### 单任务采集

```bash
bash scripts/run_isaacsim6.sh scripts/collect_data.py \
  lift_can task_config/demo.yml \
  --episode_num 1 --start_seed 0 --max_seed 0 --gpu 0 --viz none
```

### 八任务串行采集

使用批处理入口可以通过一条命令，将同一套采集参数依次应用到全部八个支持任务：

```bash
bash scripts/collect_all_tasks.sh \
  --config task_config/demo.yml \
  --episode_num 1 \
  --start_seed 0 \
  --max_seed 20 \
  --gpu 0 \
  --viz none
```

每次执行均须显式提供以上六项参数。`--config` 用于选择 YAML 采集配置（也可使用
`--yaml` 别名）；`--episode_num` 表示每个任务本次需要采集的成功 episode 数；
`--start_seed` 和 `--max_seed` 定义闭区间 seed 搜索范围；`--gpu` 与 `--viz`
会不作修改地传递给每一个任务。

八个任务严格串行执行：当前 Isaac Sim 进程完全退出后，脚本才启动下一个任务。
单个任务失败会被记录，但不会阻止剩余任务继续运行。各任务终端日志和包含退出码的
汇总表保存在 `data/_batch_runs/<时间戳>/`，采集数据沿用
`data/<任务名>/<配置名>/` 目录结构。

生成的数据集、日志、视频、checkpoint 和评测输出均不进入 Git。

## 当前状态与路线图

- 当前公开：任务环境、数据采集/运行源码、任务配置、双语文档、归属与审计记录。
- 暂时外置：assets、TacEx/UIPC、cuRobo、Isaac Lab patch、模型、标定数据和二进制。
- 下一步：发布通过许可证检查的可复现依赖补丁；建立资产来源清单；扩大 seed 验证；在独立 `research/` 目录加入项目自有 tactile/VLA 算法。

## 上游比较

见 [UPSTREAM_COMPARISON.md](docs/UPSTREAM_COMPARISON.md) 和 [MODIFICATIONS.md](MODIFICATIONS.md)。

## 致谢

本项目包含派生自 UniVTAC 的代码，UniVTAC 原作者保留其原始贡献的版权。NVIDIA Isaac Sim、Isaac Lab、TacEx、UIPC/libuipc、cuRobo 及单独获得的资产仍分别受各自许可证约束。详见 [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)。

## 引用

```bibtex
@article{chen2026univtac,
  title={UniVTAC: A Unified Simulation Platform for Visuo-Tactile Manipulation Data Generation, Learning, and Benchmarking},
  author={Chen, Baijun and Wan, Weijie and Chen, Tianxing and Guo, Xianda and Xu, Congsheng and Qi, Yuanyang and Zhang, Haojie and Wu, Longyan and Xu, Tianling and Li, Zixuan and others},
  journal={arXiv preprint arXiv:2602.10093},
  year={2026}
}
```

## 许可证与免责声明

根 [LICENSE](LICENSE) 保留上游 UniVTAC 根目录中的完整 Apache License 2.0。它不会授权单独获得的依赖和资产。UniVTAC-IsaacSim6 不提供任何担保，也不是 UniVTAC 或 NVIDIA 官方项目。
