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

## 已实现贡献与创新思路

本项目并非只修改版本号，已经落地的贡献包括：

- 建立 Isaac Lab 3 兼容边界，适配应用生命周期、scene cloning、位姿/四元数约定、相机 API 和可视化模式。
- 显式同步 UIPC、Fabric 与渲染状态，使触觉 marker 和 RTX 观测随物理状态正确更新。
- 面向接触任务的稳定化：适配夹爪闭合、摩擦/接触、避碰运动以及 Isaac Sim 6 下的恢复 workspace。
- 为插入、拔出、提升和货架放置增加物理成功判定与任务专用轨迹，而不只依赖运动序列正常结束。
- 改进完整轨迹采集、确定性 seed、失败次数边界、干净退出以及隔离验证输出，服务科研数据生产。
- 使用环境变量隔离模拟器安装与机器路径，形成可维护的运行入口。

以上属于代码和本地验证能够支持的工程与任务执行贡献。本项目不会把尚未实现的新学习策略架构写成已完成成果。

## 后续科研创新方向

本项目计划形成如下研究链路：

```text
UniVTAC upstream
    -> Isaac Sim 6 / Isaac Lab 3 兼容层
    -> 稳定的接触丰富任务执行
    -> 可复现的视触觉数据采集
    -> 触觉表征与 VLA 研究
    -> 项目自有的新操作算法
```

后续重点包括触觉表征、contact-aware VLA、视觉/动作/触觉跨模态对齐、失败感知数据治理以及新的闭环操作算法。这些内容被明确标记为研究方向，而不是已经完成的结果。

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

```bash
bash scripts/run_isaacsim6.sh scripts/collect_data.py \
  lift_can task_config/demo.yml \
  --episode_num 1 --start_seed 0 --max_seed 0 --gpu 0 --viz none
```

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
