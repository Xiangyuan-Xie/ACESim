# ACESim

[![Python](https://img.shields.io/badge/python-3.9%2B-blue.svg)](https://www.python.org/) [![Status](https://img.shields.io/badge/status-experimental-orange.svg)](#项目状态) [![Code style](https://img.shields.io/badge/code%20style-black-000000.svg)](https://github.com/psf/black) [![pre-commit](https://img.shields.io/badge/pre--commit-enabled-brightgreen?logo=pre-commit&logoColor=white)](https://pre-commit.com/)

ACESim 是一套面向 PX4 + MuJoCo 的仿真工具集，源自 ACETele 的拆分子项目，专注于多旋翼与机械臂协同仿真、传感器建模与 PX4 HIL 接入。

---

- [ACESim](#acesim)
  - [项目简介](#项目简介)
  - [功能特性](#功能特性)
  - [项目状态](#项目状态)
  - [目录结构](#目录结构)
  - [快速开始](#快速开始)
    - [运行环境](#运行环境)
    - [安装](#安装)
    - [运行仿真](#运行仿真)
  - [配置说明](#配置说明)
  - [ROS 2 集成与部署](#ros-2-集成与部署)
  - [开发与测试](#开发与测试)
  - [贡献指南](#贡献指南)
  - [开源协议](#开源协议)
  - [致谢](#致谢)

---

## 项目简介

ACESim 旨在为 PX4 + MuJoCo 的仿真工作流提供清晰、可扩展的基础设施。当前仓库包含多旋翼动力学、传感器噪声模型、HIL 数据注入以及与 PX4 SITL 的接口封装，并支持与 ACETele 的机器人端控制逻辑协同使用。

## 功能特性

- 多旋翼仿真
  - 基于 MuJoCo 的动力学与传感器建模。
  - 支持 rotor 参数与机体配置的可配置化加载。
- PX4 HIL 接入
  - 使用 MAVLink HIL 消息与 PX4 SITL 进行闭环交互。
- ROS 2 部署
  - 提供 PX4 SITL + Micro XRCE DDS Agent 的启动脚本。
- ACETele 协作
  - MCArm 环境直接复用 ACETele 机器人控制逻辑。

## 项目状态

ACESim 目前处于实验性（experimental）阶段，接口与内部实现仍可能快速迭代。如需在工程或产品场景使用，请自行评估并增加版本锁定策略。

## 目录结构

```text
ACESim/
├─ acesim/
│  ├─ config/           仿真配置与参数
│  ├─ core/             运行入口
│  ├─ deploy/           ROS 2 部署包
│  ├─ env/              仿真环境定义
│  ├─ utils/            PX4 接口与工具
│  ├─ tools/            辅助脚本
├─ README.md            本说明文档
├─ requirements.txt     运行时依赖
├─ pyproject.toml       项目元数据与构建配置
└─ .pre-commit-config.yaml  代码质量检查配置
```

## 快速开始

### 运行环境

- 操作系统：Ubuntu（推荐）/ Windows（可配合 WSL）
- Python：3.9 及以上
- MuJoCo：通过 pip 安装对应版本
- PX4 SITL（可选）：用于 HIL 闭环验证

### 安装

```bash
git clone https://github.com/Xiangyuan-Xie/ACESim.git
cd ACESim

pip install -e .
```

### 运行仿真

默认入口为 `acesim/core/play.py`，会读取 `acesim/config/default.toml` 并启动对应环境：

```bash
python -m acesim.core.play
```

若启用 `mc_arm` 环境，需要提前安装并可导入 ACETele（`acetele` 模块）。

## 配置说明

配置由 `acesim/config/default.toml` 驱动，核心字段如下：

- `basic.sim_type`：仿真后端，目前支持 `mujoco`。
- `basic.env_type`：仿真环境类型，例如 `mc_arm`。
- `basic.asset_name`：资源名称，对应 `acesim/config/<sim_type>/` 下的参数文件。

资产参数示例见 `acesim/config/mujoco/x500_arm2x.toml`，包含旋翼方向、动力学参数等。

## ROS 2 集成与部署

`acesim/deploy/aircraft/px4_sim_ros2` 包含 PX4 SITL 与 Micro XRCE DDS Agent 的启动脚本：

```bash
ros2 launch px4_sim_ros2 linux.launch.py
```

若无法自动定位仓库路径，可设置 `ACETELE_ROOT` 指向本仓库根目录，或通过启动参数指定 `px4_repo`。

## 开发与测试

本项目通过 [pre-commit](https://pre-commit.com/) 统一代码质量检查：

```bash
pip install pre-commit
pre-commit install
pre-commit run --all-files
```

## 贡献指南

欢迎提交问题与改进建议。推荐流程：

1. Fork 本仓库并创建功能分支。
2. 在本地实现与自测，并运行 `pre-commit run --all-files`。
3. 推送并发起 Pull Request，说明改动背景与依赖。

## 开源协议

当前仓库根目录尚未提供 LICENSE 文件，因此本 README 不构成法律意义上的授权声明。请在使用前与维护者确认授权条款，并在正式对外发布前补充 LICENSE 文件。

## 致谢

感谢以下开源项目与社区支持：

- [PX4](https://px4.io/) 飞控生态
- [MuJoCo](https://mujoco.org/) 物理仿真引擎
- [ROS 2](https://www.ros.org/) 机器人软件生态
