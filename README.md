# ACESim

[![Python](https://img.shields.io/badge/python-3.9%2B-blue.svg)](https://www.python.org/) [![Code style](https://img.shields.io/badge/code%20style-black-000000.svg)](https://github.com/psf/black) [![pre-commit](https://img.shields.io/badge/pre--commit-enabled-brightgreen?logo=pre-commit&logoColor=white)](https://pre-commit.com/)

ACESim 是一个面向 MuJoCo / Genesis 的载具仿真平台，支持多旋翼、机械臂协同、固定翼、VTOL、UUV 等场景，并支持与 PX4、ROS 2、UE5 等外部系统集成。

---

- [ACESim](#acesim)
  - [项目简介](#项目简介)
  - [功能特性](#功能特性)
  - [资产画廊](#资产画廊)
    - [多旋翼与机械臂](#多旋翼与机械臂)
    - [固定翼、VTOL 与 UUV](#固定翼vtol-与-uuv)
  - [快速开始](#快速开始)
    - [推荐：ROS 2 部署](#推荐ros-2-部署)
    - [轻量：Python 入口](#轻量python-入口)
  - [配置说明](#配置说明)
  - [可选集成](#可选集成)
    - [PX4 支持](#px4-支持)
    - [UE5 视觉流联调](#ue5-视觉流联调)
  - [资产与工具链](#资产与工具链)
  - [开发与测试](#开发与测试)
  - [贡献与许可](#贡献与许可)

---

## 项目简介

ACESim 提供统一的环境装配、仿真运行和部署入口。项目可以独立运行，也可以按需接入 PX4、ROS 2 和 UE5。

## 功能特性

- 支持 `mujoco` 与 `genesis` 两类仿真后端
- 支持 `mc`、`am`、`fw`、`vtol`、`uuv` 等环境类型
- 提供 `iris`、`x500`、`x500_arm2x`、`typhoon_h480`、`advanced_plane`、`standard_vtol`、`uuv_bluerov2_heavy` 等核心资产
- 推荐通过 ROS 2 部署，也支持直接运行 Python 入口
- 可选支持 PX4 HIL、UE5 视觉流与 SDF 资产导入流程

其中 `am` 表示 AM（Aerial Manipulator）环境。

## 资产画廊

### 多旋翼与机械臂

<table>
  <tr>
    <td align="center">
      <img src="docs/images/assets/iris.png" width="260" alt="iris" /><br />
      <strong><code>iris</code></strong><br />
      四旋翼
    </td>
    <td align="center">
      <img src="docs/images/assets/x500.png" width="260" alt="x500" /><br />
      <strong><code>x500</code></strong><br />
      四旋翼
    </td>
    <td align="center">
      <img src="docs/images/assets/x500_arm2x.png" width="260" alt="x500_arm2x" /><br />
      <strong><code>x500_arm2x</code></strong><br />
      机械臂四旋翼
    </td>
  </tr>
  <tr>
    <td align="center">
      <img src="docs/images/assets/typhoon_h480.png" width="260" alt="typhoon_h480" /><br />
      <strong><code>typhoon_h480</code></strong><br />
      六旋翼
    </td>
    <td align="center"></td>
    <td align="center"></td>
  </tr>
</table>

### 固定翼、VTOL 与 UUV

<table>
  <tr>
    <td align="center">
      <img src="docs/images/assets/advanced_plane.png" width="260" alt="advanced_plane" /><br />
      <strong><code>advanced_plane</code></strong><br />
      固定翼
    </td>
    <td align="center">
      <img src="docs/images/assets/standard_vtol.png" width="260" alt="standard_vtol" /><br />
      <strong><code>standard_vtol</code></strong><br />
      VTOL
    </td>
    <td align="center">
      <img src="docs/images/assets/uuv_bluerov2_heavy.png" width="260" alt="uuv_bluerov2_heavy" /><br />
      <strong><code>uuv_bluerov2_heavy</code></strong><br />
      UUV
    </td>
  </tr>
</table>

如需重新生成画廊图片：

```bash
python -m acesim.tools.render_readme_assets
```

## 快速开始

默认安装只包含公共 Python 依赖；仿真后端通过 extras 按需安装：

- `numpy`
- `scipy`
- `tqdm`
- `pymavlink`
- `pyzmq`

可选后端依赖如下：

- `.[mujoco]`：安装 MuJoCo 后端所需的 `mujoco`
- `.[genesis]`：安装 Genesis 后端所需的 `genesis-world`
- `.[all]`：同时安装 MuJoCo 与 Genesis 后端

### 推荐：ROS 2 部署

当前默认配置使用 `sim_type = "mujoco"`，因此推荐安装 MuJoCo extra：

```bash
git clone https://github.com/Xiangyuan-Xie/ACESim.git
cd ACESim
pip install -e ".[mujoco]"
ros2 launch acesim_ros2 linux.launch.py
```

如果只需要 Genesis 后端，可以改用：

```bash
pip install -e ".[genesis]"
```

如果希望一次安装全部后端：

```bash
pip install -e ".[all]"
```

ROS 2 包位于 `acesim/deploy/aircraft/acesim_ros2`，推荐在完整部署、bridge、时钟同步和联调场景下使用。

对于 ACESim 中的 AM Position：
- 需要有效的 manual-control source，例如 QGC virtual joystick 或 RC。
- 这一要求由模式实现本身固定提供，与 Position 模式对齐，而不是由 launch 参数覆写决定。
- 摇杆回中不会阻止进入模式，而是进入保持行为。
- 如果修改了 `acesim/deploy/aircraft/px4_msgs` 下的 `.msg` / `.srv` 接口，首次需要对 ROS 2 工作区做一次 clean rebuild。

补充入口：

- headless launch：`acesim/deploy/aircraft/acesim_ros2/launch/linux_headless.launch.py`
- 可执行入口：`acesim_ros2.acesim_play`
- headless 可执行入口：`acesim_ros2.acesim_play_headless`

### 轻量：Python 入口

如果只做本地验证，可以直接运行：

```bash
pip install -e ".[mujoco]"
python -m acesim.core.play
```

这个入口适合快速检查配置装配、环境切换和本地仿真流程。

## 配置说明

`python -m acesim.core.play` 与 ROS 2 播放入口都会读取 `acesim/config/default.toml`。当前默认值为：

```toml
[basic]
sim_type = "mujoco"
env_type = "am"
scene_name = "default"
asset_name = "x500_arm2x"
benchmark = "multirotor"
```

核心字段如下：

- `basic.sim_type`：仿真后端，例如 `mujoco`、`genesis`
- `basic.env_type`：环境类型，例如 `mc`、`am`、`fw`、`vtol`、`uuv`
- `basic.scene_name`：场景名
- `basic.asset_name`：资产参数文件名
- `basic.benchmark`：基准测试或运行分组字段

ACESim 会先读取顶层 `basic` 配置，再加载 `acesim/config/<sim_type>/<asset_name>.toml` 中的资产参数。

如果你要切换后端、环境类型或资产，优先修改 `default.toml` 中的 `basic` 段。

## 可选集成

### PX4 支持

ACESim 支持接入 PX4，但它是可选能力，不是使用门槛。当前 PX4 相关逻辑主要由以下模块组织：

- `acesim/utils/px4_transport.py`
- `acesim/utils/px4_sensor_scheduler.py`

测试已覆盖的 PX4 启动映射资产包括：

- `iris`
- `x500`
- `x500_arm2x`
- `typhoon_h480`
- `advanced_plane`
- `standard_vtol`
- `uuv_bluerov2_heavy`

如未显式传入 PX4 仓库路径，ROS 2 启动逻辑默认会尝试使用：

```text
acesim/third_party/aircraft/PX4-Autopilot
```

### UE5 视觉流联调

ACESim 支持把载具位姿与旋翼视觉状态通过 ZeroMQ 发布给外部渲染端。默认 endpoint 为：

```text
tcp://0.0.0.0:5601
```

UE5 相关工具位于：

- `acesim/tools/ue5/README.md`
- `acesim/tools/ue5/check_ubuntu_ue5_host.sh`
- `acesim/tools/ue5/setup_ubuntu_ue5.sh`
- `acesim/tools/ue5/create_project_scaffold.py`
- `acesim/tools/ue5/verify_visual_stream.py`

如果只是验证视觉流链路，建议先运行 `verify_visual_stream.py`；如果要完整搭建 UE5 项目，再阅读 `acesim/tools/ue5/README.md`。

## 资产与工具链

仓库内的常用工具包括：

- `acesim/tools/sdf2urdf/`
  - 第一阶段转换模块：从 SDF 源同步 ACESim 手工维护的 URDF 资产
- `acesim/tools/urdf2mjcf/`
  - 第二阶段转换模块：消费 URDF，生成并后处理 MJCF
- `acesim/tools/render_readme_assets.py`
  - 生成 README 资产预览图
- `acesim/tools/cal_dynamic_params.py`
  - 动力学参数辅助计算
- `acesim/tools/cal_thrust_coef.py`
  - 推力系数辅助计算

当前 `sdf2urdf` 的接口是面向通用 SDF 来源设计的，但仓库内已落地的 provider 主要是 `px4`。

最小两阶段工作流如下：

```bash
python -m acesim.tools.sdf2urdf --source px4 --target advanced_plane
python -m acesim.tools.urdf2mjcf --target advanced_plane
```

这两个阶段的职责边界是：

- `sdf2urdf`：读取上游 SDF 真值，生成或同步本地 meshes，并更新 ACESim 使用的 URDF。
- `urdf2mjcf`：在 URDF 已准备好的前提下，完成 MuJoCo 编译和 MJCF 后处理。

如果你只是修正上游 SDF 对应的 visual / joint / inertial truth，先运行 `sdf2urdf` 即可；如果你还需要刷新 MuJoCo 资产产物，再继续运行 `urdf2mjcf`。

## 开发与测试

运行测试：

```bash
pytest
```

当前测试覆盖的能力面包括：

- MuJoCo 默认配置与多资产 headless 启动
- PX4 传感器调度与执行器控制读取
- PX4 SDF 资产导入管线
- ROS 2 launch 组装逻辑
- 视觉流 payload 编码 / 解码
- 固定翼、VTOL、UUV 动力学关键行为

代码质量检查：

```bash
pip install pre-commit
pre-commit install
pre-commit run --all-files
```

## 贡献与许可

欢迎提交问题、修复和改进建议。推荐本地流程：

1. 创建分支并完成改动。
2. 运行 `pytest` 与 `pre-commit run --all-files`。
3. 在提交说明或 PR 中写清楚改动背景、影响范围和验证方式。

当前仓库根目录未提供正式的 `LICENSE` 文件，因此本 README 不构成法律意义上的授权声明。如果你准备在正式项目中复用本仓库内容，建议先与维护者确认授权条款。
