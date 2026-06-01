<a id="readme-top"></a>

<div align="center">

<p align="center">
  <a href="https://www.python.org/"><img src="https://img.shields.io/badge/python-3.9%2B-blue.svg" alt="Python" /></a>
  <a href="https://github.com/psf/black"><img src="https://img.shields.io/badge/code%20style-black-000000.svg" alt="Code style: black" /></a>
  <a href="https://pre-commit.com/"><img src="https://img.shields.io/badge/pre--commit-enabled-brightgreen?logo=pre-commit&logoColor=white" alt="pre-commit" /></a>
</p>

<h1 align="center">ACESim</h1>

<p align="center">
  面向多域载具的 MuJoCo / Genesis 仿真平台。
</p>

<p align="center">
  <strong>简体中文</strong>
  ·
  <a href="README.en.md">English</a>
</p>

</div>

<details>
  <summary>目录</summary>
  <ol>
    <li><a href="#项目简介">项目简介</a></li>
    <li><a href="#技术栈">技术栈</a></li>
    <li>
      <a href="#快速开始">快速开始</a>
      <ul>
        <li><a href="#环境要求">环境要求</a></li>
        <li><a href="#安装">安装</a></li>
      </ul>
    </li>
    <li>
      <a href="#使用">使用</a>
      <ul>
        <li><a href="#资产画廊">资产画廊</a></li>
        <li><a href="#配置">配置</a></li>
        <li><a href="#ros-2--px4">ROS 2 / PX4</a></li>
        <li><a href="#资产工具链">资产工具链</a></li>
      </ul>
    </li>
    <li><a href="#路线图">路线图</a></li>
    <li><a href="#贡献">贡献</a></li>
    <li><a href="#许可证">许可证</a></li>
    <li><a href="#联系">联系</a></li>
    <li><a href="#致谢">致谢</a></li>
  </ol>
</details>

## 项目简介

ACESim 提供统一的环境装配、仿真运行、资产转换和部署入口。它可以作为轻量 Python 仿真入口独立运行，也可以通过 ROS 2 launch、bridge 插件和 PX4 仓库路径接入更完整的飞控联调流程。

核心能力：

- 支持 `mujoco` 与 `genesis` 两类仿真后端。
- 支持 `mc`、`am`、`fw`、`vtol`、`uuv` 等环境类型，其中 `am` 表示 AM（Aerial Manipulator）环境。
- 提供 `iris`、`x500`、`x500_arm2x`、`typhoon_h480`、`advanced_plane`、`standard_vtol`、`uuv_bluerov2_heavy` 等核心资产。
- 推荐通过 ROS 2 部署完整联调流程，也支持直接运行 Python 入口做本地验证。
- 可选支持 PX4 HIL、ROS 2 bridge、仿真时钟同步和 SDF 资产导入流程。

<p align="right">(<a href="#readme-top">返回顶部</a>)</p>

## 技术栈

主要技术栈和运行时依赖：

- [Python](https://www.python.org/) 3.9+
- [MuJoCo](https://mujoco.org/)
- [Genesis](https://genesis-world.readthedocs.io/)
- [ROS 2](https://docs.ros.org/)
- [PX4](https://px4.io/)
- [NumPy](https://numpy.org/), [SciPy](https://scipy.org/), [pymavlink](https://github.com/ArduPilot/pymavlink), [pyzmq](https://pyzmq.readthedocs.io/)

默认安装只包含公共 Python 依赖；仿真后端通过 extras 按需安装。

| Extra | 用途 | 主要依赖 |
| --- | --- | --- |
| `.[mujoco]` | MuJoCo 后端与 README 资产渲染 | `mujoco`, `trimesh` |
| `.[genesis]` | Genesis 后端 | `genesis-world` |
| `.[all]` | 同时安装 MuJoCo 与 Genesis 后端 | `mujoco`, `genesis-world`, `trimesh` |

<p align="right">(<a href="#readme-top">返回顶部</a>)</p>

## 快速开始

### 环境要求

- Python 3.9 或更新版本。
- 如需使用 ROS 2 launch、bridge 或 PX4 联调，需要准备可用的 ROS 2 环境。
- 如需使用默认 MuJoCo 配置，安装 `.[mujoco]` extra。

### 安装

克隆仓库并安装默认 MuJoCo 后端：

```bash
git clone https://github.com/Xiangyuan-Xie/ACESim.git
cd ACESim
pip install -e ".[mujoco]"
```

如果只需要 Genesis 后端：

```bash
pip install -e ".[genesis]"
```

如果希望一次安装全部后端：

```bash
pip install -e ".[all]"
```

运行本地 Python 入口做快速验证：

```bash
python -m acesim.core.play
```

<p align="right">(<a href="#readme-top">返回顶部</a>)</p>

## 使用

### 资产画廊

#### 多旋翼与机械臂

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

#### 固定翼、VTOL 与 UUV

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

重新生成画廊图片：

```bash
python -m acesim.tools.render_readme_assets
```

### 配置

`python -m acesim.core.play` 与 ROS 2 播放入口都会读取 `acesim/config/default.toml`。当前默认值为：

```toml
[basic]
sim_type = "mujoco"
env_type = "am"
scene_name = "default"
asset_name = "x500_arm2x"
benchmark = "multirotor"
```

核心字段：

- `basic.sim_type`：仿真后端，例如 `mujoco`、`genesis`。
- `basic.env_type`：环境类型，例如 `mc`、`am`、`fw`、`vtol`、`uuv`。
- `basic.scene_name`：场景名。
- `basic.asset_name`：资产参数文件名。
- `basic.benchmark`：基准测试或运行分组字段。

ACESim 会先读取顶层 `basic` 配置，再加载 `acesim/config/<sim_type>/<asset_name>.toml` 中的资产参数。如果要切换后端、环境类型或资产，优先修改 `default.toml` 中的 `basic` 段。

### ROS 2 / PX4

ROS 2 包位于 `acesim/deploy/aircraft/acesim_ros2`，推荐在完整部署、bridge、时钟同步和联调场景下使用：

```bash
ros2 launch acesim_ros2 linux.launch.py
```

Headless launch：

```bash
ros2 launch acesim_ros2 linux_headless.launch.py
```

可执行入口：

- `acesim_play = acesim_ros2.acesim_play:main`
- `acesim_play_headless = acesim_ros2.acesim_play_headless:main`
- `acesim_bridge = acesim_ros2.acesim_bridge:main`
- `x500_arm2x_benchmark = acesim_ros2.benchmark.x500_arm2x:main`

ACESim 支持接入 PX4，但它是可选能力，不是使用门槛。PX4 相关逻辑主要由以下模块组织：

- `acesim/utils/px4_transport.py`
- `acesim/utils/px4_sensor_scheduler.py`

如未显式传入 PX4 仓库路径，ROS 2 启动逻辑默认会尝试使用：

```text
acesim/third_party/aircraft/PX4-Autopilot
```

对于 ACESim 中的 AM Position：

- 需要有效的 manual-control source，例如 QGC virtual joystick 或 RC。
- 这一要求由模式实现本身固定提供，与 Position 模式对齐，而不是由 launch 参数覆写决定。
- 摇杆回中不会阻止进入模式，而是进入保持行为。
- 如果修改了 `acesim/deploy/aircraft/px4_msgs` 下的 `.msg` / `.srv` 接口，首次需要对 ROS 2 工作区做一次 clean rebuild。

### 资产工具链

仓库内的常用工具：

- `acesim.tools.sdf2urdf`：第一阶段转换模块，从 SDF 源同步 ACESim 手工维护的 URDF 资产。
- `acesim.tools.urdf2mjcf`：第二阶段转换模块，消费 URDF，生成并后处理 MJCF。
- `acesim.tools.render_readme_assets`：生成 README 资产预览图。
- `acesim/tools/cal_dynamic_params.py`：动力学参数辅助计算。
- `acesim/tools/cal_thrust_coef.py`：推力系数辅助计算。

当前 `sdf2urdf` 的接口面向通用 SDF 来源设计，但仓库内已落地的 provider 主要是 `px4`。

最小两阶段工作流：

```bash
python -m acesim.tools.sdf2urdf --source px4 --target advanced_plane
python -m acesim.tools.urdf2mjcf --target advanced_plane
```

职责边界：

- `sdf2urdf`：读取上游 SDF 真值，生成或同步本地 meshes，并更新 ACESim 使用的 URDF。
- `urdf2mjcf`：在 URDF 已准备好的前提下，完成 MuJoCo 编译和 MJCF 后处理。

如果只是修正上游 SDF 对应的 visual / joint / inertial truth，先运行 `sdf2urdf` 即可；如果还需要刷新 MuJoCo 资产产物，再继续运行 `urdf2mjcf`。

<p align="right">(<a href="#readme-top">返回顶部</a>)</p>

## 路线图

- [x] MuJoCo 默认配置与多资产 headless 启动。
- [x] ROS 2 launch、bridge 插件和仿真时钟同步。
- [x] PX4 传感器调度与执行器控制读取。
- [x] PX4 SDF 资产导入管线。
- [x] `x500_arm2x` benchmark launch 与 console script。
- [ ] 继续完善 Genesis 后端资产覆盖与运行文档。
- [ ] 为更多任务场景沉淀 benchmark profile 与复现实验说明。

<p align="right">(<a href="#readme-top">返回顶部</a>)</p>

## 贡献

欢迎提交问题、修复和改进建议。推荐本地流程：

1. 创建分支并完成改动。
2. 运行测试：

   ```bash
   pytest
   ```

3. 运行代码质量检查：

   ```bash
   pip install pre-commit
   pre-commit install
   pre-commit run --all-files
   ```

4. 在提交说明或 PR 中写清楚改动背景、影响范围和验证方式。

当前测试覆盖的能力面包括 packaging metadata、配置加载、MuJoCo 默认配置与多资产 headless 启动、PX4 传感器调度、PX4 SDF 资产导入、ROS 2 launch 组装、bridge 运行时、视觉流 payload 编码 / 解码，以及固定翼、VTOL、UUV 动力学关键行为。

更多代码代理协作约定见 [`AGENT.md`](AGENT.md)。

<p align="right">(<a href="#readme-top">返回顶部</a>)</p>

## 许可证

ACESim 自有代码和文档采用 [Apache License 2.0](LICENSE) 发布。

第三方源码、vendored ROS 2 消息定义和外部资产保留其各自目录中的原始许可证声明，例如 `acesim/third_party/` 和 `acesim/deploy/aircraft/px4_msgs/` 下的许可证文件。

<p align="right">(<a href="#readme-top">返回顶部</a>)</p>

## 联系

维护者：Xiangyuan Xie

- Email: <dragonboat_xxy@163.com>
- Project Link: <https://github.com/Xiangyuan-Xie/ACESim>

<p align="right">(<a href="#readme-top">返回顶部</a>)</p>

## 致谢

- [othneildrew/Best-README-Template](https://github.com/othneildrew/Best-README-Template) 提供了本 README 的章节组织参考。
- MuJoCo、Genesis、PX4、ROS 2 及其生态为 ACESim 的仿真、部署和飞控联调流程提供基础能力。

<p align="right">(<a href="#readme-top">返回顶部</a>)</p>
