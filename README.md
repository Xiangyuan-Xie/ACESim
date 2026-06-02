<a id="readme-top"></a>

<div align="center">

<p align="center">
  <a href="https://www.python.org/"><img src="https://img.shields.io/badge/python-3.9%2B-blue.svg" alt="Python" /></a>
  <a href="https://github.com/psf/black"><img src="https://img.shields.io/badge/code%20style-black-000000.svg" alt="Code style: black" /></a>
  <a href="https://pre-commit.com/"><img src="https://img.shields.io/badge/pre--commit-enabled-brightgreen?logo=pre-commit&logoColor=white" alt="pre-commit" /></a>
</p>

<h1 align="center">ACESim</h1>

<p align="center">
  让策略跨越仿真边界。
  <br />
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
    <li><a href="#贡献">贡献</a></li>
    <li><a href="#许可证">许可证</a></li>
    <li><a href="#联系">联系</a></li>
    <li><a href="#致谢">致谢</a></li>
  </ol>
</details>

## 项目简介

ACESim 是一个面向多域载具跨仿真评估的仿真平台，统一管理环境装配、仿真运行、资产转换与部署流程。它通过一致的配置和运行接口连接 MuJoCo、Genesis 等仿真后端，支持在不同仿真器中复用载具资产、任务场景和控制策略，从而评估策略在跨仿真环境下的稳定性、可迁移性与泛化能力。

<p align="right">(<a href="#readme-top">返回顶部</a>)</p>

## 技术栈

- [Python](https://www.python.org/)
- [MuJoCo](https://mujoco.org/)
- [Genesis](https://genesis-world.readthedocs.io/)
- [ROS2](https://docs.ros.org/)
- [PX4 Autopliot](https://px4.io/)

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
- ROS2 Humble

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

| 字段 | 说明 | 可选值或示例 |
| --- | --- | --- |
| `basic.sim_type` | 仿真后端 | `mujoco`、`genesis` |
| `basic.env_type` | 环境类型 | `mc`、`am`、`fw`、`vtol`、`uuv` |
| `basic.scene_name` | 场景名 | `default` |
| `basic.asset_name` | 资产参数文件名 | `x500_arm2x` |
| `basic.benchmark` | 基准测试或运行分组字段 | `multirotor` |

ACESim 会先读取顶层 `basic` 配置，再加载 `acesim/config/<sim_type>/<asset_name>.toml` 中的资产参数。如果要切换后端、环境类型或资产，优先修改 `default.toml` 中的 `basic` 段。

### ROS 2 / PX4

ROS 2 包位于 `acesim/deploy/aircraft/acesim_ros2`，主要用于完整部署、飞控联调。

#### Windows + WSL

```bash
ros2 launch acesim_ros2 wsl.launch.py
```

该方式适用于在 Windows 侧运行 ACESim 前端，并在 WSL 侧运行 PX4、Micro XRCE-DDS Agent 与 ROS 2 bridge 的联调场景。

如果 PX4 仓库不在默认位置，可以显式传入：

```bash
ros2 launch acesim_ros2 wsl.launch.py px4_repo:=/path/to/PX4-Autopilot
```

#### Linux

在 Linux 环境下，可直接启动包含 ACESim、PX4、Micro XRCE-DDS Agent 与 ROS 2 bridge 的完整联调链路：

```bash
ros2 launch acesim_ros2 linux.launch.py
```

### 资产工具链

资产工具链用于把上游 SDF、手工维护的 URDF、mesh 和 MuJoCo MJCF 产物保持一致。日常修改资产时，推荐先明确自己改的是哪一层。两个转换工具不带参数运行时会默认进入各自的 BIOS 风格 TUI：

| 工具 | 职责 | 默认交互入口 |
| --- | --- | --- |
| `acesim.tools.sdf2urdf` | 从 SDF source provider 同步 URDF 和 source-owned mesh | `python -m acesim.tools.sdf2urdf` |
| `acesim.tools.urdf2mjcf` | 从 URDF 生成并后处理 MuJoCo MJCF | `python -m acesim.tools.urdf2mjcf` |
| `acesim.tools.render_readme_assets` | 从 MJCF 渲染 README 资产预览图 | `python -m acesim.tools.render_readme_assets` |

<p align="right">(<a href="#readme-top">返回顶部</a>)</p>

## 贡献

欢迎提交 Issue、功能建议和 Pull Request。为便于维护和代码审查，建议每次贡献围绕一个边界清晰的改动展开，例如仿真后端、环境配置、资产转换、动力学模型、测试修复或文档更新等。请避免在同一个分支或 PR 中混合多个无关改动。

### 分支与提交

建议从最新的主分支创建功能分支：

```bash
git checkout main
git pull
git checkout -b feat/your-feature-name
```

提交信息建议使用简洁明确的格式，例如：

```text
feat(mujoco): add new vehicle asset
feat(genesis): support multi-asset headless launch
fix(px4): correct sensor scheduling config
docs: update sim2sim evaluation guide
test: add VTOL dynamics regression tests
```

### 本地检查

提交 PR 前，建议至少运行以下测试：

```bash
pytest
```

并执行代码质量检查：

```bash
pip install pre-commit
pre-commit install
pre-commit run --all-files
```

如果改动涉及 ROS 2 launch、PX4 适配、bridge 插件或仿真运行入口，建议补充对应的最小启动验证，并在 PR 中说明运行命令和测试环境。

### 子模块修改

如果需要修改 `acesim/deploy/aircraft/px4_msgs/`、`acesim/third_party/aircraft/PX4-Autopilot/`、`acesim/third_party/aircraft/Micro-XRCE-DDS-Agent/` 或其他子模块，请先在对应子模块内完成修改并提交：

```bash
cd acesim/deploy/aircraft/px4_msgs
git checkout -b feat/your-change
git add .
git commit -m "feat: your change"
```

随后回到 ACESim 父仓库，更新并提交对应的 gitlink：

```bash
cd ../../../..
git add acesim/deploy/aircraft/px4_msgs
git commit -m "chore: update px4_msgs submodule"
```

请不要只在父仓库中提交子模块目录的未提交工作区状态，否则其他用户无法复现该修改。

### Pull Request

提交 PR 时，请简要说明：

* 本次改动的目的和背景；
* 修改涉及的主要文件、模块或仿真后端；
* 已运行的测试、启动或验证命令；
* 是否依赖 PX4、ROS 2、外部插件或特定仿真器版本；
* 是否涉及资产文件、配置格式、接口行为或数据格式变化。

这样可以帮助维护者更快地理解、复现和合并你的贡献。

<p align="right">(<a href="#readme-top">返回顶部</a>)</p>

## 许可证

本项目采用 Apache 2.0 开源许可证，详情请参见 [LICENSE](LICENSE)。项目中引用的子模块及第三方组件遵循其各自仓库声明的许可证。

<p align="right">(<a href="#readme-top">返回顶部</a>)</p>

## 联系

项目维护者：Xiangyuan Xie

项目链接: <https://github.com/Xiangyuan-Xie/ACESim>

<p align="right">(<a href="#readme-top">返回顶部</a>)</p>

## 致谢

- [MuJoCo](https://mujoco.org/)
- [Genesis](https://genesis-world.readthedocs.io/)
- [ROS2 Humble](https://docs.ros.org/)
- [PX4 Autopliot](https://px4.io/)

<p align="right">(<a href="#readme-top">返回顶部</a>)</p>
