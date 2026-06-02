# urdf2mjcf 使用指南

`acesim.tools.urdf2mjcf` 是 ACESim 资产工具链的第二阶段：它读取仓库内已经准备好的 URDF 资产，生成 MuJoCo 可直接加载的 MJCF，并在编译后补齐运行时需要的传感器、执行器、keyframe、碰撞排除和不同资产族的运行时结构。

## 适用场景

- 已经有 `acesim/env/mujoco/asset/<target>/<target>.urdf`，需要刷新对应的 `<target>.xml`。
- 修改了 URDF、mesh、joint、inertial 或 collision，需要重新生成 MuJoCo 资产。
- 需要为 collision mesh 生成凸包分解产物，提升 MuJoCo 碰撞稳定性。
- 需要为浮动载具生成带 `floating_base_joint` 和 `home` keyframe 的 MJCF。

如果资产来源是 PX4 SDF，通常先运行第一阶段 `sdf2urdf`，再运行本工具。

```bash
python -m acesim.tools.sdf2urdf --source px4 --target advanced_plane
python -m acesim.tools.urdf2mjcf --target advanced_plane
```

## 输入与输出

工具按 `--target` 自动解析路径：

| 内容 | 路径 |
| --- | --- |
| 输入 URDF | `acesim/env/mujoco/asset/<target>/<target>.urdf` |
| 输入 mesh | `acesim/env/mujoco/asset/<target>/meshes/` |
| 输出 MJCF | `acesim/env/mujoco/asset/<target>/<target>.xml` |
| 凸包分解 mesh | `acesim/env/mujoco/asset/<target>/meshes/*_decomp_*.stl` |

当输出 XML 已存在时，命令会询问是否覆盖。输入 `y` 后会删除旧 XML 和旧的 `*_decomp_*.stl`，再重新生成。

## 依赖

推荐先安装 MuJoCo extra：

```bash
pip install -e ".[mujoco]"
```

工具还依赖：

- `pinocchio`：计算初始姿态下的自动离地高度。
- `trimesh`：读取和处理 mesh。
- `mujoco` 或 MuJoCo `compile` 二进制：把 URDF 编译成 MJCF。
- `coacd`：仅在使用 `--decompose` 时需要，用于凸包分解。

如果系统中存在 MuJoCo `compile`，工具会优先使用它；否则会回退到 Python `mujoco` 包进行编译。显式传入 `--mujoco-bin` 时，路径必须存在。

## 命令参数

```bash
python -m acesim.tools.urdf2mjcf --target <target> [options]
```

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `--target` | `ace_leader` | 资产名称，对应 `asset/<target>/<target>.urdf`。 |
| `--floating` | 关闭 | 为根 body 添加自由关节，适用于飞行器、UUV 等自由运动载具。 |
| `--decompose` | 关闭 | 对 URDF collision mesh 做 CoACD 凸包分解。 |
| `--safety-margin` | `0.05` | 自动离地高度的额外安全裕量，单位为米。 |
| `--q0` | 空 | 初始关节零位，格式为 `joint=value,joint=value`。用于计算自动高度和写入 `home` keyframe。 |
| `--mujoco-bin` | 空 | 指定 MuJoCo `compile` 二进制路径。 |
| `--tui` | 关闭 | 启动本工具自己的交互式终端界面。 |

## TUI 模式

不带参数运行时，`urdf2mjcf` 默认进入自己的 BIOS 风格 TUI：

```bash
python -m acesim.tools.urdf2mjcf
```

也可以显式启动：

```bash
python -m acesim.tools.urdf2mjcf --tui
```

也可以直接运行 TUI 模块：

```bash
python -m acesim.tools.urdf2mjcf.tui
```

TUI 会以全屏设置界面列出 target、是否添加 floating root、是否运行凸包分解、`safety-margin`、`q0`、MuJoCo `compile` 路径和是否覆盖已有 XML。`q0` 是二级页面：进入时会按当前 target 读取 URDF，把可配置的初始关节零位逐项列出；执行前会重新拼接为 CLI 使用的 `joint=value,joint=value` 格式。它只封装本阶段的 URDF -> MJCF 转换，不会提供 `sdf2urdf` 的交互入口。

| 按键 | 功能 |
| --- | --- |
| `↑` / `↓` | 移动当前选项 |
| `Enter` | 编辑文本/数字字段、进入枚举选择页或在确认页执行 |
| `Space` | 切换开关字段 |
| `F10` | 进入确认页；在确认页执行转换 |
| `Esc` / `q` | 退出 TUI |

传入 `--target` 等参数时，工具会保持脚本化 CLI 模式，不进入 TUI。

## 常用命令

生成普通固定资产：

```bash
python -m acesim.tools.urdf2mjcf --target advanced_plane
```

生成浮动根资产：

```bash
python -m acesim.tools.urdf2mjcf --target x500 --floating
```

生成带凸包分解的浮动根资产：

```bash
python -m acesim.tools.urdf2mjcf --target x500_arm2x --floating --decompose
```

为 `x500_arm2x` 指定机械臂零位并做凸包分解：

```bash
python -m acesim.tools.urdf2mjcf \
  --target x500_arm2x \
  --floating \
  --decompose \
  --q0 'joint_1=-1.5708,joint_2=3.1416,joint_3=0.0,joint_4=0.0'
```

非交互覆盖已有 XML：

```bash
printf 'y\n' | python -m acesim.tools.urdf2mjcf --target x500_arm2x --floating --decompose
```

## 处理流程

一次转换大致包含这些步骤：

1. 检查目标 URDF 是否存在。
2. 如目标 mesh 目录存在，同步 SDF 阶段管理的手工 mesh 和 URDF 片段。
3. 执行资产族运行时准备逻辑，例如多旋翼、固定翼、VTOL、UUV 的专用 mesh 或结构调整。
4. 如启用 `--decompose`，对 collision mesh 运行 CoACD，并临时替换 URDF collision 节点。
5. 使用 `q0` 计算当前姿态下的最低点，并应用 `--safety-margin` 得到根节点高度。
6. 预处理 URDF：规范 mesh 路径，必要时插入 `floating_base_joint` 和 MuJoCo compiler 配置。
7. 使用 MuJoCo 编译 URDF 到 MJCF。
8. 后处理 MJCF：补齐传感器、执行器、`home` keyframe、碰撞排除和资产族运行时结构。
9. 清理临时文件和仅转换期间需要的 mesh。

## 资产族处理

工具会按 target 选择运行时 handler：

| Target | 资产族 |
| --- | --- |
| `iris`、`x500`、`typhoon_h480` | `multirotor` |
| `advanced_plane` | `fixedwing` |
| `standard_vtol` | `vtol` |
| `uuv_bluerov2_heavy` | `uuv` |
| 其他 target | `generic` |

`x500_arm2x` 当前走 `generic` handler，但后处理阶段仍会保留机械臂 actuator、传感器和 `home` keyframe。

## 转换后验证

推荐至少做一次 MuJoCo 加载检查：

```bash
python - <<'PY'
from pathlib import Path
import mujoco

target = "x500_arm2x"
xml_path = Path("acesim/env/mujoco/asset") / target / f"{target}.xml"
model = mujoco.MjModel.from_xml_path(str(xml_path))
print("loaded", xml_path)
print("nbody", model.nbody, "ngeom", model.ngeom, "nmesh", model.nmesh, "nkey", model.nkey)
PY
```

检查 `home` keyframe：

```bash
python - <<'PY'
import xml.etree.ElementTree as ET
from pathlib import Path

target = "x500_arm2x"
xml_path = Path("acesim/env/mujoco/asset") / target / f"{target}.xml"
root = ET.parse(xml_path).getroot()
home = root.find("./keyframe/key[@name='home']")
print(home.get("qpos") if home is not None else "missing home keyframe")
PY
```

刷新 README 资产图：

```bash
python -m acesim.tools.render_readme_assets --assets x500_arm2x
```

运行相关测试：

```bash
python -m pytest tests/test_urdf2mjcf_compiler.py tests/test_render_readme_assets.py -q
```

## 常见问题

### `pinocchio is required to calculate the robot auto-height`

需要安装 Pinocchio。工具会使用它按 `q0` 计算 collision mesh 的最低点，从而写入合理的根节点高度。

### `coacd is required when --decompose is enabled`

启用了 `--decompose` 但未安装 CoACD。安装 `coacd` 后重试，或去掉 `--decompose`。

### MuJoCo 找不到 mesh

检查 URDF 中 mesh 路径和 `meshes/` 目录是否一致。工具会把 `package://.../meshes/` 或其他前缀规范到本地 `meshes/` 路径，但源文件仍必须存在于目标资产目录下。

### STL 面数超过 MuJoCo 限制

Python MuJoCo 的 STL 读取器对单个 STL 面数有限制。先简化源 mesh，再重新同步到 `acesim/env/mujoco/asset/<target>/meshes/` 并重新转换。

### `free joint can only be used on top level`

浮动根资产应使用 `--floating`，并让 Python MuJoCo fallback 保持默认 static fusion。不要在源 URDF 中手工加入额外固定父节点和 free joint，除非确认 MuJoCo 可以把根结构正确融合。

### 输出 XML 没更新

如果 XML 已存在但没有输入 `y`，工具会中止。需要覆盖时使用交互确认，或使用：

```bash
printf 'y\n' | python -m acesim.tools.urdf2mjcf --target <target> [options]
```
