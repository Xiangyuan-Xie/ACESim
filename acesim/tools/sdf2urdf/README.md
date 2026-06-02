# sdf2urdf 使用指南

`acesim.tools.sdf2urdf` 是 ACESim 资产工具链的第一阶段：它从 SDF source provider 读取上游模型真值，同步仓库内手工维护的 URDF，并按需生成 source-owned mesh。

当前仓库内已落地的 source provider 是 `px4`。

## 输入与输出

工具按 `--target` 自动解析路径：

| 内容 | 路径 |
| --- | --- |
| 输入 SDF truth | 由 source provider 解析，例如 `px4` provider |
| 输入/输出 URDF | `acesim/env/mujoco/asset/<target>/<target>.urdf` |
| 输出 mesh | `acesim/env/mujoco/asset/<target>/meshes/` |

## CLI 模式

```bash
python -m acesim.tools.sdf2urdf --source px4 --target advanced_plane
```

清理 provider 生成后不再需要的临时 mesh：

```bash
python -m acesim.tools.sdf2urdf --source px4 --target advanced_plane --cleanup
```

参数：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `--source` | `px4` | SDF source provider 名称。 |
| `--target` | 必填 | 资产名称，对应 `asset/<target>/<target>.urdf`。 |
| `--cleanup` | 关闭 | 同步后删除 stale generated meshes。 |
| `--tui` | 关闭 | 启动本工具自己的交互式终端界面。 |

## TUI 模式

不带参数运行时，`sdf2urdf` 默认进入自己的 BIOS 风格 TUI：

```bash
python -m acesim.tools.sdf2urdf
```

也可以显式启动：

```bash
python -m acesim.tools.sdf2urdf --tui
```

也可以直接运行 TUI 模块：

```bash
python -m acesim.tools.sdf2urdf.tui
```

TUI 会以全屏设置界面列出 source、target 和 cleanup 开关。它只封装本阶段的 SDF -> URDF 同步，不会提供 `urdf2mjcf` 的交互入口。

| 按键 | 功能 |
| --- | --- |
| `↑` / `↓` | 移动当前选项 |
| `Enter` | 编辑文本字段、进入枚举选择页或在确认页执行 |
| `Space` | 切换开关字段 |
| `F10` | 进入确认页；在确认页执行同步 |
| `Esc` / `q` | 退出 TUI |

传入 `--source`、`--target` 等参数时，工具会保持脚本化 CLI 模式，不进入 TUI。

## 两阶段工作流

如果需要从上游 SDF 真值刷新完整 MuJoCo 资产，先运行本工具，再运行 `urdf2mjcf`：

```bash
python -m acesim.tools.sdf2urdf --source px4 --target advanced_plane
python -m acesim.tools.urdf2mjcf --target advanced_plane
```

两个工具也可以分别使用各自的 TUI：

```bash
python -m acesim.tools.sdf2urdf --tui
python -m acesim.tools.urdf2mjcf --tui
```
