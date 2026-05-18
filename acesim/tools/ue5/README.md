# UE5 Integration Tools

这组脚本把 ACESim 的 UE5 联调拆成三步：

1. `setup_ubuntu_ue5.sh`
   - 在 Ubuntu 上安装 UE5 源码编译依赖
   - 检查 `nvidia-smi`
   - 拉取 Epic 官方 UE5 源码
   - 执行 `Setup.sh`、`GenerateProjectFiles.sh`、`make UnrealEditor`
   - 生成并编译 `/tmp/ACESim-unreal/projects/ACESimUE` 与 `ACESimBridge` 插件
2. `check_ubuntu_ue5_host.sh`
   - 在不改动系统的前提下检查 `sudo`、`nvidia-smi`、UE 仓库访问和工具链状态
   - 输出诊断日志到 `/tmp/ACESim-unreal/logs/host_check.txt`
3. `create_project_scaffold.py`
   - 在 `/tmp/ACESim-unreal/projects/ACESimUE` 生成最小 C++ UE5 项目
   - 项目内自带 `ACESimBridge` 插件
   - 插件包含 `AACESimVehicleActor` 与 `UACESimVehicleSyncComponent`
4. `verify_visual_stream.py`
   - 直接订阅 `tcp://127.0.0.1:5601`
   - 支持 `--timeout-sec`，避免没有发布端时无限等待
   - 校验 ACESim 发布的视觉状态 payload
5. `package_ue_runtime.sh`
   - 预检 UE source、project、DDC、默认 map、插件产物和磁盘空间
   - 构建 `ShaderCompileWorker` 与 `ACESimUEEditor`
   - 执行 `RunUAT BuildCookRun`，输出 `/tmp/ACESim-unreal/packages/ACESimUE-Linux`
   - 自动定位实际 `ACESimUE` executable，供 ROS2 launch 使用

## 默认目录

- UE5 源码：`/tmp/ACESim-unreal/UnrealEngine`
- UE5 工程：`/tmp/ACESim-unreal/projects/ACESimUE`
- UE5 runtime package：`/tmp/ACESim-unreal/packages/ACESimUE-Linux`
- 日志目录：`/tmp/ACESim-unreal/logs`

## Architecture

ACESim/MuJoCo 仍然是动力学权威，UE5 只负责场景渲染。Phase 1 的桥接链路是单向的：

```text
ACESim/MuJoCo -> ZeroMQ vehicle visual state -> ACESimBridge -> UE actors
```

`ACESimBridge` 插件订阅 `tcp://127.0.0.1:5601`，把 MuJoCo 发布的 NWU/FLU 位姿转换到 UE 坐标系，并在 UE tick 中驱动 actor 与旋翼组件。Phase 1 不把 UE 相机、深度、分割或碰撞事件反馈给 ACESim。

## Run Order

```bash
bash /home/xxy/ACESim/acesim/tools/ue5/check_ubuntu_ue5_host.sh

bash /home/xxy/ACESim/acesim/tools/ue5/setup_ubuntu_ue5.sh

python3 -m acesim.deploy.aircraft.acesim_ros2.acesim_ros2.acesim_play_headless

python3 /home/xxy/ACESim/acesim/tools/ue5/verify_visual_stream.py --endpoint tcp://127.0.0.1:5601 --samples 5 --timeout-sec 10
```

推荐顺序：

1. 先用 `check_ubuntu_ue5_host.sh` 检查主机状态。
2. 构建或复用 UE：`setup_ubuntu_ue5.sh` 会生成并编译 `/tmp/ACESim-unreal/projects/ACESimUE`。
3. 启动 ACESim headless，选择视觉流 endpoint 为 `tcp://0.0.0.0:5601` 的 MuJoCo 配置。
4. 用 `verify_visual_stream.py --endpoint tcp://127.0.0.1:5601 --samples 5 --timeout-sec 10` 解码流并确认 payload 正常。
5. 打开 `/tmp/ACESim-unreal/projects/ACESimUE/ACESimUE.uproject`。
6. 点击 Play；默认 `ACESimUEGameMode` 会在场景里自动生成一个带 `ACESimVehicleSyncComponent` 的 `ACESimVehicleActor`。
7. 如果你手动放置了自己的 `ACESimVehicleActor`，默认 GameMode 会复用它，不会再生成第二个。

如果只需要重新编译已生成的 UE 工程和插件，可以直接运行：

```bash
/tmp/ACESim-unreal/UnrealEngine/Engine/Build/BatchFiles/Linux/Build.sh \
  ACESimUEEditor Linux Development \
  -Project=/tmp/ACESim-unreal/projects/ACESimUE/ACESimUE.uproject \
  -Progress \
  -NoHotReloadFromIDE \
  -NoUBA
```

`-NoUBA` 会禁用 Unreal Build Accelerator。本机验证中 UBA local executor 曾在项目级小编译上卡住，而普通 Parallel executor 可以稳定产出：

- `/tmp/ACESim-unreal/projects/ACESimUE/Binaries/Linux/libUnrealEditor-ACESimUE.so`
- `/tmp/ACESim-unreal/projects/ACESimUE/Plugins/ACESimBridge/Binaries/Linux/libUnrealEditor-ACESimBridge.so`

生成工程默认使用轻量 `/Engine/Maps/Templates/Template_Default`。旧版生成器曾默认使用 `/Engine/Maps/Templates/OpenWorld`，第一次启动会触发 World Partition 和大量 Vulkan shader 编译；如果日志停在 `Using 16 local workers for shader compilation` 或多次出现 shader job 超过几十秒，通常是在编译 shader，不是 `HotfixForNextBoot.txt` 缺失导致的错误。

如果启动 editor 或 `linux_ue.launch.py` 时弹出 `Unable to launch ShaderCompileWorker`，说明 UE 源码目录里还缺少着色器编译子程序。单独补构建即可：

```bash
/tmp/ACESim-unreal/UnrealEngine/Engine/Build/BatchFiles/Linux/Build.sh \
  ShaderCompileWorker Linux Development \
  -Progress \
  -NoHotReloadFromIDE \
  -NoUBA
```

通过标准：

```bash
test -x /tmp/ACESim-unreal/UnrealEngine/Engine/Binaries/Linux/ShaderCompileWorker
```

如果现有 `/tmp/ACESim-unreal/projects/ACESimUE/Config/DefaultEngine.ini` 仍指向 OpenWorld，可以改成：

```ini
[/Script/EngineSettings.GameMapsSettings]
EditorStartupMap=/Engine/Maps/Templates/Template_Default
GameDefaultMap=/Engine/Maps/Templates/Template_Default
GlobalDefaultGameMode=/Script/ACESimUE.ACESimUEGameMode
```

## Manual Viewport Acceptance

这一步用于确认真实 UE viewport 里能看到由 MuJoCo 状态驱动的 actor。前面的 smoke gate 已经覆盖编译、插件加载和 live bridge 日志；这里补的是肉眼确认渲染结果。

这一步只需要在以下场景执行：

- 第一次完成 UE bridge 集成后。
- 改了 actor、mesh、材质、相机、GameMode、坐标转换或 rotor 可视化。
- 准备对外演示或发布前，需要确认画面质量。

日常代码验证、CI 或 headless 服务器验证不需要打开编辑器；使用 `smoke_ue_bridge.sh` 和 `verify_visual_stream.py` 即可。

### 1. 预检

先确认无界面 smoke 能通过：

```bash
cd /home/xxy/ACESim
bash acesim/tools/ue5/smoke_ue_bridge.sh
```

预期输出包含：

- `Result: Succeeded`
- `Success - 0 error(s), 0 warning(s)`

### 2. 启动 ACESim 发布端

开一个终端，保持它运行：

```bash
cd /home/xxy/ACESim
python3 -m acesim.deploy.aircraft.acesim_ros2.acesim_ros2.acesim_play_headless
```

另开一个终端确认视觉流正在发布：

```bash
cd /home/xxy/ACESim
python3 acesim/tools/ue5/verify_visual_stream.py \
  --endpoint tcp://127.0.0.1:5601 \
  --samples 5 \
  --timeout-sec 10
```

通过标准：

- 打印 5 个 sample。
- timestamp 递增。
- 默认机型应看到 `rotors=4`。

### 3. 打开 UE 图形界面

```bash
/tmp/ACESim-unreal/UnrealEngine/Engine/Binaries/Linux/UnrealEditor \
  /tmp/ACESim-unreal/projects/ACESimUE/ACESimUE.uproject
```

打开后点击 Play。默认 `ACESimUEGameMode` 会自动生成 `ACESimVehicleActor`，并附带 `ACESimVehicleSyncComponent`。如果当前 map 覆盖了 GameMode，改回 `ACESimUEGameMode`，或者手动放置一个 `ACESimVehicleActor`。

如果视口里没立刻看到 vehicle：

- 在 World Outliner 里选中 `ACESimVehicle` 或手动放置的 `ACESimVehicleActor`。
- 在 viewport 按 `F` 聚焦选中 actor。
- 在 Details 面板确认 `ACESimVehicleSyncComponent` 的 endpoint 是 `tcp://127.0.0.1:5601`，并且 `Auto Connect On Begin Play` 处于启用状态。

### 4. 观察与判定

通过标准：

- Play 后 viewport 中出现简化 vehicle body 和 rotor 组件。
- Output Log 或 `/tmp/ACESim-unreal/projects/ACESimUE/Saved/Logs/ACESimUE.log` 包含 `ACESim visual stream connected`。
- Output Log 或日志包含 `ACESim visual state applied`。
- actor 的 Location/Rotation 随 MuJoCo 状态更新。如果默认场景运动不明显，至少确认 Details 里的 timestamp 日志推进，且 rotor 组件持续按视觉角度变化。
- 默认四旋翼流应显示 4 个 rotor 组件参与动画。

失败判定：

- `verify_visual_stream.py` 超时或没有 sample。
- UE 日志没有 `ACESim visual stream connected`。
- UE 有连接日志但没有 `ACESim visual state applied`。
- viewport 中没有 vehicle actor，或者 actor 存在但 endpoint/自动连接设置不正确。
- 默认四旋翼流显示 `rotors=0` 或 rotor 组件完全不动。

### 5. 传感器反馈占位检查

Phase 1 不发布 UE camera/depth/segmentation/event payload。可选验收：

1. 给任意 actor 添加 `ACESimSensorFeedbackComponent`。
2. 保持 `Enable Sensor Feedback` 关闭，Play 后不应出现反馈发布行为。
3. 如果临时打开 `Enable Sensor Feedback`，日志只应出现 phase 1 不发布 sensor payload 的 warning，不应打开 `5610` 到 `5613` 的反馈通道。

### 6. 清理

结束 Play，关闭 UE，然后停止 ACESim headless 终端。需要确认后台进程时可运行：

```bash
pgrep -af 'acesim_play_headless|UnrealEditor|UnrealTraceServer|verify_visual_stream|zenserver'
```

没有输出表示本次验收相关进程已退出。

### 7. 验收记录模板

```text
Date:
ACESim command/config:
verify_visual_stream result:
UE command:
Observed actor motion:
Observed rotor animation:
UE log markers:
Sensor feedback placeholder check:
Result: PASS/FAIL
Notes or screenshot path:
```

## Packaging

发布不依赖手动打开编辑器；推荐使用命令行先验证、再 package。当前工程默认使用 `/Engine/Maps/Templates/Template_Default`，并把 `GlobalDefaultGameMode` 设置为 `ACESimUEGameMode`，所以打包后运行时也会自动生成带 bridge sync 的 `ACESimVehicleActor`。

### 1. 发布前检查

```bash
cd /home/xxy/ACESim
bash acesim/tools/ue5/smoke_ue_bridge.sh
```

如果要确认真实 ACESim 流也能驱动 UE：

```bash
cd /home/xxy/ACESim
python3 -m acesim.deploy.aircraft.acesim_ros2.acesim_ros2.acesim_play_headless
```

另一个终端运行：

```bash
cd /home/xxy/ACESim
RUN_LIVE_BRIDGE_SMOKE=1 bash acesim/tools/ue5/smoke_ue_bridge.sh
```

### 2. 命令行打包 Linux 版本

```bash
cd /home/xxy/ACESim
bash acesim/tools/ue5/package_ue_runtime.sh
```

产物目录：

```text
/tmp/ACESim-unreal/packages/ACESimUE-Linux
```

脚本会完成这些步骤：

- 检查 `UnrealEditor`、`UnrealEditor-Cmd`、项目 `.uproject`、DDC 可写状态、磁盘空间和旧 `OpenWorld` map 残留。
- 构建 `ShaderCompileWorker Linux Development`。
- 构建 `ACESimUEEditor Linux Development`。
- 执行 `RunUAT BuildCookRun` 到 `/tmp/ACESim-unreal/packages/ACESimUE-Linux`。
- 打印实际找到的 `ACESimUE executable` 路径。

打包产物本质上是一个可独立运行的 Linux UE 应用目录，通常包含：

- 一个 `ACESimUE` 可执行文件。
- Cooked content、`.pak` 文件和运行时依赖库。
- UE 项目和 `ACESimBridge` 插件编译后的 runtime 代码。

它不是 ROS2 package，也不会把 MuJoCo/PX4/ACESim Python 环境一起打进去。它只负责订阅视觉状态并渲染。

如果要做更接近交付的包，把 `-clientconfig=Development` 改为 `-clientconfig=Shipping`。Shipping 可能裁剪部分日志；第一次发布建议先用 Development 包确认 bridge 日志和运行行为。

### 3. 运行打包产物

先启动 ACESim 发布端：

```bash
cd /home/xxy/ACESim
python3 -m acesim.deploy.aircraft.acesim_ros2.acesim_ros2.acesim_play_headless
```

再运行打包后的 UE 程序。实际可执行文件名以 package 目录为准，通常在归档目录的 Linux 子目录中：

```bash
find /tmp/ACESim-unreal/packages/ACESimUE-Linux -maxdepth 3 -type f -executable | sort
```

找到 `ACESimUE` 可执行文件后启动：

```bash
/tmp/ACESim-unreal/packages/ACESimUE-Linux/Linux/ACESimUE/Binaries/Linux/ACESimUE
```

通过标准：

- 窗口正常打开。
- ACESim headless 正在发布时，画面中出现 vehicle。
- Development 包日志中能看到 `ACESim visual stream connected` 和 `ACESim visual state applied`。
- actor 与 rotor 动画表现和编辑器 Play 模式一致。

### 4. 发布内容边界

当前 phase 1 package 是 UE 渲染前端，不包含 ACESim/MuJoCo/PX4 运行时。发布或演示时需要同时部署：

- 打包后的 UE 程序。
- ACESim Python 环境和 MuJoCo/PX4 依赖。
- 能发布 `tcp://0.0.0.0:5601` 视觉流的 ACESim 配置。

如果 UE 和 ACESim 不在同一台机器上，需要把 `ACESimVehicleSyncComponent` 的 endpoint 从 `tcp://127.0.0.1:5601` 改成 ACESim 主机 IP，并确保网络允许 TCP 5601。

## ROS2 Launch With UE Rendering

可以通过现有 ROS2 包启动 UE 渲染链路，但推荐把 UE 当作 ROS2 launch 托管的外部进程，而不是做成 ROS2 node：

```text
ros2 launch
  -> MicroXRCEAgent
  -> PX4 SITL
  -> acesim_bridge
  -> acesim_play_ue           # starts packaged UE, then steps MuJoCo headless
```

关键点：

- 使用 `linux_ue.launch.py` 对应的 `acesim_play_ue`，不要用 `linux.launch.py` 的 `acesim_play`，因为后者会调用 MuJoCo viewer。
- `acesim_play_ue` 会先启动打包后的 UE executable，再进入和 `acesim_play_headless` 一样的 MuJoCo `env.step()` 循环。
- MuJoCo 仍然在 ACESim 里跑动力学和 PX4/控制闭环。
- UE5 只订阅 ZeroMQ visual stream 并渲染，不接管物理仿真。
- 默认视觉流配置已经是 `tcp://0.0.0.0:5601`，UE 默认订阅 `tcp://127.0.0.1:5601`。

一条命令启动 PX4、ACESim headless 和 UE 渲染：

```bash
ros2 launch acesim_ros2 linux_ue.launch.py
```

默认 `ue_mode:=package`、`ue_executable:=auto`。这会只启动已打包 runtime：

```text
/tmp/ACESim-unreal/packages/ACESimUE-Linux/Linux/ACESimUE/Binaries/Linux/ACESimUE
```

如果这个 executable 不存在，启动会直接失败并提示运行：

```bash
bash acesim/tools/ue5/package_ue_runtime.sh
```

这条默认路径不会隐式 fallback 到 Editor，避免用户等在 `Turnkey`、DDC、Vulkan shader、`EditorResources` 或 `VREditor` 构建过程中。

如果你的打包产物在别处，可以显式指定：

```bash
ros2 launch acesim_ros2 linux_ue.launch.py ue_executable:=/path/to/ACESimUE
```

Editor 开发模式仍然保留，但必须显式打开：

```bash
ros2 launch acesim_ros2 linux_ue.launch.py ue_mode:=editor
```

也可以覆盖 Editor 或 project 路径：

```bash
ros2 launch acesim_ros2 linux_ue.launch.py \
  ue_mode:=editor \
  unreal_editor:=/tmp/ACESim-unreal/UnrealEngine/Engine/Binaries/Linux/UnrealEditor \
  ue_project:=/tmp/ACESim-unreal/projects/ACESimUE/ACESimUE.uproject
```

Editor 模式会启动 `UnrealEditor <uproject> -game`，首次运行可能触发 shader/DDC 编译；它用于开发验收，不是 ROS2 日常启动默认路径。

也可以直接运行新的 console script：

```bash
ACESIM_UE_EXECUTABLE=/tmp/ACESim-unreal/packages/ACESimUE-Linux/Linux/ACESimUE/Binaries/Linux/ACESimUE \
  ros2 run acesim_ros2 acesim_play_ue
```

`acesim_play_ue` 支持 `--ue-executable` 覆盖路径，并可重复传 `--ue-arg` 给 UE，例如：

```bash
ros2 run acesim_ros2 acesim_play_ue \
  --ue-executable /tmp/ACESim-unreal/packages/ACESimUE-Linux/Linux/ACESimUE/Binaries/Linux/ACESimUE \
  --ue-arg -windowed
```

直接运行 Editor 开发模式：

```bash
ros2 run acesim_ros2 acesim_play_ue --ue-mode editor --ue-arg -windowed
```

退出时，`acesim_play_ue` 会关闭 ACESim env，并尝试终止 UE 子进程。

### 更新 ROS2 install space

`ros2 launch` 运行的是 `/home/xxy/ws_acesim/install/acesim_ros2/lib/acesim_ros2/acesim_play_ue` 里的已安装 console script。修改 `acesim_play_ue.py`、`setup.py` 或 launch 文件后，需要重新构建并重新 source install space：

```bash
cd /home/xxy/ws_acesim
PYTHONNOUSERSITE=1 colcon build --packages-select acesim_ros2
source /home/xxy/ws_acesim/install/setup.zsh
```

这里使用 `PYTHONNOUSERSITE=1` 是为了避免用户目录里的新版 `setuptools` 影响 ROS2 Python 包的 legacy install/develop 流程。如果普通 `colcon build` 报 `option --editable not recognized` 或 `option --uninstall not recognized`，用上面的命令重建。

重建后可以先做一个不启动仿真的 wrapper 参数检查：

```bash
/home/xxy/ws_acesim/install/acesim_ros2/lib/acesim_ros2/acesim_play_ue --help --ros-args
```

通过标准：命令打印 `acesim_play_ue` 的 help，不再出现 `unrecognized arguments: --ros-args`。

## 启动卡住排查表

| 症状 | 常见原因 | 检查命令 | 处理方式 |
| --- | --- | --- | --- |
| 日志只有 `HotfixForNextBoot.txt` 缺失 | 这是 UE 启动时的正常配置噪声，不是阻塞原因 | 查看后续 `ACESimUE.log` 是否继续进入 shader/DDC/EditorResources/VREditor | 继续看后续日志；不要把这条当作失败根因 |
| `UnrealEditor -game` 长时间无响应 | 显式 Editor 模式正在做 Turnkey、DDC、Vulkan shader 或 EditorResources 构建 | `tail -f /tmp/ACESim-unreal/projects/ACESimUE/Saved/Logs/ACESimUE.log` | 日常 ROS2 启动改用 packaged runtime：`bash acesim/tools/ue5/package_ue_runtime.sh` 后运行 `ros2 launch acesim_ros2 linux_ue.launch.py` |
| 默认 `linux_ue.launch.py` 立即报缺 executable | `ue_mode:=package` 默认只接受已打包 runtime | `test -x /tmp/ACESim-unreal/packages/ACESimUE-Linux/Linux/ACESimUE/Binaries/Linux/ACESimUE` | 运行 `bash acesim/tools/ue5/package_ue_runtime.sh`，或传 `ue_executable:=/path/to/ACESimUE` |
| 弹窗或日志出现 `Unable to launch ShaderCompileWorker` | UE source build 缺少 shader 编译子程序 | `test -x /tmp/ACESim-unreal/UnrealEngine/Engine/Binaries/Linux/ShaderCompileWorker` | 运行 `bash acesim/tools/ue5/package_ue_runtime.sh`，或单独执行 `Build.sh ShaderCompileWorker Linux Development -Progress -NoHotReloadFromIDE -NoUBA` |
| 启动仍进入 OpenWorld 或第一次 shader 编译非常重 | 旧工程 `DefaultEngine.ini` 仍指向 `/Engine/Maps/Templates/OpenWorld` | `grep -R OpenWorld /tmp/ACESim-unreal/projects/ACESimUE/Config` | 重新运行 `create_project_scaffold.py --overwrite`，或手动改为 `/Engine/Maps/Templates/Template_Default` |
| 修了源码但 `ros2 launch` 仍报旧错误 | ROS2 install space 还在使用旧 console script | `/home/xxy/ws_acesim/install/acesim_ros2/lib/acesim_ros2/acesim_play_ue --help --ros-args` | `cd /home/xxy/ws_acesim && PYTHONNOUSERSITE=1 colcon build --packages-select acesim_ros2 && source install/setup.zsh` |
| UE 窗口打开但没有视觉状态 | ACESim visual stream 没发布、endpoint 不一致或 bridge 没连接 | `python3 acesim/tools/ue5/verify_visual_stream.py --samples 5 --timeout-sec 10` | 确认 ACESim headless 正在发布 `tcp://0.0.0.0:5601`，UE 订阅 `tcp://127.0.0.1:5601`；日志应出现 `ACESim visual stream connected` 和 `ACESim visual state applied` |

## Reserved Feedback Interfaces

Phase 1 不发布 UE camera、depth、segmentation 或 event feedback。生成的插件只预留默认关闭的 `UACESimSensorFeedbackComponent` 设置、`FACESimSensorFrameHeader` 消息头名称和 `FACESimBridgeClock` 同步辅助名称，后续阶段可在这些通道上扩展：

- RGB：`tcp://127.0.0.1:5610`
- Depth：`tcp://127.0.0.1:5611`
- Segmentation：`tcp://127.0.0.1:5612`
- Events：`tcp://127.0.0.1:5613`

## Verification

仓库内自动验证覆盖生成器和视觉流工具：

```bash
python3 -m pytest tests/test_ue5_project_scaffold.py tests/test_ue5_visual_stream_tool.py -v
```

项目级 UE 编译验证：

```bash
/tmp/ACESim-unreal/UnrealEngine/Engine/Build/BatchFiles/Linux/Build.sh \
  ACESimUEEditor Linux Development \
  -Project=/tmp/ACESim-unreal/projects/ACESimUE/ACESimUE.uproject \
  -Progress \
  -NoHotReloadFromIDE \
  -NoUBA
```

无界面加载检查：

```bash
/usr/bin/timeout 120s \
  /tmp/ACESim-unreal/UnrealEngine/Engine/Binaries/Linux/UnrealEditor-Cmd \
  /tmp/ACESim-unreal/projects/ACESimUE/ACESimUE.uproject \
  -run=SmokeTest \
  -DDC-ForceMemoryCache \
  -ddc=NoZenLocalFallback \
  -NullRHI \
  -Unattended \
  -NoSplash \
  -NoSound
```

真实 ACESim -> UE bridge smoke 检查：

```bash
python3 -m acesim.deploy.aircraft.acesim_ros2.acesim_ros2.acesim_play_headless
```

另一个终端运行：

```bash
RUN_LIVE_BRIDGE_SMOKE=1 bash /home/xxy/ACESim/acesim/tools/ue5/smoke_ue_bridge.sh
```

`smoke_ue_bridge.sh` 会先编译 `ACESimUEEditor`，再运行 `SmokeTest`，最后在 live 模式下启动 UE game 并等待 bridge 日志。脚本会把 UE 的 `HOME`、`XDG_CONFIG_HOME` 和 `XDG_CACHE_HOME` 定向到 `/tmp/ACESim-unreal/smoke-home`，并用 `-DDC-ForceMemoryCache -ddc=NoZenLocalFallback` 让无界面 smoke 不依赖用户目录下的 Zen/local DDC 写入权限。

live 模式不会向 UE 传 `-ExecCmds=Quit`，而是将输出写到 `/tmp/ACESim-unreal/projects/ACESimUE/Saved/Logs/ACESimUEBridgeSmoke.log`，等到同时出现 `ACESim visual stream connected` 和 `ACESim visual state applied` 后再停止 UE。成功时脚本会打印 `Live bridge smoke passed`。

`tests/test_ue5_visual_stream_tool.py` 会创建本地 ZeroMQ PUB/SUB 并调用 `verify_visual_stream.py` 解码 3 个样本；如果当前 sandbox 禁止创建 localhost socket，该测试会跳过。真实机器上验证 UE 链路时，仍以 `tcp://127.0.0.1:5601` 的 live ACESim stream 为准。
