# UE5 Integration Tools

这组脚本把 ACESim 的 UE5 联调拆成三步：

1. `setup_ubuntu_ue5.sh`
   - 在 Ubuntu 上安装 UE5 源码编译依赖
   - 检查 `nvidia-smi`
   - 拉取 Epic 官方 UE5 源码
   - 执行 `Setup.sh`、`GenerateProjectFiles.sh`、`make UnrealEditor`
2. `check_ubuntu_ue5_host.sh`
   - 在不改动系统的前提下检查 `sudo`、`nvidia-smi`、UE 仓库访问和工具链状态
   - 输出诊断日志到 `/tmp/ACESim-unreal/logs/host_check.txt`
3. `create_project_scaffold.py`
   - 在 `/tmp/ACESim-unreal/projects/ACESimUE` 生成最小 C++ UE5 项目
   - 项目内自带 `ACESimBridge` 插件
   - 插件包含 `AACESimVehicleActor` 与 `UACESimVehicleSyncComponent`
4. `verify_visual_stream.py`
   - 直接订阅 `tcp://127.0.0.1:5602`
   - 校验 ACESim 发布的视觉状态 payload

## 默认目录

- UE5 源码：`/tmp/ACESim-unreal/UnrealEngine`
- UE5 工程：`/tmp/ACESim-unreal/projects/ACESimUE`
- 日志目录：`/tmp/ACESim-unreal/logs`

## 典型顺序

```bash
bash /home/xxy/ACESim/acesim/tools/ue5/check_ubuntu_ue5_host.sh

bash /home/xxy/ACESim/acesim/tools/ue5/setup_ubuntu_ue5.sh

python3 -m acesim.deploy.aircraft.acesim_ros2.acesim_ros2.acesim_play_headless

python3 /home/xxy/ACESim/acesim/tools/ue5/verify_visual_stream.py
```

UE5 编辑器启动后：

- 打开 `/tmp/ACESim-unreal/projects/ACESimUE/ACESimUE.uproject`
- 在关卡里放一个 `ACESimVehicleActor`
- 给它添加 `ACESimVehicleSyncComponent`
- 保持 endpoint 为 `tcp://127.0.0.1:5602`
- Play 后即可从 ACESim 订阅状态并驱动 actor
