from __future__ import annotations

import argparse
import os
import shlex
import subprocess
import sys
import time
from pathlib import Path
from typing import Sequence

from acesim.core.play import make_env
from acesim.env.base_env import BaseEnv

DEFAULT_UE_EXECUTABLE = "/home/xxy/ACESim-unreal/packages/ACESimUE-Linux/Linux/ACESimUE/Binaries/Linux/ACESimUE"
DEFAULT_UNREAL_EDITOR = "/home/xxy/ACESim-unreal/UnrealEngine/Engine/Binaries/Linux/UnrealEditor"
DEFAULT_UE_PROJECT = "/home/xxy/ACESim/acesim/third_party/unreal/ACESimUE/ACESimUE.uproject"
UE_READY_MARKERS = (
    "ACESim UE scene ready for visual stream",
    "ACESim real vehicle mesh loaded",
    "ACESim manifest-loaded assembly",
)
DEFAULT_EDITOR_READY_TIMEOUT_SEC = 180.0
DEFAULT_PACKAGE_READY_TIMEOUT_SEC = 60.0
DEFAULT_REALTIME_FACTOR = 1.0
DEFAULT_TIMESTEP_SEC = 0.001
DEFAULT_UE_ARGS = [
    "-Windowed",
    "-ForceRes",
    "-ResX=1280",
    "-ResY=720",
    "-WinX=64",
    "-WinY=64",
    "-DefaultViewportMouseCaptureMode=CaptureDuringMouseDown",
]
RENDER_PRESETS = ("performance", "lumen", "raytracing")
LIGHTING_PRESETS = ("cinematic_day", "clean_sim", "golden_hour", "mythic_forest_day", "performance_day")
UE_SETTINGS_TEXT = """[/Script/Engine.GameUserSettings]
ResolutionSizeX=1280
ResolutionSizeY=720
LastUserConfirmedResolutionSizeX=1280
LastUserConfirmedResolutionSizeY=720
FullscreenMode=0
LastConfirmedFullscreenMode=0
PreferredFullscreenMode=0
bUseDesktopResolutionForFullscreen=False
"""
UE_INPUT_SETTINGS_TEXT = """[/Script/Engine.InputSettings]
bCaptureMouseOnLaunch=False
DefaultViewportMouseCaptureMode=CaptureDuringMouseDown
DefaultViewportMouseLockMode=DoNotLock
"""


def _env_float_or_none(name: str) -> float | None:
    value = os.environ.get(name)
    if value is None or value == "":
        return None
    return float(value)


def _normalize_ue_arg_options(argv: Sequence[str] | None) -> Sequence[str] | None:
    if argv is None:
        argv = sys.argv[1:]

    if "--ros-args" in argv:
        argv = argv[: argv.index("--ros-args")]

    normalized: list[str] = []
    index = 0
    while index < len(argv):
        item = argv[index]
        if item == "--ue-arg" and index + 1 < len(argv):
            normalized.append(f"--ue-arg={argv[index + 1]}")
            index += 2
            continue
        normalized.append(item)
        index += 1
    return normalized


def _parse_args(argv: Sequence[str] | None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run ACESim/MuJoCo headless while a packaged UE app renders the visual stream."
    )
    parser.add_argument(
        "--ue-mode",
        choices=("package", "editor"),
        default=os.environ.get("ACESIM_UE_MODE", "package"),
        help=(
            "UE launch mode. The default 'package' mode only starts the packaged runtime. "
            "'editor' starts UnrealEditor <uproject> -game and may spend a long time compiling shader/DDC data."
        ),
    )
    parser.add_argument(
        "--ue-executable",
        default=os.environ.get("ACESIM_UE_EXECUTABLE", "auto"),
        help=("Path to a packaged ACESimUE executable. In package mode, 'auto' means " f"{DEFAULT_UE_EXECUTABLE}."),
    )
    parser.add_argument(
        "--unreal-editor",
        default=os.environ.get("ACESIM_UNREAL_EDITOR", DEFAULT_UNREAL_EDITOR),
        help="UnrealEditor executable used only with --ue-mode editor.",
    )
    parser.add_argument(
        "--ue-project",
        default=os.environ.get("ACESIM_UE_PROJECT", DEFAULT_UE_PROJECT),
        help="ACESimUE .uproject used only with --ue-mode editor.",
    )
    parser.add_argument(
        "--ue-arg",
        action="append",
        default=[],
        help="Additional argument passed to the UE executable. Repeat for multiple arguments.",
    )
    parser.add_argument(
        "--render-preset",
        choices=RENDER_PRESETS,
        default=os.environ.get("ACESIM_UE_RENDER_PRESET", "lumen"),
        help="Rendering preset expected by the packaged UE runtime.",
    )
    parser.add_argument(
        "--lighting-preset",
        choices=LIGHTING_PRESETS,
        default=os.environ.get("ACESIM_UE_LIGHTING_PRESET", "cinematic_day"),
        help="Outdoor lighting preset expected by the UE runtime.",
    )
    parser.add_argument(
        "--shutdown-timeout-sec",
        type=float,
        default=5.0,
        help="Seconds to wait for UE to exit after ACESim stops before killing it.",
    )
    parser.add_argument(
        "--ue-ready-timeout-sec",
        type=float,
        default=_env_float_or_none("ACESIM_UE_READY_TIMEOUT_SEC"),
        help="Seconds to wait for the UE scene-ready log marker before starting MuJoCo.",
    )
    parser.add_argument(
        "--realtime-factor",
        type=float,
        default=float(os.environ.get("ACESIM_UE_REALTIME_FACTOR", DEFAULT_REALTIME_FACTOR)),
        help="MuJoCo pacing factor in UE mode. Use 1.0 for realtime, or 0 to disable pacing.",
    )
    return parser.parse_args(_normalize_ue_arg_options(argv))


def _package_command_hint() -> str:
    return "bash acesim/third_party/unreal/ACESimUE/Tools/package_ue_runtime.sh"


def _resolve_ue_executable(path_text: str, *, default_package: bool = False) -> Path:
    path = Path(path_text).expanduser()
    if not path.is_file():
        if default_package:
            raise FileNotFoundError(
                f"Build the UE package first: {_package_command_hint()}\n"
                f"Expected packaged ACESimUE executable: {path}"
            )
        raise FileNotFoundError(f"ACESim UE executable not found: {path}")
    if not os.access(path, os.X_OK):
        raise PermissionError(f"ACESim UE executable is not executable: {path}")
    return path


def _resolve_unreal_editor(path_text: str) -> Path:
    path = Path(path_text).expanduser()
    if not path.is_file():
        raise FileNotFoundError(f"UnrealEditor not found: {path}")
    if not os.access(path, os.X_OK):
        raise PermissionError(f"UnrealEditor is not executable: {path}")
    return path


def _resolve_ue_project(path_text: str) -> Path:
    path = Path(path_text).expanduser()
    if not path.is_file():
        raise FileNotFoundError(f"ACESim UE project not found: {path}")
    return path


def _is_packaged_linux_binary(path: Path) -> bool:
    return path.name == "ACESimUE" and path.parts[-3:] == ("Binaries", "Linux", "ACESimUE")


def _packaged_runtime_root(executable: Path) -> Path:
    return executable.parents[2]


def _ue_arg_key(arg: str) -> str:
    lowered = arg.lower()
    if lowered in {"-windowed", "-fullscreen"}:
        return "window_mode"
    if lowered == "-forceres":
        return "forceres"
    if lowered.startswith("-resx="):
        return "resx"
    if lowered.startswith("-resy="):
        return "resy"
    if lowered.startswith("-winx="):
        return "winx"
    if lowered.startswith("-winy="):
        return "winy"
    if lowered.startswith("-defaultviewportmousecapturemode="):
        return "mouse_capture_mode"
    if lowered.startswith("-acesimrenderpreset="):
        return "acesim_render_preset"
    if lowered.startswith("-acesimlightingpreset="):
        return "acesim_lighting_preset"
    if lowered in {"-vulkan", "-opengl", "-nullrhi"}:
        return "rhi"
    return arg


def _merge_ue_args(default_args: Sequence[str], user_args: Sequence[str]) -> list[str]:
    merged: list[str] = []
    key_to_index: dict[str, int] = {}
    for arg in [*default_args, *user_args]:
        key = _ue_arg_key(arg)
        existing_index = key_to_index.get(key)
        if existing_index is None:
            key_to_index[key] = len(merged)
            merged.append(arg)
        else:
            merged[existing_index] = arg
    return merged


def _build_ue_command(
    *,
    ue_mode: str,
    ue_executable: str,
    unreal_editor: str,
    ue_project: str,
    ue_args: Sequence[str],
    render_preset: str = "lumen",
    lighting_preset: str = "cinematic_day",
) -> list[str]:
    if render_preset not in RENDER_PRESETS:
        raise ValueError(f"Unsupported ACESim UE render preset: {render_preset}")
    if lighting_preset not in LIGHTING_PRESETS:
        raise ValueError(f"Unsupported ACESim UE lighting preset: {lighting_preset}")
    preset_args = [f"-ACESimRenderPreset={render_preset}", f"-ACESimLightingPreset={lighting_preset}"]
    if render_preset == "raytracing":
        preset_args.append("-vulkan")
    command_args = _merge_ue_args(DEFAULT_UE_ARGS, [*preset_args, *ue_args])
    if ue_mode == "editor":
        editor = _resolve_unreal_editor(unreal_editor)
        project = _resolve_ue_project(ue_project)
        return [str(editor), str(project), "-game", *command_args]

    package_path = DEFAULT_UE_EXECUTABLE if ue_executable == "auto" else ue_executable
    executable = _resolve_ue_executable(package_path, default_package=ue_executable == "auto")
    command = [str(executable)]
    if _is_packaged_linux_binary(executable):
        command.append("ACESimUE")
    command.extend(command_args)
    return command


def _write_game_user_settings(path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(UE_SETTINGS_TEXT, encoding="utf-8")


def _write_input_settings(path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(UE_INPUT_SETTINGS_TEXT, encoding="utf-8")


def _packaged_game_user_settings_paths(executable: Path) -> list[Path]:
    paths: list[Path] = []
    if _is_packaged_linux_binary(executable):
        paths.append(executable.parents[2] / "Saved" / "Config" / "Linux" / "GameUserSettings.ini")
    elif executable.name == "ACESimUE.sh":
        paths.append(executable.parent / "ACESimUE" / "Saved" / "Config" / "Linux" / "GameUserSettings.ini")
    paths.append(executable.parent / "Saved" / "Config" / "Linux" / "GameUserSettings.ini")
    return list(dict.fromkeys(paths))


def _input_settings_path_for_game_user_settings(path: Path) -> Path:
    return path.with_name("Input.ini")


def _write_non_fullscreen_settings(*, ue_mode: str, ue_command: Sequence[str]) -> None:
    if ue_mode == "editor":
        project = Path(ue_command[1])
        settings_path = project.parent / "Saved" / "Config" / "LinuxEditor" / "GameUserSettings.ini"
        _write_game_user_settings(settings_path)
        _write_input_settings(_input_settings_path_for_game_user_settings(settings_path))
        return

    executable = Path(ue_command[0])
    for settings_path in _packaged_game_user_settings_paths(executable):
        _write_game_user_settings(settings_path)
        _write_input_settings(_input_settings_path_for_game_user_settings(settings_path))


def _ue_log_path_for_command(*, ue_mode: str, ue_command: Sequence[str]) -> Path:
    if ue_mode == "editor":
        project = Path(ue_command[1])
        return project.parent / "Saved" / "Logs" / f"{project.stem}.log"

    executable = Path(ue_command[0])
    if _is_packaged_linux_binary(executable):
        runtime_root = _packaged_runtime_root(executable)
    elif executable.name == "ACESimUE.sh":
        runtime_root = executable.parent / "ACESimUE"
    else:
        runtime_root = executable.parent
    project_name = "ACESimUE"
    return runtime_root / "Saved" / "Logs" / f"{project_name}.log"


def _ready_timeout_sec(*, ue_mode: str, override_sec: float | None) -> float:
    if override_sec is not None:
        if override_sec <= 0.0:
            raise ValueError("ACESim UE ready timeout must be positive.")
        return override_sec
    if ue_mode == "editor":
        return DEFAULT_EDITOR_READY_TIMEOUT_SEC
    return DEFAULT_PACKAGE_READY_TIMEOUT_SEC


def _wait_for_ue_ready(
    process: subprocess.Popen[bytes],
    log_path: Path,
    *,
    timeout_sec: float,
    poll_interval_sec: float = 0.1,
) -> str:
    start_time = time.monotonic()
    log_offset = log_path.stat().st_size if log_path.exists() else 0

    while True:
        returncode = process.poll()
        if returncode is not None:
            raise RuntimeError(
                f"UE exited before ACESim scene became ready "
                f"(exit code {returncode}); expected one of {UE_READY_MARKERS} in {log_path}"
            )

        if log_path.exists():
            log_size = log_path.stat().st_size
            if log_size < log_offset:
                log_offset = 0
            if log_size > log_offset:
                with log_path.open("rb") as stream:
                    stream.seek(log_offset)
                    chunk = stream.read(log_size - log_offset).decode("utf-8", errors="ignore")
                log_offset = log_size
                for marker in UE_READY_MARKERS:
                    if marker in chunk:
                        return marker

        if time.monotonic() - start_time >= timeout_sec:
            raise TimeoutError(
                f"Timed out after {timeout_sec:.1f}s waiting for ACESim UE scene readiness in {log_path}. "
                f"Expected one of: {', '.join(UE_READY_MARKERS)}"
            )
        time.sleep(poll_interval_sec)


def _env_step_period_sec(env: BaseEnv) -> float:
    model = getattr(env, "_mj_model", None)
    opt = getattr(model, "opt", None)
    timestep = getattr(opt, "timestep", DEFAULT_TIMESTEP_SEC)
    try:
        value = float(timestep)
    except (TypeError, ValueError):
        return DEFAULT_TIMESTEP_SEC
    if value <= 0.0:
        return DEFAULT_TIMESTEP_SEC
    return value


def _step_env_until_ue_exits(
    *,
    env: BaseEnv,
    ue_process: subprocess.Popen[bytes],
    realtime_factor: float,
) -> None:
    if realtime_factor < 0.0:
        raise ValueError("ACESim UE realtime factor must be non-negative.")

    if realtime_factor == 0.0:
        while ue_process.poll() is None:
            env.step()
            time.sleep(0.0)
        return

    start_wall_time = time.monotonic()
    step_period_wall_sec = _env_step_period_sec(env) / realtime_factor
    step_count = 0
    while ue_process.poll() is None:
        env.step()
        step_count += 1
        target_wall_time = start_wall_time + step_count * step_period_wall_sec
        sleep_sec = target_wall_time - time.monotonic()
        time.sleep(max(0.0, sleep_sec))


def _stop_ue_process(process: subprocess.Popen[bytes], timeout_sec: float) -> None:
    if process.poll() is not None:
        return

    process.terminate()
    try:
        process.wait(timeout=timeout_sec)
    except subprocess.TimeoutExpired:
        process.kill()
        process.wait()


def main(argv: Sequence[str] | None = None) -> None:
    args = _parse_args(argv)
    ue_command = _build_ue_command(
        ue_mode=args.ue_mode,
        ue_executable=args.ue_executable,
        unreal_editor=args.unreal_editor,
        ue_project=args.ue_project,
        ue_args=args.ue_arg,
        render_preset=args.render_preset,
        lighting_preset=args.lighting_preset,
    )
    _write_non_fullscreen_settings(ue_mode=args.ue_mode, ue_command=ue_command)
    print(f"ACESim UE render preset: {args.render_preset}", flush=True)
    print(f"ACESim UE lighting preset: {args.lighting_preset}", flush=True)
    print(f"Starting UE: {shlex.join(ue_command)}", flush=True)
    ue_process: subprocess.Popen[bytes] | None = None
    env: BaseEnv | None = None
    try:
        ue_process = subprocess.Popen(ue_command)
        ue_log_path = _ue_log_path_for_command(ue_mode=args.ue_mode, ue_command=ue_command)
        ready_marker = _wait_for_ue_ready(
            ue_process,
            ue_log_path,
            timeout_sec=_ready_timeout_sec(ue_mode=args.ue_mode, override_sec=args.ue_ready_timeout_sec),
        )
        print(f"ACESim UE ready marker: {ready_marker}", flush=True)
        env = make_env()
        _step_env_until_ue_exits(env=env, ue_process=ue_process, realtime_factor=args.realtime_factor)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if env is not None:
                env.close()
        finally:
            if ue_process is not None:
                _stop_ue_process(ue_process, args.shutdown_timeout_sec)


if __name__ == "__main__":
    main()
