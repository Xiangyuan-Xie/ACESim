from __future__ import annotations

import argparse
import json
import os
import shlex
import subprocess
import sys
import time
from pathlib import Path
from typing import Sequence

from acesim.core.play import make_env
from acesim.env.base_env import BaseEnv

DEFAULT_UE_EXECUTABLE = "/tmp/ACESim-unreal/packages/ACESimUE-Linux/Linux/ACESimUE/Binaries/Linux/ACESimUE"
DEFAULT_UNREAL_EDITOR = "/tmp/ACESim-unreal/UnrealEngine/Engine/Binaries/Linux/UnrealEditor"
DEFAULT_UE_PROJECT = "/tmp/ACESim-unreal/projects/ACESimUE/ACESimUE.uproject"
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
        default=os.environ.get("ACESIM_UE_RENDER_PRESET", "performance"),
        help="Rendering preset expected by the packaged UE runtime.",
    )
    parser.add_argument(
        "--shutdown-timeout-sec",
        type=float,
        default=5.0,
        help="Seconds to wait for UE to exit after ACESim stops before killing it.",
    )
    return parser.parse_args(_normalize_ue_arg_options(argv))


def _package_command_hint() -> str:
    return "bash acesim/tools/ue5/package_ue_runtime.sh"


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


def _environment_sidecar_profile(env_style: str) -> tuple[str, str] | None:
    normalized = env_style.strip().lower()
    if normalized == "heliport":
        return ("Heliport", "heliport")
    if normalized == "airport":
        return ("Airport", "airport")
    return None


def _validate_packaged_environment_runtime(executable: Path) -> None:
    env_style = os.environ.get("ACESIM_UE_ENV_STYLE", "heliport")
    profile = _environment_sidecar_profile(env_style)
    if profile is None:
        return
    if not _is_packaged_linux_binary(executable):
        return

    folder, prefix = profile
    runtime_root = _packaged_runtime_root(executable)
    environment_dir = runtime_root / "Content" / "ACESim" / "Environment" / folder
    package_manifest = runtime_root / "ACESimUE_PACKAGE_MANIFEST.json"
    required_files = [
        package_manifest,
        environment_dir / "ATTRIBUTION.txt",
        environment_dir / f"{prefix}_manifest.json",
    ]
    missing = [str(path) for path in required_files if not path.is_file()]
    model_assets = list((environment_dir / "Model").rglob("*.uasset"))
    if not model_assets:
        missing.append(str(environment_dir / "Model" / "*.uasset"))
    if missing:
        raise RuntimeError(
            f"ACESim UE {prefix} package is incomplete; rebuild it with "
            f"{_package_command_hint()} before launching. Missing: {', '.join(missing)}"
        )

    manifest = json.loads(package_manifest.read_text(encoding="utf-8"))
    if manifest.get("env_style") != prefix:
        raise RuntimeError(
            f"ACESim UE {prefix} package is incomplete; package marker is not for env_style={prefix}. "
            f"Rebuild it with {_package_command_hint()}."
        )


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
    render_preset: str = "performance",
) -> list[str]:
    if render_preset not in RENDER_PRESETS:
        raise ValueError(f"Unsupported ACESim UE render preset: {render_preset}")
    preset_args = [f"-ACESimRenderPreset={render_preset}"]
    if render_preset == "raytracing":
        preset_args.append("-vulkan")
    command_args = _merge_ue_args(DEFAULT_UE_ARGS, [*preset_args, *ue_args])
    if ue_mode == "editor":
        editor = _resolve_unreal_editor(unreal_editor)
        project = _resolve_ue_project(ue_project)
        return [str(editor), str(project), "-game", *command_args]

    package_path = DEFAULT_UE_EXECUTABLE if ue_executable == "auto" else ue_executable
    executable = _resolve_ue_executable(package_path, default_package=ue_executable == "auto")
    _validate_packaged_environment_runtime(executable)
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
    )
    _write_non_fullscreen_settings(ue_mode=args.ue_mode, ue_command=ue_command)
    print(f"ACESim UE render preset: {args.render_preset}", flush=True)
    print(f"Starting UE: {shlex.join(ue_command)}", flush=True)
    ue_process: subprocess.Popen[bytes] | None = None
    env: BaseEnv | None = None
    try:
        env = make_env()
        ue_process = subprocess.Popen(ue_command)
        while ue_process.poll() is None:
            env.step()
            # Keep this loop cooperative when MuJoCo steps are faster than wall time.
            time.sleep(0.0)
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
