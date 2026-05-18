from __future__ import annotations

import importlib.util
import sys
from pathlib import Path
from types import ModuleType
from unittest.mock import Mock, patch

import pytest

ROOT = Path(__file__).resolve().parents[1]
ROS2_ROOT = ROOT / "acesim" / "deploy" / "aircraft" / "acesim_ros2"
MODULE_PATH = ROS2_ROOT / "acesim_ros2" / "acesim_play_ue.py"
DEFAULT_UE_ARGS = [
    "-Windowed",
    "-ForceRes",
    "-ResX=1280",
    "-ResY=720",
    "-WinX=64",
    "-WinY=64",
    "-DefaultViewportMouseCaptureMode=CaptureDuringMouseDown",
]
DEFAULT_RENDER_ARG = "-ACESimRenderPreset=performance"


def _load_acesim_play_ue() -> ModuleType:
    spec = importlib.util.spec_from_file_location("_test_acesim_play_ue", MODULE_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Failed to load module spec for {MODULE_PATH}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class _FakeProcess:
    def __init__(self) -> None:
        self.returncode: int | None = None
        self.terminated = False
        self.killed = False
        self.wait_timeout: float | None = None

    def poll(self) -> int | None:
        return self.returncode

    def terminate(self) -> None:
        self.terminated = True
        self.returncode = -15

    def kill(self) -> None:
        self.killed = True
        self.returncode = -9

    def wait(self, timeout: float | None = None) -> int | None:
        self.wait_timeout = timeout
        return self.returncode


def _write_airport_package_sidecars(runtime_root: Path) -> None:
    airport_dir = runtime_root / "Content" / "ACESim" / "Environment" / "Airport"
    model_dir = airport_dir / "Model"
    model_dir.mkdir(parents=True)
    (airport_dir / "ATTRIBUTION.txt").write_text("airport attribution\n", encoding="utf-8")
    (airport_dir / "airport_manifest.json").write_text("{}", encoding="utf-8")
    (model_dir / "Airport.uasset").write_text("mesh", encoding="utf-8")
    (runtime_root / "ACESimUE_PACKAGE_MANIFEST.json").write_text(
        '{"env_style":"airport"}\n',
        encoding="utf-8",
    )


def _write_heliport_package_sidecars(runtime_root: Path) -> None:
    heliport_dir = runtime_root / "Content" / "ACESim" / "Environment" / "Heliport"
    model_dir = heliport_dir / "Model"
    model_dir.mkdir(parents=True)
    (heliport_dir / "ATTRIBUTION.txt").write_text("heliport attribution\n", encoding="utf-8")
    (heliport_dir / "heliport_manifest.json").write_text("{}", encoding="utf-8")
    (model_dir / "Heliport.uasset").write_text("mesh", encoding="utf-8")
    (runtime_root / "ACESimUE_PACKAGE_MANIFEST.json").write_text(
        '{"env_style":"heliport"}\n',
        encoding="utf-8",
    )


def test_acesim_play_ue_launches_ue_and_steps_headless_env(tmp_path: Path) -> None:
    module = _load_acesim_play_ue()
    ue_executable = tmp_path / "ACESimUE"
    ue_executable.write_text("#!/usr/bin/env bash\n", encoding="utf-8")
    ue_executable.chmod(0o755)
    process = _FakeProcess()

    class FakeEnv:
        def __init__(self) -> None:
            self.steps = 0
            self.closed = False
            self.run = Mock()

        def step(self) -> None:
            self.steps += 1
            if self.steps == 3:
                process.returncode = 0

        def close(self) -> None:
            self.closed = True

    env = FakeEnv()
    popen = Mock(return_value=process)

    with patch.object(module, "make_env", return_value=env), patch.object(module.subprocess, "Popen", popen):
        module.main(["--ue-executable", str(ue_executable), "--ue-arg", "-windowed"])

    popen.assert_called_once_with([str(ue_executable), "-windowed", *DEFAULT_UE_ARGS[1:], DEFAULT_RENDER_ARG])
    assert env.steps == 3
    env.run.assert_not_called()
    assert env.closed
    assert not process.terminated


def test_acesim_play_ue_starts_headless_env_before_ue_process(tmp_path: Path) -> None:
    module = _load_acesim_play_ue()
    ue_executable = tmp_path / "ACESimUE"
    ue_executable.write_text("#!/usr/bin/env bash\n", encoding="utf-8")
    ue_executable.chmod(0o755)
    process = _FakeProcess()
    events: list[str] = []

    class FakeEnv:
        def step(self) -> None:
            process.returncode = 0

        def close(self) -> None:
            events.append("close_env")

    def make_env() -> FakeEnv:
        events.append("make_env")
        return FakeEnv()

    def popen(command: list[str]) -> _FakeProcess:
        events.append("popen")
        return process

    with patch.object(module, "make_env", make_env), patch.object(module.subprocess, "Popen", popen):
        module.main(["--ue-executable", str(ue_executable)])

    assert events[:2] == ["make_env", "popen"]


def test_acesim_play_ue_defaults_to_windowed_low_resolution(tmp_path: Path) -> None:
    module = _load_acesim_play_ue()
    ue_executable = tmp_path / "ACESimUE"
    ue_executable.write_text("#!/usr/bin/env bash\n", encoding="utf-8")
    ue_executable.chmod(0o755)
    process = _FakeProcess()

    class FakeEnv:
        def __init__(self) -> None:
            self.closed = False

        def step(self) -> None:
            process.returncode = 0

        def close(self) -> None:
            self.closed = True

    env = FakeEnv()
    popen = Mock(return_value=process)

    with patch.object(module, "make_env", return_value=env), patch.object(module.subprocess, "Popen", popen):
        module.main(["--ue-executable", str(ue_executable)])

    popen.assert_called_once_with([str(ue_executable), *DEFAULT_UE_ARGS, DEFAULT_RENDER_ARG])
    assert env.closed


def test_acesim_play_ue_user_args_override_default_window_and_resolution(tmp_path: Path) -> None:
    module = _load_acesim_play_ue()
    ue_executable = tmp_path / "ACESimUE"
    ue_executable.write_text("#!/usr/bin/env bash\n", encoding="utf-8")
    ue_executable.chmod(0o755)
    process = _FakeProcess()

    class FakeEnv:
        def __init__(self) -> None:
            self.closed = False

        def step(self) -> None:
            process.returncode = 0

        def close(self) -> None:
            self.closed = True

    env = FakeEnv()
    popen = Mock(return_value=process)

    with patch.object(module, "make_env", return_value=env), patch.object(module.subprocess, "Popen", popen):
        module.main(
            [
                "--ue-executable",
                str(ue_executable),
                "--ue-arg",
                "-fullscreen",
                "--ue-arg",
                "-ResX=1920",
                "--ue-arg",
                "-ResY=1080",
            ]
        )

    popen.assert_called_once_with(
        [
            str(ue_executable),
            "-fullscreen",
            "-ForceRes",
            "-ResX=1920",
            "-ResY=1080",
            "-WinX=64",
            "-WinY=64",
            "-DefaultViewportMouseCaptureMode=CaptureDuringMouseDown",
            DEFAULT_RENDER_ARG,
        ]
    )
    assert env.closed


def test_acesim_play_ue_user_args_override_force_res_and_window_position(tmp_path: Path) -> None:
    module = _load_acesim_play_ue()
    ue_executable = tmp_path / "ACESimUE"
    ue_executable.write_text("#!/usr/bin/env bash\n", encoding="utf-8")
    ue_executable.chmod(0o755)

    command = module._build_ue_command(
        ue_mode="package",
        ue_executable=str(ue_executable),
        unreal_editor="unused",
        ue_project="unused",
        ue_args=["-ForceRes", "-WinX=200", "-WinY=140", "-windowed", "-ResX=1600", "-ResY=900"],
    )

    assert command == [
        str(ue_executable),
        "-windowed",
        "-ForceRes",
        "-ResX=1600",
        "-ResY=900",
        "-WinX=200",
        "-WinY=140",
        "-DefaultViewportMouseCaptureMode=CaptureDuringMouseDown",
        DEFAULT_RENDER_ARG,
    ]


def test_acesim_play_ue_user_args_override_default_mouse_capture_mode(tmp_path: Path) -> None:
    module = _load_acesim_play_ue()
    ue_executable = tmp_path / "ACESimUE"
    ue_executable.write_text("#!/usr/bin/env bash\n", encoding="utf-8")
    ue_executable.chmod(0o755)

    command = module._build_ue_command(
        ue_mode="package",
        ue_executable=str(ue_executable),
        unreal_editor="unused",
        ue_project="unused",
        ue_args=["-DefaultViewportMouseCaptureMode=CapturePermanently"],
    )

    assert command == [
        str(ue_executable),
        "-Windowed",
        "-ForceRes",
        "-ResX=1280",
        "-ResY=720",
        "-WinX=64",
        "-WinY=64",
        "-DefaultViewportMouseCaptureMode=CapturePermanently",
        DEFAULT_RENDER_ARG,
    ]


def test_acesim_play_ue_rejects_missing_ue_executable(tmp_path: Path) -> None:
    module = _load_acesim_play_ue()
    make_env = Mock()

    with patch.object(module, "make_env", make_env):
        with pytest.raises(FileNotFoundError, match="ACESim UE executable"):
            module.main(["--ue-executable", str(tmp_path / "missing-ACESimUE")])

    make_env.assert_not_called()


def test_acesim_play_ue_passes_project_name_for_packaged_linux_binary(tmp_path: Path) -> None:
    module = _load_acesim_play_ue()
    ue_executable = tmp_path / "Linux" / "ACESimUE" / "Binaries" / "Linux" / "ACESimUE"
    ue_executable.parent.mkdir(parents=True)
    ue_executable.write_text("#!/usr/bin/env bash\n", encoding="utf-8")
    ue_executable.chmod(0o755)
    _write_heliport_package_sidecars(tmp_path / "Linux" / "ACESimUE")

    command = module._build_ue_command(
        ue_mode="package",
        ue_executable=str(ue_executable),
        unreal_editor="unused",
        ue_project="unused",
        ue_args=[],
    )

    assert command == [str(ue_executable), "ACESimUE", *DEFAULT_UE_ARGS, DEFAULT_RENDER_ARG]


def test_acesim_play_ue_rejects_packaged_airport_runtime_without_sidecars(tmp_path: Path) -> None:
    module = _load_acesim_play_ue()
    ue_executable = tmp_path / "Linux" / "ACESimUE" / "Binaries" / "Linux" / "ACESimUE"
    ue_executable.parent.mkdir(parents=True)
    ue_executable.write_text("#!/usr/bin/env bash\n", encoding="utf-8")
    ue_executable.chmod(0o755)

    with pytest.raises(RuntimeError, match="heliport package is incomplete"):
        module._build_ue_command(
            ue_mode="package",
            ue_executable=str(ue_executable),
            unreal_editor="unused",
            ue_project="unused",
            ue_args=[],
        )


def test_acesim_play_ue_allows_explicit_airport_package_with_airport_sidecars(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    module = _load_acesim_play_ue()
    monkeypatch.setenv("ACESIM_UE_ENV_STYLE", "airport")
    ue_executable = tmp_path / "Linux" / "ACESimUE" / "Binaries" / "Linux" / "ACESimUE"
    ue_executable.parent.mkdir(parents=True)
    ue_executable.write_text("#!/usr/bin/env bash\n", encoding="utf-8")
    ue_executable.chmod(0o755)
    _write_airport_package_sidecars(tmp_path / "Linux" / "ACESimUE")

    command = module._build_ue_command(
        ue_mode="package",
        ue_executable=str(ue_executable),
        unreal_editor="unused",
        ue_project="unused",
        ue_args=[],
    )

    assert command == [str(ue_executable), "ACESimUE", *DEFAULT_UE_ARGS, DEFAULT_RENDER_ARG]


def test_acesim_play_ue_allows_explicit_testfield_package_without_airport_sidecars(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    module = _load_acesim_play_ue()
    monkeypatch.setenv("ACESIM_UE_ENV_STYLE", "testfield")
    ue_executable = tmp_path / "Linux" / "ACESimUE" / "Binaries" / "Linux" / "ACESimUE"
    ue_executable.parent.mkdir(parents=True)
    ue_executable.write_text("#!/usr/bin/env bash\n", encoding="utf-8")
    ue_executable.chmod(0o755)

    command = module._build_ue_command(
        ue_mode="package",
        ue_executable=str(ue_executable),
        unreal_editor="unused",
        ue_project="unused",
        ue_args=[],
    )

    assert command[0] == str(ue_executable)


def test_acesim_play_ue_package_mode_rejects_missing_default_package(tmp_path: Path) -> None:
    module = _load_acesim_play_ue()

    with patch.object(module, "DEFAULT_UE_EXECUTABLE", str(tmp_path / "packages" / "ACESimUE")):
        with patch.object(module, "make_env", Mock()) as make_env:
            with pytest.raises(FileNotFoundError, match="Build the UE package first"):
                module.main([])

    make_env.assert_not_called()


def test_acesim_play_ue_editor_mode_uses_unreal_editor_game(tmp_path: Path) -> None:
    module = _load_acesim_play_ue()
    unreal_editor = tmp_path / "UnrealEditor"
    unreal_editor.write_text("#!/usr/bin/env bash\n", encoding="utf-8")
    unreal_editor.chmod(0o755)
    project_file = tmp_path / "ACESimUE.uproject"
    project_file.write_text("{}", encoding="utf-8")
    process = _FakeProcess()

    class FakeEnv:
        def __init__(self) -> None:
            self.steps = 0
            self.closed = False

        def step(self) -> None:
            self.steps += 1
            process.returncode = 0

        def close(self) -> None:
            self.closed = True

    env = FakeEnv()
    popen = Mock(return_value=process)

    with (
        patch.object(module, "DEFAULT_UE_EXECUTABLE", str(tmp_path / "packages" / "ACESimUE")),
        patch.object(module, "DEFAULT_UNREAL_EDITOR", str(unreal_editor)),
        patch.object(module, "DEFAULT_UE_PROJECT", str(project_file)),
        patch.object(module, "make_env", return_value=env),
        patch.object(module.subprocess, "Popen", popen),
    ):
        module.main(["--ue-mode", "editor", "--ue-arg", "-windowed"])

    popen.assert_called_once_with(
        [str(unreal_editor), str(project_file), "-game", "-windowed", *DEFAULT_UE_ARGS[1:], DEFAULT_RENDER_ARG]
    )
    assert env.steps == 1
    assert env.closed


def test_acesim_play_ue_ignores_ros_launch_arguments(tmp_path: Path) -> None:
    module = _load_acesim_play_ue()
    ue_executable = tmp_path / "ACESimUE"
    ue_executable.write_text("#!/usr/bin/env bash\n", encoding="utf-8")
    ue_executable.chmod(0o755)
    process = _FakeProcess()

    class FakeEnv:
        def __init__(self) -> None:
            self.steps = 0
            self.closed = False

        def step(self) -> None:
            self.steps += 1
            process.returncode = 0

        def close(self) -> None:
            self.closed = True

    env = FakeEnv()
    popen = Mock(return_value=process)

    with patch.object(module, "make_env", return_value=env), patch.object(module.subprocess, "Popen", popen):
        module.main(
            [
                "--ue-executable",
                str(ue_executable),
                "--ros-args",
                "-r",
                "__node:=acesim_play_ue",
                "--params-file",
                "/tmp/launch_params.yaml",
            ]
        )

    popen.assert_called_once_with([str(ue_executable), *DEFAULT_UE_ARGS, DEFAULT_RENDER_ARG])
    assert env.steps == 1
    assert env.closed


def test_acesim_play_ue_ignores_ros_launch_arguments_from_sys_argv(tmp_path: Path) -> None:
    module = _load_acesim_play_ue()
    ue_executable = tmp_path / "ACESimUE"
    ue_executable.write_text("#!/usr/bin/env bash\n", encoding="utf-8")
    ue_executable.chmod(0o755)
    process = _FakeProcess()

    class FakeEnv:
        def __init__(self) -> None:
            self.steps = 0
            self.closed = False

        def step(self) -> None:
            self.steps += 1
            process.returncode = 0

        def close(self) -> None:
            self.closed = True

    env = FakeEnv()
    popen = Mock(return_value=process)

    with (
        patch.object(module, "make_env", return_value=env),
        patch.object(module.subprocess, "Popen", popen),
        patch.object(
            sys,
            "argv",
            [
                "acesim_play_ue",
                "--ue-executable",
                str(ue_executable),
                "--ros-args",
            ],
        ),
    ):
        module.main()

    popen.assert_called_once_with([str(ue_executable), *DEFAULT_UE_ARGS, DEFAULT_RENDER_ARG])
    assert env.steps == 1
    assert env.closed


def test_acesim_play_ue_writes_non_fullscreen_settings_for_packaged_runtime(tmp_path: Path) -> None:
    module = _load_acesim_play_ue()
    ue_executable = tmp_path / "Linux" / "ACESimUE"
    ue_executable.parent.mkdir(parents=True)
    ue_executable.write_text("#!/usr/bin/env bash\n", encoding="utf-8")
    ue_executable.chmod(0o755)
    process = _FakeProcess()

    class FakeEnv:
        def close(self) -> None:
            pass

        def step(self) -> None:
            process.returncode = 0

    settings_path = ue_executable.parent / "Saved" / "Config" / "Linux" / "GameUserSettings.ini"
    settings_path.parent.mkdir(parents=True)
    settings_path.write_text(
        "[/Script/Engine.GameUserSettings]\n" "ResolutionSizeX=1920\n" "ResolutionSizeY=1080\n" "FullscreenMode=1\n",
        encoding="utf-8",
    )

    with (
        patch.object(module, "make_env", return_value=FakeEnv()),
        patch.object(module.subprocess, "Popen", return_value=process),
    ):
        module.main(["--ue-executable", str(ue_executable)])

    settings_text = settings_path.read_text(encoding="utf-8")
    assert "ResolutionSizeX=1280" in settings_text
    assert "ResolutionSizeY=720" in settings_text
    assert "FullscreenMode=0" in settings_text
    assert "LastConfirmedFullscreenMode=0" in settings_text

    input_settings_path = ue_executable.parent / "Saved" / "Config" / "Linux" / "Input.ini"
    input_settings_text = input_settings_path.read_text(encoding="utf-8")
    assert "bCaptureMouseOnLaunch=False" in input_settings_text
    assert "DefaultViewportMouseCaptureMode=CaptureDuringMouseDown" in input_settings_text
    assert "DefaultViewportMouseLockMode=DoNotLock" in input_settings_text


def test_acesim_play_ue_writes_settings_for_packaged_linux_archive_layout(tmp_path: Path) -> None:
    module = _load_acesim_play_ue()
    ue_executable = tmp_path / "Linux" / "ACESimUE" / "Binaries" / "Linux" / "ACESimUE"
    ue_executable.parent.mkdir(parents=True)
    ue_executable.write_text("#!/usr/bin/env bash\n", encoding="utf-8")
    ue_executable.chmod(0o755)
    _write_heliport_package_sidecars(tmp_path / "Linux" / "ACESimUE")
    process = _FakeProcess()

    class FakeEnv:
        def close(self) -> None:
            pass

        def step(self) -> None:
            process.returncode = 0

    with (
        patch.object(module, "make_env", return_value=FakeEnv()),
        patch.object(module.subprocess, "Popen", return_value=process),
    ):
        module.main(["--ue-executable", str(ue_executable)])

    runtime_settings = tmp_path / "Linux" / "ACESimUE" / "Saved" / "Config" / "Linux" / "GameUserSettings.ini"
    binary_local_settings = ue_executable.parent / "Saved" / "Config" / "Linux" / "GameUserSettings.ini"
    runtime_input = tmp_path / "Linux" / "ACESimUE" / "Saved" / "Config" / "Linux" / "Input.ini"
    binary_local_input = ue_executable.parent / "Saved" / "Config" / "Linux" / "Input.ini"
    assert "FullscreenMode=0" in runtime_settings.read_text(encoding="utf-8")
    assert "ResolutionSizeX=1280" in binary_local_settings.read_text(encoding="utf-8")
    assert "DefaultViewportMouseCaptureMode=CaptureDuringMouseDown" in runtime_input.read_text(encoding="utf-8")
    assert "DefaultViewportMouseLockMode=DoNotLock" in binary_local_input.read_text(encoding="utf-8")
    assert process.returncode == 0


def test_acesim_play_ue_writes_non_fullscreen_settings_for_editor_project(tmp_path: Path) -> None:
    module = _load_acesim_play_ue()
    unreal_editor = tmp_path / "UnrealEditor"
    unreal_editor.write_text("#!/usr/bin/env bash\n", encoding="utf-8")
    unreal_editor.chmod(0o755)
    project_dir = tmp_path / "ACESimUE"
    project_file = project_dir / "ACESimUE.uproject"
    project_dir.mkdir()
    project_file.write_text("{}", encoding="utf-8")
    process = _FakeProcess()

    class FakeEnv:
        def close(self) -> None:
            pass

        def step(self) -> None:
            process.returncode = 0

    with (
        patch.object(module, "make_env", return_value=FakeEnv()),
        patch.object(module.subprocess, "Popen", return_value=process),
    ):
        module.main(["--ue-mode", "editor", "--unreal-editor", str(unreal_editor), "--ue-project", str(project_file)])

    settings_path = project_dir / "Saved" / "Config" / "LinuxEditor" / "GameUserSettings.ini"
    settings_text = settings_path.read_text(encoding="utf-8")
    assert "ResolutionSizeX=1280" in settings_text
    assert "ResolutionSizeY=720" in settings_text
    assert "FullscreenMode=0" in settings_text

    input_settings_path = project_dir / "Saved" / "Config" / "LinuxEditor" / "Input.ini"
    input_settings_text = input_settings_path.read_text(encoding="utf-8")
    assert "bCaptureMouseOnLaunch=False" in input_settings_text
    assert "DefaultViewportMouseCaptureMode=CaptureDuringMouseDown" in input_settings_text
    assert "DefaultViewportMouseLockMode=DoNotLock" in input_settings_text


def test_acesim_play_ue_prints_actual_command_before_launch(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    module = _load_acesim_play_ue()
    ue_executable = tmp_path / "ACESimUE"
    ue_executable.write_text("#!/usr/bin/env bash\n", encoding="utf-8")
    ue_executable.chmod(0o755)
    process = _FakeProcess()

    class FakeEnv:
        def close(self) -> None:
            pass

        def step(self) -> None:
            process.returncode = 0

    with (
        patch.object(module, "make_env", return_value=FakeEnv()),
        patch.object(module.subprocess, "Popen", return_value=process),
    ):
        module.main(["--ue-executable", str(ue_executable)])

    stdout = capsys.readouterr().out
    assert "Starting UE:" in stdout
    assert str(ue_executable) in stdout
    assert "-Windowed -ForceRes -ResX=1280 -ResY=720 -WinX=64 -WinY=64" in stdout
    assert "-DefaultViewportMouseCaptureMode=CaptureDuringMouseDown" in stdout


def test_acesim_play_ue_supports_render_presets(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    module = _load_acesim_play_ue()
    ue_executable = tmp_path / "ACESimUE"
    ue_executable.write_text("#!/usr/bin/env bash\n", encoding="utf-8")
    ue_executable.chmod(0o755)
    process = _FakeProcess()

    class FakeEnv:
        def close(self) -> None:
            pass

        def step(self) -> None:
            process.returncode = 0

    command = module._build_ue_command(
        ue_mode="package",
        ue_executable=str(ue_executable),
        unreal_editor="unused",
        ue_project="unused",
        ue_args=[],
        render_preset="raytracing",
    )
    assert "-ACESimRenderPreset=raytracing" in command
    assert "-vulkan" in command

    with (
        patch.object(module, "make_env", return_value=FakeEnv()),
        patch.object(module.subprocess, "Popen", return_value=process),
    ):
        module.main(["--ue-executable", str(ue_executable), "--render-preset", "lumen"])

    stdout = capsys.readouterr().out
    assert "ACESim UE render preset: lumen" in stdout
    assert "-ACESimRenderPreset=lumen" in stdout


def test_acesim_play_ue_terminates_ue_process_on_interrupt(tmp_path: Path) -> None:
    module = _load_acesim_play_ue()
    ue_executable = tmp_path / "ACESimUE"
    ue_executable.write_text("#!/usr/bin/env bash\n", encoding="utf-8")
    ue_executable.chmod(0o755)
    process = _FakeProcess()

    class FakeEnv:
        def __init__(self) -> None:
            self.closed = False

        def step(self) -> None:
            raise KeyboardInterrupt

        def close(self) -> None:
            self.closed = True

    env = FakeEnv()

    with (
        patch.object(module, "make_env", return_value=env),
        patch.object(module.subprocess, "Popen", return_value=process),
    ):
        module.main(["--ue-executable", str(ue_executable), "--shutdown-timeout-sec", "0.5"])

    assert env.closed
    assert process.terminated
    assert process.wait_timeout == 0.5


def test_acesim_play_ue_does_not_launch_ue_process_when_make_env_fails(tmp_path: Path) -> None:
    module = _load_acesim_play_ue()
    ue_executable = tmp_path / "ACESimUE"
    ue_executable.write_text("#!/usr/bin/env bash\n", encoding="utf-8")
    ue_executable.chmod(0o755)
    popen = Mock()

    with (
        patch.object(module, "make_env", side_effect=RuntimeError("env exploded")),
        patch.object(module.subprocess, "Popen", popen),
    ):
        with pytest.raises(RuntimeError, match="env exploded"):
            module.main(["--ue-executable", str(ue_executable), "--shutdown-timeout-sec", "0.5"])

    popen.assert_not_called()


def test_acesim_play_ue_closes_env_when_ue_process_launch_fails(tmp_path: Path) -> None:
    module = _load_acesim_play_ue()
    ue_executable = tmp_path / "ACESimUE"
    ue_executable.write_text("#!/usr/bin/env bash\n", encoding="utf-8")
    ue_executable.chmod(0o755)

    class FakeEnv:
        def __init__(self) -> None:
            self.closed = False

        def close(self) -> None:
            self.closed = True

    env = FakeEnv()

    with (
        patch.object(module, "make_env", return_value=env),
        patch.object(module.subprocess, "Popen", side_effect=OSError("ue exploded")),
    ):
        with pytest.raises(OSError, match="ue exploded"):
            module.main(["--ue-executable", str(ue_executable), "--shutdown-timeout-sec", "0.5"])

    assert env.closed


def test_acesim_play_ue_is_registered_as_ros2_console_script() -> None:
    setup_py = (ROS2_ROOT / "setup.py").read_text(encoding="utf-8")
    assert "acesim_play_ue = acesim_ros2.acesim_play_ue:main" in setup_py


def test_linux_ue_launch_uses_acesim_play_ue_entrypoint() -> None:
    launch_py = (ROS2_ROOT / "launch" / "linux_ue.launch.py").read_text(encoding="utf-8")
    assert 'play_executable="acesim_play_ue"' in launch_py
    assert 'DeclareLaunchArgument("ue_mode", default_value="package"' in launch_py
    assert 'LaunchConfiguration("ue_mode")' in launch_py
    assert 'LaunchConfiguration("ue_executable")' in launch_py
    assert 'LaunchConfiguration("unreal_editor")' in launch_py
    assert 'LaunchConfiguration("ue_project")' in launch_py
    assert '"ACESIM_UE_MODE": ue_mode' in launch_py
    assert '"ACESIM_UE_EXECUTABLE": ue_executable' in launch_py
    assert '"ACESIM_UNREAL_EDITOR": unreal_editor' in launch_py
    assert '"ACESIM_UE_PROJECT": ue_project' in launch_py
