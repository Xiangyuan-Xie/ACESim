import importlib.util
import os
import stat
import subprocess
import sys
import textwrap
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_UE_ROOT = "/home/xxy/ACESim-unreal"
ACESIMUE_DIR = ROOT / "acesim" / "third_party" / "unreal" / "ACESimUE"
UE_TOOL_DIR = ACESIMUE_DIR / "Tools"
REMOVED_PARENT_UE_TOOL_DIR = ROOT / "acesim" / "tools" / "ue5"
REMOVED_PARENT_UE_TOOL_TEXT = "acesim" + "/tools" + "/ue5"
REMOVED_SYNC_SCRIPT = "create_project" + "_scaffold.py"
REMOVED_SYNC_ENV = "ACESIM_UE_SKIP" + "_SYNC"
REMOVED_REGENERATE_ENV = "ACESIM_UE_SKIP" + "_REGENERATE"
REMOVED_SYNC_PROJECT = "/home/xxy/ACESim-unreal" + "/projects/ACESimUE"
UE_BASH_SCRIPTS = [
    UE_TOOL_DIR / "check_ubuntu_ue5_host.sh",
    UE_TOOL_DIR / "setup_ubuntu_ue5.sh",
    UE_TOOL_DIR / "package_ue_runtime.sh",
    UE_TOOL_DIR / "smoke_ue_bridge.sh",
]
REMOVED_DEVELOPMENT_SCRIPTS = [
    "export_mjcf_visual_assets.py",
    "generate_acesim_testfield_meshes.py",
    "generate_ue_ground_assets.py",
    "prepare_ue_airport_assets.py",
    "prepare_ue_environment_assets.py",
    "setup_ue_editor_plane.sh",
]
BASH_REEXEC_GUARD = """if [ -z "${BASH_VERSION:-}" ]; then
  if command -v bash >/dev/null 2>&1; then
    exec bash "$0" "$@"
  fi
  echo "This script requires bash. Install bash or run it with: bash $0" >&2
  exit 1
fi
"""


def _write_executable(path: Path, content: str) -> None:
    path.write_text(textwrap.dedent(content).lstrip(), encoding="utf-8")
    path.chmod(path.stat().st_mode | stat.S_IXUSR)


def test_removed_development_scripts_are_not_in_ue_tools() -> None:
    for script_name in REMOVED_DEVELOPMENT_SCRIPTS:
        assert not (UE_TOOL_DIR / script_name).exists(), script_name


def test_parent_ue_tools_directory_is_removed() -> None:
    assert not REMOVED_PARENT_UE_TOOL_DIR.exists()


def test_acesimue_owns_all_ue_tools() -> None:
    expected_tools = [
        "check_ubuntu_ue5_host.sh",
        "setup_ubuntu_ue5.sh",
        "package_ue_runtime.sh",
        "smoke_ue_bridge.sh",
        "verify_ue_runtime_visual.py",
        "verify_visual_stream.py",
        "verify_acesim_assets.py",
        "cleanup_ue_processes.py",
    ]

    for tool_name in expected_tools:
        assert (UE_TOOL_DIR / tool_name).is_file(), tool_name
    for tool_name in expected_tools[:4]:
        assert os.access(UE_TOOL_DIR / tool_name, os.X_OK), tool_name


def test_acesimue_readme_is_development_and_tool_guide() -> None:
    readme = (ACESIMUE_DIR / "README.md").read_text(encoding="utf-8")

    assert "Tools/check_ubuntu_ue5_host.sh" in readme
    assert "Tools/package_ue_runtime.sh" in readme
    assert REMOVED_PARENT_UE_TOOL_TEXT not in readme


def test_active_ue_tool_scripts_do_not_call_removed_development_scripts() -> None:
    active_paths = [
        *UE_BASH_SCRIPTS,
        ROOT / "README.md",
        ACESIMUE_DIR / "README.md",
        ACESIMUE_DIR / "AGENT.md",
        ROOT / "acesim" / "deploy" / "aircraft" / "acesim_ros2" / "acesim_ros2" / "acesim_play_ue.py",
    ]
    payload = "\n".join(path.read_text(encoding="utf-8") for path in active_paths)

    for script_name in REMOVED_DEVELOPMENT_SCRIPTS:
        assert script_name not in payload
    assert "SKETCHFAB_API_TOKEN" not in payload
    assert "ACESIM_UE_FORCE_ASSET_IMPORT" not in payload


def test_host_check_falls_back_when_rg_is_missing(tmp_path: Path) -> None:
    bin_dir = tmp_path / "bin"
    log_dir = tmp_path / "logs"
    bin_dir.mkdir()

    _write_executable(
        bin_dir / "sudo",
        """
        #!/usr/bin/env bash
        exit 0
        """,
    )
    _write_executable(
        bin_dir / "git",
        """
        #!/usr/bin/env bash
        if [ "$1" = "ls-remote" ] && [ "$2" = "--tags" ]; then
          echo "abc123 refs/tags/5.7.4-release"
          exit 0
        fi
        exit 1
        """,
    )
    _write_executable(
        bin_dir / "nvidia-smi",
        """
        #!/usr/bin/env bash
        exit 0
        """,
    )

    env = os.environ.copy()
    env["PATH"] = f"{bin_dir}:/usr/bin:/bin:/usr/sbin"
    env["UE_LOG_DIR"] = str(log_dir)

    result = subprocess.run(
        ["bash", str(UE_TOOL_DIR / "check_ubuntu_ue5_host.sh")],
        check=True,
        capture_output=True,
        text=True,
        env=env,
    )

    assert "rg: command not found" not in result.stdout
    assert "sudo_non_interactive=ok" in result.stdout
    assert "nvidia_smi=ok" in result.stdout
    assert "repo_access=ok" in result.stdout
    assert (log_dir / "host_check.txt").is_file()


def test_ue_tool_scripts_default_to_home_workspace() -> None:
    for script_path in UE_BASH_SCRIPTS:
        script = script_path.read_text(encoding="utf-8")
        assert f'UE_ROOT="${{UE_ROOT:-{DEFAULT_UE_ROOT}}}"' in script, script_path


def test_ue_shell_scripts_reexec_under_bash_when_called_from_other_shells() -> None:
    for script_path in UE_BASH_SCRIPTS:
        script = script_path.read_text(encoding="utf-8")
        assert script.startswith("#!/usr/bin/env bash\n"), script_path
        assert BASH_REEXEC_GUARD in script, script_path
        assert script.index(BASH_REEXEC_GUARD) < script.index("set -euo pipefail")


def test_host_check_can_be_invoked_through_zsh(tmp_path: Path) -> None:
    bin_dir = tmp_path / "bin"
    log_dir = tmp_path / "logs"
    bin_dir.mkdir()

    _write_executable(bin_dir / "sudo", "#!/usr/bin/env bash\nexit 0\n")
    _write_executable(
        bin_dir / "git",
        """
        #!/usr/bin/env bash
        if [ "$1" = "ls-remote" ] && [ "$2" = "--tags" ]; then
          echo "abc123 refs/tags/5.7.4-release"
          exit 0
        fi
        exit 1
        """,
    )
    _write_executable(bin_dir / "nvidia-smi", "#!/usr/bin/env bash\nexit 0\n")

    env = os.environ.copy()
    env["PATH"] = f"{bin_dir}:/usr/bin:/bin:/usr/sbin"
    env["UE_LOG_DIR"] = str(log_dir)

    result = subprocess.run(
        ["zsh", str(UE_TOOL_DIR / "check_ubuntu_ue5_host.sh")],
        check=True,
        capture_output=True,
        text=True,
        env=env,
    )

    assert "sudo_non_interactive=ok" in result.stdout
    assert "nvidia_smi=ok" in result.stdout
    assert "repo_access=ok" in result.stdout
    assert (log_dir / "host_check.txt").is_file()


def test_host_check_rejects_empty_ls_remote_output(tmp_path: Path) -> None:
    bin_dir = tmp_path / "bin"
    log_dir = tmp_path / "logs"
    bin_dir.mkdir()

    _write_executable(bin_dir / "sudo", "#!/usr/bin/env bash\nexit 0\n")
    _write_executable(
        bin_dir / "git",
        """
        #!/usr/bin/env bash
        if [ "$1" = "ls-remote" ]; then
          exit 0
        fi
        exit 1
        """,
    )
    _write_executable(bin_dir / "nvidia-smi", "#!/usr/bin/env bash\nexit 0\n")

    env = os.environ.copy()
    env["PATH"] = f"{bin_dir}:/usr/bin:/bin:/usr/sbin"
    env["UE_LOG_DIR"] = str(log_dir)

    result = subprocess.run(
        ["bash", str(UE_TOOL_DIR / "check_ubuntu_ue5_host.sh")],
        check=True,
        capture_output=True,
        text=True,
        env=env,
    )

    assert "repo_access=blocked" in result.stdout


def test_setup_script_builds_external_engine_and_synced_acesimue_project() -> None:
    script = (UE_TOOL_DIR / "setup_ubuntu_ue5.sh").read_text(encoding="utf-8")

    assert "UE_REPO_URL" in script
    assert 'UE_REF="${UE_REF:-5.7.4-release}"' in script
    assert "git clone --branch" in script
    assert "EpicGames/UnrealEngine.git" in script
    assert "git submodule" not in script
    assert "Building ACESimUE project from submodule workspace" in script
    assert "verify_acesim_assets.py" in script
    assert script.index("verify_acesim_assets.py") < script.index("ACESimUEEditor Linux Development")
    assert REMOVED_SYNC_SCRIPT not in script
    assert REMOVED_SYNC_ENV not in script
    assert REMOVED_REGENERATE_ENV not in script
    assert REMOVED_SYNC_PROJECT not in script
    assert 'Build.sh" ShaderCompileWorker Linux Development' in script
    assert 'Build.sh" ACESimUEEditor Linux Development' in script
    assert '-Project="${UE_PROJECT_DIR}/ACESimUE.uproject"' in script
    assert "-NoHotReloadFromIDE" in script
    assert script.count("-NoUBA") >= 2


def test_package_runtime_script_builds_and_archives_synced_project_without_asset_generation() -> None:
    script = (UE_TOOL_DIR / "package_ue_runtime.sh").read_text(encoding="utf-8")

    assert "Packaging ACESimUE project from submodule workspace" in script
    assert "verify_acesim_assets.py" in script
    assert script.index("verify_acesim_assets.py") < script.index("Building ShaderCompileWorker")
    assert REMOVED_SYNC_SCRIPT not in script
    assert REMOVED_SYNC_ENV not in script
    assert REMOVED_REGENERATE_ENV not in script
    assert REMOVED_SYNC_PROJECT not in script
    assert 'Build.sh" ShaderCompileWorker Linux Development' in script
    assert 'Build.sh" ACESimUEEditor Linux Development' in script
    assert "RunUAT.sh" in script
    assert "-nocompileuat" in script
    assert "-NoUBA" in script
    assert "BuildCookRun" in script
    assert "-archivedirectory=${UE_PACKAGE_DIR}" in script
    assert "find_acesimue_executable" in script
    assert "*/ACESimUE/Binaries/Linux/ACESimUE" in script
    assert "ACESimUE executable:" in script
    assert "verify_ue_runtime_visual.py" in script
    assert "write_package_marker" in script
    for script_name in REMOVED_DEVELOPMENT_SCRIPTS:
        assert script_name not in script


def test_package_runtime_script_runs_visual_verifier_unless_explicitly_skipped() -> None:
    script = (UE_TOOL_DIR / "package_ue_runtime.sh").read_text(encoding="utf-8")

    assert 'ACESIM_UE_SKIP_VISUAL_VERIFY="${ACESIM_UE_SKIP_VISUAL_VERIFY:-0}"' in script
    assert 'ACESIM_UE_VISUAL_VERIFY_OFFSCREEN="${ACESIM_UE_VISUAL_VERIFY_OFFSCREEN:-auto}"' in script
    assert 'ACESIM_UE_RENDER_PRESET="${ACESIM_UE_RENDER_PRESET:-lumen}"' in script
    assert "verify_ue_runtime_visual.py" in script
    assert "--render-preset" in script
    assert "--lighting-preset" in script
    assert 'ACESIM_UE_LIGHTING_PRESET="${ACESIM_UE_LIGHTING_PRESET:-cinematic_day}"' in script
    assert "--ue-executable" in script
    assert "ACESIM_UE_SKIP_VISUAL_VERIFY=1" in script
    assert "visual_verify_args" in script
    assert "--offscreen" in script
    assert "DISPLAY" in script
    assert "WAYLAND_DISPLAY" in script
    assert script.index("verify_ue_runtime_visual.py") > script.index("write_package_marker")


def test_package_runtime_script_preflights_runtime_dependencies() -> None:
    script = (UE_TOOL_DIR / "package_ue_runtime.sh").read_text(encoding="utf-8")

    assert "Checking ACESimUE runtime assets" in script
    assert "Missing UnrealEditor:" in script
    assert "Missing UnrealEditor-Cmd:" in script
    assert "Missing ShaderCompileWorker:" in script
    assert "Missing ACESim UE project:" in script
    assert "Missing ACESimBridge runtime plugin" in script
    assert "DefaultEngine.ini still references OpenWorld" in script
    assert "DDC path is not writable" in script
    assert "Available space under" in script


def test_asset_verifier_documents_lfs_pull_for_missing_or_pointer_assets() -> None:
    verifier = (UE_TOOL_DIR / "verify_acesim_assets.py").read_text(encoding="utf-8")

    assert "Content/ACESim/x500_arm2x/visual_manifest.json" in verifier
    assert "Content/ACESim/x500_arm2x/base_link.uasset" in verifier
    assert "Content/ACESim/Environment/Ground/Materials/M_ACESim_HelipadConcrete.uasset" in verifier
    assert "Content/ACESim/Environment/Ground/Textures/T_ACESim_HelipadTop_BaseColor.uasset" in verifier
    assert "Content/ACESim/Environment/TestField/Meshes/SM_TestField_LandingPad.uasset" in verifier
    assert "Content/StarterContent/Materials/M_Basic_Wall.uasset" in verifier
    assert "version https://git-lfs.github.com/spec/v1" in verifier
    assert "git lfs pull" in verifier


def test_package_runtime_script_keeps_uat_logs_under_tmp() -> None:
    script = (UE_TOOL_DIR / "package_ue_runtime.sh").read_text(encoding="utf-8")

    assert 'UAT_LOG_DIR="${UAT_LOG_DIR:-${UE_ROOT}/logs/uat}"' in script
    assert "AUTOMATION_TOOL_DLL" in script
    assert "Missing precompiled AutomationTool:" in script
    assert 'export uebp_LogFolder="${UAT_LOG_DIR}"' in script


def test_package_runtime_script_forces_ddc_fallback_for_headless_cook() -> None:
    script = (UE_TOOL_DIR / "package_ue_runtime.sh").read_text(encoding="utf-8")

    assert "-DDC-ForceMemoryCache" in script
    assert "-ddc=NoZenLocalFallback" in script


def test_setup_script_allows_interactive_sudo_and_optional_gpu_check() -> None:
    script = (UE_TOOL_DIR / "setup_ubuntu_ue5.sh").read_text(encoding="utf-8")

    assert "sudo -v" in script
    assert "UE_REQUIRE_NVIDIA" in script
    assert 'if [ "${UE_REQUIRE_NVIDIA}" = "1" ]' in script


def test_runtime_smoke_script_documents_ue_and_bridge_gates() -> None:
    script = (UE_TOOL_DIR / "smoke_ue_bridge.sh").read_text(encoding="utf-8")

    assert "ACESimUEEditor Linux Development" in script
    assert "-run=SmokeTest" in script
    assert "RUN_LIVE_BRIDGE_SMOKE" in script
    assert "ACESim visual stream connected" in script
    assert "ACESim visual state applied" in script


def test_runtime_smoke_script_forces_memory_ddc_for_headless_editor() -> None:
    script = (UE_TOOL_DIR / "smoke_ue_bridge.sh").read_text(encoding="utf-8")

    assert script.count("-DDC-ForceMemoryCache") == 2
    assert script.count("-ddc=NoZenLocalFallback") == 2


def test_runtime_smoke_script_keeps_ue_user_paths_under_tmp() -> None:
    script = (UE_TOOL_DIR / "smoke_ue_bridge.sh").read_text(encoding="utf-8")

    assert "UE_SMOKE_HOME" in script
    assert "XDG_CONFIG_HOME" in script
    assert "XDG_CACHE_HOME" in script
    assert 'export HOME="${UE_SMOKE_HOME}"' in script


def test_runtime_smoke_script_waits_for_bridge_log_markers_before_exit() -> None:
    script = (UE_TOOL_DIR / "smoke_ue_bridge.sh").read_text(encoding="utf-8")

    assert "BridgePid" in script
    assert "BridgeDeadline" in script
    assert "BRIDGE_OUTPUT_LOG" in script
    assert "ACESim visual stream connected" in script
    assert "ACESim visual state applied" in script
    assert "-ExecCmds=Quit" not in script


def test_visual_runtime_verifier_launches_runtime_and_checks_screenshot() -> None:
    script = (UE_TOOL_DIR / "verify_ue_runtime_visual.py").read_text(encoding="utf-8")

    assert "ACESim vehicle visible in visual smoke frame" in script
    assert "ACESim outdoor test field spawned" in script
    assert "ACESim player controller ready" in script
    assert "ACESim camera tick drag started" in script
    assert "ACESim camera wheel zoom applied" in script
    assert "visual-checks" in script
    assert "visual_report.json" in script
    assert "HighResShot" in script
    assert "RenderOffScreen" in script
    assert 'default=os.environ.get("ACESIM_UE_RENDER_PRESET", "lumen")' in script
    assert "No available video device" in script
    assert "log_path.unlink()" in script
    assert "ImageStat" in script
    assert "VISUAL_CHECK_VIEWS" in script
    assert "EDITOR_LIGHTING_CHECK_VIEWS" in script
    assert "acesim_ue_visual_shadow_check" in script
    assert "acesim_ue_visual_pad_top" in script
    assert "acesim_ue_visual_low_oblique" in script
    assert "acesim_ue_visual_vehicle_close" in script
    assert "LinuxEditor" in script
    assert 'return ",".join(commands)' in script
    assert "min_required=1" in script
    assert "MIN_REQUIRED_SCREENSHOT_COUNT" in script
    assert "ACESimGameViewportClient not installed" in script
    assert "UE runtime is still loading the Unreal template map" in script
    assert "--ue-mode" in script
    assert '"editor"' in script
    assert "UnrealEditor" in script
    assert "lighting_report.json" in script
    assert "shadow_contrast < 0.05" in script
    assert "shadow_dark_pixel_ratio < 0.005" in script
    assert "golden_hour" in script
    assert "mythic_forest_day" in script
    assert "ACESim cinematic outdoor lighting ready" in script
    assert "validate_packaged_environment_assets" not in script
    assert "validate_packaged_airport_assets" not in script
    assert "airport_manifest.json" not in script
    assert "heliport_manifest.json" not in script


def test_editor_lighting_verifier_uses_single_shadow_check_screenshot() -> None:
    verifier_path = UE_TOOL_DIR / "verify_ue_runtime_visual.py"
    spec = importlib.util.spec_from_file_location("verify_ue_runtime_visual", verifier_path)
    assert spec is not None
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)

    command = module._editor_command(
        Path("/UE/UnrealEditor"),
        Path("/ACESimUE/ACESimUE.uproject"),
        "lumen",
        "cinematic_day",
    )
    exec_cmds = next(arg for arg in command if arg.startswith("-ExecCmds="))

    assert "acesim_ue_visual_shadow_check.png" in exec_cmds
    assert "acesim_ue_visual_field_wide.png" not in exec_cmds


def test_editor_lighting_verifier_waits_for_editor_shadow_view(tmp_path: Path) -> None:
    verifier_path = UE_TOOL_DIR / "verify_ue_runtime_visual.py"
    spec = importlib.util.spec_from_file_location("verify_ue_runtime_visual", verifier_path)
    assert spec is not None
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)

    screenshot = tmp_path / "acesim_ue_visual_shadow_check.png"
    screenshot.write_bytes(b"not-a-real-png")

    screenshots = module._wait_for_visual_check_screenshots(
        tmp_path,
        screenshot.stat().st_mtime,
        0.1,
        min_required=1,
        check_views=module.EDITOR_LIGHTING_CHECK_VIEWS,
        required_views=module.EDITOR_LIGHTING_CHECK_VIEWS,
    )

    assert screenshots == [screenshot]


def test_visual_runtime_verifier_accepts_realistic_default_cinematic_shadow_check() -> None:
    verifier_path = UE_TOOL_DIR / "verify_ue_runtime_visual.py"
    spec = importlib.util.spec_from_file_location("verify_ue_runtime_visual", verifier_path)
    assert spec is not None
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)

    realistic_shadow_stat = module.ImageStat(
        path=Path("realistic_shadow.png"),
        size=(1280, 720),
        mean_luma=88.7,
        p50_luma=90,
        p95_luma=122,
        color_span=161,
        dark_ratio=0.0,
        highlight_clip_ratio=0.0,
        green_ratio=0.145,
        concrete_ratio=0.055,
        sky_mean_rgb=(45.0, 62.0, 72.0),
        sky_luma=72.7,
        sky_saturation=0.20,
        sky_blue_orange_ratio=1.49,
        sky_blue_dominance=0.106,
        scene_blue_warm_ratio=1.18,
        ground_blue_warm_ratio=1.20,
        ground_blue_green_ratio=0.95,
        shadow_contrast=0.055,
        shadow_dark_pixel_ratio=0.0,
        shadow_roi_luma_p10=75,
        lit_reference_luma_p50=89,
    )

    module._validate_lighting_image_stat(realistic_shadow_stat, "cinematic_day")


def test_visual_runtime_verifier_rejects_weak_default_cinematic_shadow() -> None:
    verifier_path = UE_TOOL_DIR / "verify_ue_runtime_visual.py"
    spec = importlib.util.spec_from_file_location("verify_ue_runtime_visual", verifier_path)
    assert spec is not None
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)

    weak_shadow_stat = module.ImageStat(
        path=Path("weak_shadow.png"),
        size=(1280, 720),
        mean_luma=118.0,
        p50_luma=100,
        p95_luma=210,
        color_span=80,
        dark_ratio=0.04,
        highlight_clip_ratio=0.0,
        green_ratio=0.20,
        concrete_ratio=0.12,
        sky_mean_rgb=(120.0, 145.0, 190.0),
        sky_luma=128.0,
        sky_saturation=0.18,
        sky_blue_orange_ratio=1.45,
        sky_blue_dominance=0.12,
        scene_blue_warm_ratio=1.08,
        ground_blue_warm_ratio=1.02,
        ground_blue_green_ratio=0.92,
        shadow_contrast=0.03,
        shadow_dark_pixel_ratio=0.002,
        shadow_roi_luma_p10=95,
        lit_reference_luma_p50=108,
    )

    with pytest.raises(RuntimeError, match="weak aircraft shadow"):
        module._validate_lighting_image_stat(weak_shadow_stat, "cinematic_day")
