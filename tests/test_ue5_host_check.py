import os
import stat
import subprocess
import textwrap
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def _write_executable(path: Path, content: str) -> None:
    path.write_text(textwrap.dedent(content).lstrip(), encoding="utf-8")
    path.chmod(path.stat().st_mode | stat.S_IXUSR)


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
        ["bash", str(ROOT / "acesim" / "tools" / "ue5" / "check_ubuntu_ue5_host.sh")],
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


def test_host_check_rejects_empty_ls_remote_output(tmp_path: Path) -> None:
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
        if [ "$1" = "ls-remote" ]; then
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
        ["bash", str(ROOT / "acesim" / "tools" / "ue5" / "check_ubuntu_ue5_host.sh")],
        check=True,
        capture_output=True,
        text=True,
        env=env,
    )

    assert "repo_access=blocked" in result.stdout


def test_setup_script_builds_generated_project_with_no_uba() -> None:
    script = (ROOT / "acesim" / "tools" / "ue5" / "setup_ubuntu_ue5.sh").read_text(encoding="utf-8")

    assert "ACESIM_UE_SKIP_REGENERATE" in script
    assert '--project-root "${UE_PROJECT_DIR}"' in script
    assert "--overwrite" in script
    assert 'Build.sh" ShaderCompileWorker Linux Development' in script
    assert 'Build.sh" ACESimUEEditor Linux Development' in script
    assert '-Project="${UE_PROJECT_DIR}/ACESimUE.uproject"' in script
    assert "-NoHotReloadFromIDE" in script
    assert script.count("-NoUBA") >= 2


def test_package_runtime_script_builds_and_archives_linux_package() -> None:
    script = (ROOT / "acesim" / "tools" / "ue5" / "package_ue_runtime.sh").read_text(encoding="utf-8")

    assert "ACESIM_UE_SKIP_REGENERATE" in script
    assert '--project-root "${UE_PROJECT_DIR}"' in script
    assert '--render-preset "${ACESIM_UE_RENDER_PRESET}"' in script
    assert "--overwrite" in script
    assert 'Build.sh" ShaderCompileWorker Linux Development' in script
    assert 'Build.sh" ACESimUEEditor Linux Development' in script
    assert "RunUAT.sh" in script
    assert "-nocompileuat" in script
    assert "-NoUBA" in script
    assert "BuildCookRun" in script
    assert "-archivedirectory=${UE_PACKAGE_DIR}" in script
    assert "find_acesimue_executable" in script
    assert "*/ACESimUE/Binaries/Linux/ACESimUE" in script
    assert "export_mjcf_visual_assets.py" in script
    assert "import_acesim_assets.py" in script
    assert "prepare_ue_environment_assets.py" in script
    assert "prepare_ue_airport_assets.py" in script
    assert 'ACESIM_UE_ENV_STYLE="${ACESIM_UE_ENV_STYLE:-heliport}"' in script
    assert 'ACESIM_UE_HELIPORT_MODEL_UID="${ACESIM_UE_HELIPORT_MODEL_UID:-5bc89e02a58b4ebca7404e5e35da2481}"' in script
    assert 'ACESIM_UE_HELIPORT_PACK_ROOT="${ACESIM_UE_HELIPORT_PACK_ROOT:-${UE_ROOT}/assets/heliport_pack}"' in script
    assert 'ACESIM_UE_AIRPORT_MODEL_UID="${ACESIM_UE_AIRPORT_MODEL_UID:-c90d33875c824a1884a1dc936db405a3}"' in script
    assert 'ACESIM_UE_AIRPORT_PACK_ROOT="${ACESIM_UE_AIRPORT_PACK_ROOT:-${UE_ROOT}/assets/airport_pack}"' in script
    assert "SKETCHFAB_API_TOKEN" in script
    assert "import_acesim_heliport_assets.py" in script
    assert "import_acesim_airport_assets.py" in script
    assert "fix_acesim_environment_materials.py" in script
    assert "ACESIM_UE_RENDER_PRESET" in script
    assert "ACESimUE executable:" in script


def test_package_runtime_script_fails_before_build_when_environment_assets_are_missing() -> None:
    script = (ROOT / "acesim" / "tools" / "ue5" / "package_ue_runtime.sh").read_text(encoding="utf-8")

    build_index = script.index('echo "[1/4] Building ShaderCompileWorker"')
    env_preflight_index = script.index("Preflighting ACESim UE environment asset cache")
    stale_package_index = script.index("isolate_stale_environment_package")
    assert stale_package_index < env_preflight_index
    assert env_preflight_index < build_index
    assert "SKETCHFAB_API_TOKEN is required before building the heliport runtime" in script
    assert "heliport_asset_manifest.json" in script
    assert "airport_asset_manifest.json" in script
    assert "gltf" in script
    assert "ACESIM_UE_ENV_STYLE=testfield" in script
    assert "stale-environment-packages" in script
    assert "Refusing to leave a stale heliport runtime package active" in script


def test_package_runtime_script_validates_environment_import_outputs_before_cook() -> None:
    script = (ROOT / "acesim" / "tools" / "ue5" / "package_ue_runtime.sh").read_text(encoding="utf-8")

    assert "validate_environment_runtime_assets" in script
    assert "heliport_import_validation.json" in script
    assert "airport_import_validation.json" in script
    assert "invalid_material_slot_count" in script
    assert "default_material_slot_count" in script
    assert "Heliport import produced no cooked source assets" in script
    assert "Content/ACESim/Environment/Airport/ATTRIBUTION.txt" in script
    assert "Content/ACESim/Environment/Heliport/ATTRIBUTION.txt" in script
    assert "Content/ACESim/Environment/Airport/airport_manifest.json" in script
    assert "Content/ACESim/Environment/Heliport/heliport_manifest.json" in script
    assert 'find "${UE_PROJECT_DIR}/Content/ACESim/Environment/Airport/Model" -name "*.uasset"' in script
    assert 'find "${UE_PROJECT_DIR}/Content/ACESim/Environment/Heliport/Model" -name "*.uasset"' in script
    assert script.index("validate_environment_runtime_assets") < script.index(
        'echo "[4/5] Packaging ACESimUE Linux runtime"'
    )


def test_package_runtime_script_validates_archived_environment_package_and_writes_marker() -> None:
    script = (ROOT / "acesim" / "tools" / "ue5" / "package_ue_runtime.sh").read_text(encoding="utf-8")

    assert "write_package_marker" in script
    assert "validate_packaged_environment_runtime" in script
    assert "ACESimUE_PACKAGE_MANIFEST.json" in script
    assert '"env_style": "${ACESIM_UE_ENV_STYLE}"' in script
    assert '"heliport_model_uid": "${ACESIM_UE_HELIPORT_MODEL_UID}"' in script
    assert '"airport_model_uid": "${ACESIM_UE_AIRPORT_MODEL_UID}"' in script
    assert "Packaged heliport runtime is missing attribution" in script
    assert "Packaged heliport runtime is missing manifest" in script
    assert "Packaged heliport runtime has no staged heliport uasset" in script
    assert script.rindex("validate_packaged_environment_runtime") > script.index("BuildCookRun")
    assert script.rindex("write_package_marker") > script.rindex("validate_packaged_environment_runtime")


def test_package_runtime_script_runs_visual_verifier_unless_explicitly_skipped() -> None:
    script = (ROOT / "acesim" / "tools" / "ue5" / "package_ue_runtime.sh").read_text(encoding="utf-8")

    assert 'ACESIM_UE_SKIP_VISUAL_VERIFY="${ACESIM_UE_SKIP_VISUAL_VERIFY:-0}"' in script
    assert 'ACESIM_UE_VISUAL_VERIFY_OFFSCREEN="${ACESIM_UE_VISUAL_VERIFY_OFFSCREEN:-auto}"' in script
    assert "verify_ue_runtime_visual.py" in script
    assert "--render-preset" in script
    assert "--ue-executable" in script
    assert "ACESIM_UE_SKIP_VISUAL_VERIFY=1" in script
    assert "visual_verify_args" in script
    assert "--offscreen" in script
    assert "DISPLAY" in script
    assert "WAYLAND_DISPLAY" in script
    assert script.index("verify_ue_runtime_visual.py") > script.index("write_package_marker")


def test_package_runtime_script_preflights_runtime_dependencies() -> None:
    script = (ROOT / "acesim" / "tools" / "ue5" / "package_ue_runtime.sh").read_text(encoding="utf-8")

    assert "Missing UnrealEditor:" in script
    assert "Missing UnrealEditor-Cmd:" in script
    assert "Missing ShaderCompileWorker:" in script
    assert "Missing ACESim UE project:" in script
    assert "Missing ACESimBridge runtime plugin" in script
    assert "DefaultEngine.ini still references OpenWorld" in script
    assert "DDC path is not writable" in script
    assert "Available space under" in script


def test_package_runtime_script_keeps_uat_logs_under_tmp() -> None:
    script = (ROOT / "acesim" / "tools" / "ue5" / "package_ue_runtime.sh").read_text(encoding="utf-8")

    assert 'UAT_LOG_DIR="${UAT_LOG_DIR:-${UE_ROOT}/logs/uat}"' in script
    assert "AUTOMATION_TOOL_DLL" in script
    assert "Missing precompiled AutomationTool:" in script
    assert 'export uebp_LogFolder="${UAT_LOG_DIR}"' in script


def test_package_runtime_script_forces_ddc_fallback_for_headless_import_and_cook() -> None:
    script = (ROOT / "acesim" / "tools" / "ue5" / "package_ue_runtime.sh").read_text(encoding="utf-8")

    assert script.count("-DDC-ForceMemoryCache") >= 2
    assert script.count("-ddc=NoZenLocalFallback") >= 2


def test_package_runtime_script_keeps_testfield_as_explicit_debug_fallback() -> None:
    script = (ROOT / "acesim" / "tools" / "ue5" / "package_ue_runtime.sh").read_text(encoding="utf-8")

    assert 'if [ "${ACESIM_UE_ENV_STYLE}" = "heliport" ]' in script
    assert 'elif [ "${ACESIM_UE_ENV_STYLE}" = "airport" ]' in script
    assert 'elif [ "${ACESIM_UE_ENV_STYLE}" = "testfield" ]' in script
    assert "Unsupported ACESIM_UE_ENV_STYLE" in script
    assert "only heliport, airport, and testfield are implemented" in script


def test_setup_script_allows_interactive_sudo_and_optional_gpu_check() -> None:
    script = (ROOT / "acesim" / "tools" / "ue5" / "setup_ubuntu_ue5.sh").read_text(encoding="utf-8")

    assert "sudo -v" in script
    assert "UE_REQUIRE_NVIDIA" in script
    assert 'if [ "${UE_REQUIRE_NVIDIA}" = "1" ]' in script


def test_runtime_smoke_script_documents_ue_and_bridge_gates() -> None:
    script = (ROOT / "acesim" / "tools" / "ue5" / "smoke_ue_bridge.sh").read_text(encoding="utf-8")

    assert "ACESimUEEditor Linux Development" in script
    assert "-run=SmokeTest" in script
    assert "RUN_LIVE_BRIDGE_SMOKE" in script
    assert "ACESim visual stream connected" in script
    assert "ACESim visual state applied" in script


def test_runtime_smoke_script_forces_memory_ddc_for_headless_editor() -> None:
    script = (ROOT / "acesim" / "tools" / "ue5" / "smoke_ue_bridge.sh").read_text(encoding="utf-8")

    assert script.count("-DDC-ForceMemoryCache") == 2
    assert script.count("-ddc=NoZenLocalFallback") == 2


def test_runtime_smoke_script_keeps_ue_user_paths_under_tmp() -> None:
    script = (ROOT / "acesim" / "tools" / "ue5" / "smoke_ue_bridge.sh").read_text(encoding="utf-8")

    assert "UE_SMOKE_HOME" in script
    assert "XDG_CONFIG_HOME" in script
    assert "XDG_CACHE_HOME" in script
    assert 'export HOME="${UE_SMOKE_HOME}"' in script


def test_runtime_smoke_script_waits_for_bridge_log_markers_before_exit() -> None:
    script = (ROOT / "acesim" / "tools" / "ue5" / "smoke_ue_bridge.sh").read_text(encoding="utf-8")

    assert "BridgePid" in script
    assert "BridgeDeadline" in script
    assert "BRIDGE_OUTPUT_LOG" in script
    assert "ACESim visual stream connected" in script
    assert "ACESim visual state applied" in script
    assert "-ExecCmds=Quit" not in script


def test_visual_runtime_verifier_launches_runtime_and_checks_screenshot() -> None:
    script = (ROOT / "acesim" / "tools" / "ue5" / "verify_ue_runtime_visual.py").read_text(encoding="utf-8")

    assert "ACESim airport assets loaded" in script
    assert "ACESim airport attribution loaded" in script
    assert "ACESim heliport assets loaded" in script
    assert "ACESim heliport attribution loaded" in script
    assert "ACESim vehicle visible in visual smoke frame" in script
    assert "ACESim outdoor field materials loaded" not in script
    assert "ACESim player controller ready" in script
    assert "ACESim camera tick drag started" in script
    assert "ACESim camera wheel zoom applied" in script
    assert "visual-checks" in script
    assert "visual_report.json" in script
    assert "HighResShot" in script
    assert "RenderOffScreen" in script
    assert "No available video device" in script
    assert "log_path.unlink()" in script
    assert "ImageStat" in script
    assert "VISUAL_CHECK_VIEWS" in script
    assert "acesim_ue_visual_pad_top" in script
    assert "acesim_ue_visual_low_oblique" in script
    assert "acesim_ue_visual_vehicle_close" in script
    assert "acesim_ue_visual_heliport_wide" in script
    assert "MIN_REQUIRED_SCREENSHOT_COUNT" in script
    assert "Failed to find object 'MaterialInterface" in script
    assert "Default Material will be used in game" in script
    assert "WorldGridMaterial" in script
    assert "no StaticMesh assets under /Game/ACESim/Environment/Airport/Model" in script
    assert "no StaticMesh assets under /Game/ACESim/Environment/Heliport/Model" in script
    assert "ACESim outdoor field asset missing" in script
    assert "validate_packaged_airport_assets" in script
    assert "missing bUsedWithInstancedStaticMeshes" in script
    assert "ACESimGameViewportClient not installed" in script
    assert "ACESim real vehicle mesh loaded" in script
    assert "ACESim offline test field meshes loaded" in script
    assert "UE runtime is still using the lightweight test-field fallback" in script
    assert "Template_Default" in script
    assert "UE runtime is still loading the Unreal template map" in script


def test_visual_runtime_verifier_checks_packaged_airport_assets_before_launch() -> None:
    script = (ROOT / "acesim" / "tools" / "ue5" / "verify_ue_runtime_visual.py").read_text(encoding="utf-8")

    assert "validate_packaged_airport_assets" in script
    assert "validate_packaged_environment_assets" in script
    assert "ACESim/Environment/Heliport/ATTRIBUTION.txt" in script
    assert "ACESim/Environment/Heliport/heliport_manifest.json" in script
    assert "ACESim/Environment/Heliport/Model" in script
    assert "ACESim/Environment/Airport/ATTRIBUTION.txt" in script
    assert "ACESim/Environment/Airport/airport_manifest.json" in script
    assert "ACESim/Environment/Airport/Model" in script
    assert "before launching UE" in script
    assert "asset_inventory" in script
    assert "package_manifest" in script


def test_airport_asset_preparer_documents_sketchfab_cache_and_attribution() -> None:
    script = (ROOT / "acesim" / "tools" / "ue5" / "prepare_ue_airport_assets.py").read_text(encoding="utf-8")

    assert "SKETCHFAB_API_TOKEN" in script
    assert "c90d33875c824a1884a1dc936db405a3" in script
    assert "https://api.sketchfab.com/v3/models" in script
    assert "airport_asset_manifest.json" in script
    assert "ATTRIBUTION.txt" in script
    assert "CC Attribution" in script
    assert "triangle_budget" in script
    assert "import_acesim_airport_assets.py" in script


def test_environment_asset_preparer_writes_material_usage_fix_script() -> None:
    script = (ROOT / "acesim" / "tools" / "ue5" / "prepare_ue_environment_assets.py").read_text(encoding="utf-8")

    assert "fix_acesim_environment_materials.py" in script
    assert "used_with_instanced_static_meshes" in script
    assert "M_Concrete_Poured" in script
    assert "M_Rock" in script
    assert "M_Bush" in script
    assert "generate_acesim_testfield_meshes.py" in script
    assert "SM_TestField_Runway.obj" in script
    assert "T_Metal_Rust_N.uasset" in script


def test_testfield_mesh_generator_writes_uvs_for_material_scale() -> None:
    script = (ROOT / "acesim" / "tools" / "ue5" / "generate_acesim_testfield_meshes.py").read_text(encoding="utf-8")

    assert "uv_scale_cm" in script
    assert "vt " in script
    assert "mtllib" in script
    assert "usemtl" in script
    assert "ACESimTestFieldSurface" in script
    assert "_ring_mesh" in script
    assert "9000.0, 6800.0" in script
    assert "{vertex_index}/{uv_index}" in script
    assert "SM_TestField_Runway.obj" in script
    assert "SM_TestField_RunwayCenterline.obj" in script
    assert "SM_TestField_LandingPadMarkings.obj" in script
