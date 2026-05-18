#!/usr/bin/env python3
"""Launch ACESimUE briefly and verify the outdoor scene renders non-empty pixels."""

from __future__ import annotations

import argparse
import json
import os
import re
import shutil
import subprocess
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Sequence

DEFAULT_UE_EXECUTABLE = Path("/tmp/ACESim-unreal/packages/ACESimUE-Linux/Linux/ACESimUE/Binaries/Linux/ACESimUE")
DEFAULT_OUTPUT_DIR = Path("/tmp/ACESim-unreal/visual-checks")
RENDER_PRESETS = ("performance", "lumen", "raytracing")
AIRPORT_STAGE_DIR = Path("Content/ACESim/Environment/Airport")
AIRPORT_ATTRIBUTION_STAGE_PATH = Path("Content/ACESim/Environment/Airport/ATTRIBUTION.txt")
AIRPORT_MANIFEST_STAGE_PATH = Path("Content/ACESim/Environment/Airport/airport_manifest.json")
AIRPORT_MODEL_STAGE_DIR = Path("Content/ACESim/Environment/Airport/Model")
HELIPORT_STAGE_DIR = Path("Content/ACESim/Environment/Heliport")
HELIPORT_ATTRIBUTION_STAGE_PATH = Path("Content/ACESim/Environment/Heliport/ATTRIBUTION.txt")
HELIPORT_MANIFEST_STAGE_PATH = Path("Content/ACESim/Environment/Heliport/heliport_manifest.json")
HELIPORT_MODEL_STAGE_DIR = Path("Content/ACESim/Environment/Heliport/Model")
MIN_REQUIRED_SCREENSHOT_COUNT = 2
COMMON_REQUIRED_LOG_MARKERS = (
    "ACESim player controller ready",
    "ACESim camera tick drag started",
    "ACESim camera wheel zoom applied",
    "ACESim real vehicle mesh loaded",
    "ACESim viewport input diagnostic",
    "ACESim viewport input diagnostic: installed=1 capture_mode=orbit",
    "ACESim vehicle visible in visual smoke frame",
)
AIRPORT_REQUIRED_LOG_MARKERS = (
    "ACESim airport assets loaded",
    "ACESim airport attribution loaded",
)
HELIPORT_REQUIRED_LOG_MARKERS = (
    "ACESim heliport assets loaded",
    "ACESim heliport attribution loaded",
)
FORBIDDEN_LOG_MARKERS = (
    "Failed to find object 'MaterialInterface",
    "Default Material will be used in game",
    "no StaticMesh assets under /Game/ACESim/Environment/Airport/Model",
    "no StaticMesh assets under /Game/ACESim/Environment/Heliport/Model",
    "ACESim airport mesh uses invalid/default material",
    "ACESim heliport mesh uses invalid/default material",
    "ACESim outdoor field asset missing",
    "missing bUsedWithInstancedStaticMeshes",
    "ACESimGameViewportClient not installed",
    "Template_Default",
    "ACESim offline test field meshes loaded",
)
VISUAL_CHECK_VIEWS = (
    "pad_top",
    "low_oblique",
    "vehicle_close",
    "heliport_wide",
)


@dataclass(frozen=True)
class ImageStat:
    path: Path
    size: tuple[int, int]
    mean_luma: float
    color_span: int
    green_ratio: float
    concrete_ratio: float


@dataclass(frozen=True)
class PackageAssetInventory:
    runtime_root: Path
    package_manifest: Path
    env_style: str
    environment_attribution: Path
    environment_manifest: Path
    environment_uasset_count: int

    @property
    def airport_attribution(self) -> Path:
        return self.environment_attribution

    @property
    def airport_manifest(self) -> Path:
        return self.environment_manifest

    @property
    def airport_uasset_count(self) -> int:
        return self.environment_uasset_count


def _is_packaged_linux_binary(path: Path) -> bool:
    return path.name == "ACESimUE" and path.parts[-3:] == ("Binaries", "Linux", "ACESimUE")


def _runtime_root_for(executable: Path) -> Path:
    if _is_packaged_linux_binary(executable):
        return executable.parents[2]
    return executable.parent


def _environment_stage_paths(env_style: str) -> tuple[Path, Path, Path, Path]:
    if env_style == "heliport":
        return (
            HELIPORT_STAGE_DIR,
            HELIPORT_ATTRIBUTION_STAGE_PATH,
            HELIPORT_MANIFEST_STAGE_PATH,
            HELIPORT_MODEL_STAGE_DIR,
        )
    if env_style == "airport":
        return (
            AIRPORT_STAGE_DIR,
            AIRPORT_ATTRIBUTION_STAGE_PATH,
            AIRPORT_MANIFEST_STAGE_PATH,
            AIRPORT_MODEL_STAGE_DIR,
        )
    raise RuntimeError(f"Packaged ACESim runtime uses unsupported env_style={env_style}")


def validate_packaged_environment_assets(executable: Path) -> PackageAssetInventory:
    """Fail before launching UE when the packaged environment scene is not staged."""
    runtime_root = _runtime_root_for(executable)
    package_manifest = runtime_root / "ACESimUE_PACKAGE_MANIFEST.json"
    if not package_manifest.is_file():
        raise RuntimeError(f"Packaged ACESim environment assets are incomplete before launching UE: {package_manifest}")
    manifest_payload = json.loads(package_manifest.read_text(encoding="utf-8"))
    env_style = str(manifest_payload.get("env_style") or "heliport").lower()
    stage_dir, attribution_stage, manifest_stage, model_stage = _environment_stage_paths(env_style)
    runtime_root / stage_dir
    environment_attribution = runtime_root / attribution_stage
    environment_manifest = runtime_root / manifest_stage
    model_dir = runtime_root / model_stage
    missing = [
        str(path) for path in (package_manifest, environment_attribution, environment_manifest) if not path.is_file()
    ]
    environment_assets = sorted(model_dir.rglob("*.uasset")) if model_dir.is_dir() else []
    if not environment_assets:
        missing.append(str(model_dir / "*.uasset"))
    if missing:
        raise RuntimeError(
            f"Packaged ACESim {env_style} assets are incomplete before launching UE: " + ", ".join(missing)
        )
    if manifest_payload.get("env_style") != env_style:
        raise RuntimeError(f"Packaged ACESim runtime is not a valid {env_style} package: {package_manifest}")
    return PackageAssetInventory(
        runtime_root=runtime_root,
        package_manifest=package_manifest,
        env_style=env_style,
        environment_attribution=environment_attribution,
        environment_manifest=environment_manifest,
        environment_uasset_count=len(environment_assets),
    )


def validate_packaged_airport_assets(executable: Path) -> PackageAssetInventory:
    """Backwards-compatible wrapper used by older tests/importers."""
    return validate_packaged_environment_assets(executable)


def _screenshot_dir_for(executable: Path) -> Path:
    if _is_packaged_linux_binary(executable):
        return executable.parents[2] / "Saved" / "Screenshots" / "Linux"
    return executable.parent / "Saved" / "Screenshots" / "Linux"


def _log_path_for(executable: Path) -> Path:
    if _is_packaged_linux_binary(executable):
        return executable.parents[2] / "Saved" / "Logs" / "ACESimUE.log"
    return executable.parent / "Saved" / "Logs" / "ACESimUE.log"


def _exec_cmds_for_visual_checks() -> str:
    commands = [
        "HighResShot 1280x720 filename=acesim_ue_visual_pad_top.png",
        "HighResShot 1280x720 filename=acesim_ue_visual_low_oblique.png",
        "HighResShot 1280x720 filename=acesim_ue_visual_vehicle_close.png",
        "HighResShot 1280x720 filename=acesim_ue_visual_heliport_wide.png",
    ]
    return ";".join(commands)


def _runtime_command(executable: Path, render_preset: str) -> list[str]:
    command = [
        str(executable),
    ]
    if _is_packaged_linux_binary(executable):
        command.append("ACESimUE")
    command.extend(
        [
            "-Windowed",
            "-ForceRes",
            "-ResX=1280",
            "-ResY=720",
            "-DefaultViewportMouseCaptureMode=CaptureDuringMouseDown",
            "-ACESimVisualSmoke",
            f"-ACESimRenderPreset={render_preset}",
            f"-ExecCmds={_exec_cmds_for_visual_checks()}",
        ]
    )
    if render_preset == "raytracing":
        command.append("-vulkan")
    return command


def _required_log_markers(env_style: str) -> tuple[str, ...]:
    if env_style == "heliport":
        return (*HELIPORT_REQUIRED_LOG_MARKERS, *COMMON_REQUIRED_LOG_MARKERS)
    if env_style == "airport":
        return (*AIRPORT_REQUIRED_LOG_MARKERS, *COMMON_REQUIRED_LOG_MARKERS)
    return COMMON_REQUIRED_LOG_MARKERS


def _runtime_log_contains_startup_failure(payload: str) -> str | None:
    if "No available video device" in payload or "InitSDL() failed" in payload:
        return (
            "UE visual smoke could not create a Linux window. "
            "Run with --offscreen in headless/remote shells, or run from a desktop session with DISPLAY set."
        )
    if "Could not initialize UDEV" in payload or "Vulkan device could not be created" in payload:
        return (
            "UE visual smoke could not initialize SDL/Vulkan in this shell. "
            "This usually means the sandbox or remote session cannot access the GPU/display device."
        )
    if "Template_Default" in payload:
        return "UE runtime is still loading the Unreal template map instead of the ACESim empty runtime map."
    if "ACESim offline test field meshes loaded" in payload:
        return "UE runtime is still using the lightweight test-field fallback instead of the airport scene."
    for marker in FORBIDDEN_LOG_MARKERS:
        if marker in payload:
            return f"UE visual smoke log contains forbidden marker: {marker}"
    if "WorldGridMaterial" in payload and ("ACESim airport" in payload or "ACESim heliport" in payload):
        return "UE environment import/runtime log references WorldGridMaterial."
    return None


def _wait_for_log_markers(log_path: Path, timeout_sec: float, env_style: str) -> str:
    deadline = time.monotonic() + timeout_sec
    payload = ""
    required_markers = _required_log_markers(env_style)
    while time.monotonic() < deadline:
        if log_path.is_file():
            payload = log_path.read_text(encoding="utf-8", errors="ignore")
            startup_failure = _runtime_log_contains_startup_failure(payload)
            if startup_failure is not None:
                raise RuntimeError(startup_failure)
            if all(marker in payload for marker in required_markers):
                return payload
        time.sleep(0.25)
    missing = [marker for marker in required_markers if marker not in payload]
    raise RuntimeError(f"UE visual smoke log did not contain required markers: {missing}")


def _latest_screenshot(screenshot_dir: Path, started_at: float, timeout_sec: float) -> Path:
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        candidates = [path for path in screenshot_dir.glob("*.png") if path.stat().st_mtime >= started_at - 1.0]
        if candidates:
            return max(candidates, key=lambda path: path.stat().st_mtime)
        time.sleep(0.25)
    raise RuntimeError(f"UE visual smoke did not produce a screenshot under {screenshot_dir}")


def _wait_for_visual_check_screenshots(screenshot_dir: Path, started_at: float, timeout_sec: float) -> list[Path]:
    deadline = time.monotonic() + timeout_sec
    expected = [screenshot_dir / f"acesim_ue_visual_{view}.png" for view in VISUAL_CHECK_VIEWS]
    required = [
        screenshot_dir / "acesim_ue_visual_vehicle_close.png",
        screenshot_dir / "acesim_ue_visual_heliport_wide.png",
    ]
    while time.monotonic() < deadline:
        existing = [path for path in expected if path.is_file() and path.stat().st_mtime >= started_at - 1.0]
        required_existing = [path for path in required if path.is_file() and path.stat().st_mtime >= started_at - 1.0]
        if len(required_existing) == len(required) and len(existing) >= MIN_REQUIRED_SCREENSHOT_COUNT:
            return existing
        time.sleep(0.25)
    recent = [path for path in screenshot_dir.glob("*.png") if path.stat().st_mtime >= started_at - 1.0]
    missing = [str(path) for path in expected if not path.is_file()]
    produced = [path.name for path in screenshot_dir.glob("*.png")]
    raise RuntimeError(
        f"UE visual smoke did not produce expected screenshots: {missing}; " f"recent={len(recent)} produced={produced}"
    )


def _read_png_stat(path: Path) -> ImageStat:
    try:
        from PIL import Image
    except ImportError as exc:  # pragma: no cover - depends on optional local tooling
        raise RuntimeError("Pillow is required for UE visual screenshot verification") from exc

    with Image.open(path) as image:
        rgb = image.convert("RGB")
        pixels = list(rgb.getdata())
        width, height = rgb.size
    if not pixels:
        raise RuntimeError(f"Screenshot has no pixels: {path}")

    lumas = [(red * 299 + green * 587 + blue * 114) // 1000 for red, green, blue in pixels]
    mean_luma = sum(lumas) / len(lumas)
    color_span = max(lumas) - min(lumas)
    greenish = sum(1 for red, green, blue in pixels if green > red * 1.05 and green > blue * 1.05)
    concrete = sum(
        1 for red, green, blue in pixels if abs(red - green) < 18 and abs(green - blue) < 18 and 55 < red < 210
    )
    return ImageStat(
        path=path,
        size=(width, height),
        mean_luma=mean_luma,
        color_span=color_span,
        green_ratio=greenish / len(pixels),
        concrete_ratio=concrete / len(pixels),
    )


def _validate_image_stat(stat: ImageStat) -> None:
    if stat.size[0] < 640 or stat.size[1] < 360:
        raise RuntimeError(f"Screenshot resolution is unexpectedly small: {stat.size}")
    if stat.mean_luma < 12.0 or stat.mean_luma > 245.0:
        raise RuntimeError(f"Screenshot looks blank or blown out: mean_luma={stat.mean_luma:.1f}")
    if stat.color_span < 35:
        raise RuntimeError(f"Screenshot has too little contrast: color_span={stat.color_span}")
    if stat.green_ratio < 0.03 and stat.concrete_ratio < 0.03:
        raise RuntimeError(
            "Screenshot does not show enough grass/concrete-like pixels: "
            f"green_ratio={stat.green_ratio:.3f} concrete_ratio={stat.concrete_ratio:.3f}"
        )


def _raytracing_is_enabled(log_payload: str) -> bool:
    if re.search(r"Ray tracing is disabled", log_payload, re.IGNORECASE):
        return False
    return "r.RayTracing=1" in log_payload or "Ray tracing is enabled" in log_payload


def _append_offscreen_args(command: list[str]) -> list[str]:
    return [*command, "-RenderOffScreen"]


def _write_visual_report(
    *,
    output_dir: Path,
    log_path: Path,
    screenshots: Sequence[Path],
    stats: Sequence[ImageStat],
    asset_inventory: PackageAssetInventory,
    render_preset: str,
) -> Path:
    report_path = output_dir / "visual_report.json"
    report = {
        "render_preset": render_preset,
        "log_path": str(log_path),
        "screenshots": [str(path) for path in screenshots],
        "asset_inventory": {
            "runtime_root": str(asset_inventory.runtime_root),
            "package_manifest": str(asset_inventory.package_manifest),
            "env_style": asset_inventory.env_style,
            "environment_attribution": str(asset_inventory.environment_attribution),
            "environment_manifest": str(asset_inventory.environment_manifest),
            "environment_uasset_count": asset_inventory.environment_uasset_count,
            "airport_attribution": str(asset_inventory.airport_attribution),
            "airport_manifest": str(asset_inventory.airport_manifest),
            "airport_uasset_count": asset_inventory.airport_uasset_count,
        },
        "vehicle_visibility": {
            "visible_marker": True,
            "vehicle_screen_bounds": "logged-by-runtime-marker",
        },
        "package_manifest": json.loads(asset_inventory.package_manifest.read_text(encoding="utf-8")),
        "image_stats": [
            {
                "path": str(stat.path),
                "size": list(stat.size),
                "mean_luma": stat.mean_luma,
                "color_span": stat.color_span,
                "green_ratio": stat.green_ratio,
                "concrete_ratio": stat.concrete_ratio,
            }
            for stat in stats
        ],
    }
    report_path.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return report_path


def verify_runtime_visual(
    *,
    executable: Path,
    render_preset: str,
    output_dir: Path,
    timeout_sec: float,
    offscreen: bool = False,
) -> ImageStat:
    if render_preset not in RENDER_PRESETS:
        raise ValueError(f"Unsupported render preset: {render_preset}")
    if not executable.is_file():
        raise FileNotFoundError(f"ACESimUE executable not found: {executable}")

    asset_inventory = validate_packaged_environment_assets(executable)
    screenshot_dir = _screenshot_dir_for(executable)
    log_path = _log_path_for(executable)
    screenshot_dir.mkdir(parents=True, exist_ok=True)
    output_dir.mkdir(parents=True, exist_ok=True)
    if log_path.exists():
        log_path.unlink()
    command = _runtime_command(executable, render_preset)
    if offscreen:
        command = _append_offscreen_args(command)
    started_at = time.time()
    process = subprocess.Popen(command)
    try:
        log_payload = _wait_for_log_markers(log_path, timeout_sec, asset_inventory.env_style)
        if render_preset == "raytracing" and not _raytracing_is_enabled(log_payload):
            raise RuntimeError("Ray tracing preset requested, but UE log says ray tracing is disabled")
        screenshots = _wait_for_visual_check_screenshots(screenshot_dir, started_at, timeout_sec)
        stats: list[ImageStat] = []
        for screenshot in screenshots:
            destination = output_dir / screenshot.name
            shutil.copy2(screenshot, destination)
            stat = _read_png_stat(destination)
            _validate_image_stat(stat)
            stats.append(stat)
        report_path = _write_visual_report(
            output_dir=output_dir,
            log_path=log_path,
            screenshots=[output_dir / screenshot.name for screenshot in screenshots],
            stats=stats,
            asset_inventory=asset_inventory,
            render_preset=render_preset,
        )
        print(f"UE visual smoke report: {report_path}", flush=True)
        return stats[0]
    finally:
        if process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()


def main(argv: Sequence[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description="Verify the packaged ACESimUE outdoor scene with a screenshot.")
    parser.add_argument("--ue-executable", type=Path, default=DEFAULT_UE_EXECUTABLE)
    parser.add_argument(
        "--render-preset",
        choices=RENDER_PRESETS,
        default=os.environ.get("ACESIM_UE_RENDER_PRESET", "performance"),
    )
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    parser.add_argument("--timeout-sec", type=float, default=20.0)
    parser.add_argument(
        "--offscreen",
        action="store_true",
        help="Use UE's -RenderOffScreen path for headless Linux screenshot verification.",
    )
    args = parser.parse_args(argv)

    stat = verify_runtime_visual(
        executable=args.ue_executable.expanduser().resolve(),
        render_preset=args.render_preset,
        output_dir=args.output_dir.expanduser().resolve(),
        timeout_sec=args.timeout_sec,
        offscreen=args.offscreen,
    )
    print(
        "UE visual smoke passed: "
        f"path={stat.path} size={stat.size[0]}x{stat.size[1]} "
        f"mean_luma={stat.mean_luma:.1f} color_span={stat.color_span} "
        f"green_ratio={stat.green_ratio:.3f} concrete_ratio={stat.concrete_ratio:.3f}",
        flush=True,
    )


if __name__ == "__main__":
    main()
