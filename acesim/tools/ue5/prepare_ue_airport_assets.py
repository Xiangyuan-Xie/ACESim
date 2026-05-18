#!/usr/bin/env python3
"""Download/cache Sketchfab environment models and prepare UE import scripts."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import shutil
import time
import urllib.request
import zipfile
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable

DEFAULT_AIRPORT_MODEL_UID = "c90d33875c824a1884a1dc936db405a3"
DEFAULT_AIRPORT_MODEL_URL = "https://sketchfab.com/3d-models/low-poly-airport-c90d33875c824a1884a1dc936db405a3"
DEFAULT_HELIPORT_MODEL_UID = "5bc89e02a58b4ebca7404e5e35da2481"
DEFAULT_HELIPORT_MODEL_URL = (
    "https://sketchfab.com/3d-models/heliport-helipad-air-base-helicopter-5bc89e02a58b4ebca7404e5e35da2481"
)
DEFAULT_AIRPORT_PACK_ROOT = Path("/tmp/ACESim-unreal/assets/airport_pack")
DEFAULT_HELIPORT_PACK_ROOT = Path("/tmp/ACESim-unreal/assets/heliport_pack")
DEFAULT_PACK_ROOT = DEFAULT_AIRPORT_PACK_ROOT
DEFAULT_PROJECT_CONTENT_DIR = Path("/tmp/ACESim-unreal/projects/ACESimUE/Content")
SKETCHFAB_API_ROOT = "https://api.sketchfab.com/v3/models"
DEFAULT_AUTH_SCHEME = "Token"
AIRPORT_ASSET_MANIFEST_NAME = "airport_asset_manifest.json"
HELIPORT_ASSET_MANIFEST_NAME = "heliport_asset_manifest.json"
AIRPORT_IMPORT_SCRIPT_NAME = "import_acesim_airport_assets.py"
HELIPORT_IMPORT_SCRIPT_NAME = "import_acesim_heliport_assets.py"
UrlOpen = Callable[..., Any]
ProgressCallback = Callable[[int, int | None], None]


@dataclass(frozen=True)
class EnvironmentAssetProfile:
    env_style: str
    display_name: str
    content_folder: str
    model_uid: str
    model_url: str
    default_pack_root: Path
    triangle_budget: tuple[int, int]

    @property
    def asset_prefix(self) -> str:
        return self.env_style

    @property
    def cache_manifest_name(self) -> str:
        if self.env_style == "airport":
            return AIRPORT_ASSET_MANIFEST_NAME
        if self.env_style == "heliport":
            return HELIPORT_ASSET_MANIFEST_NAME
        return f"{self.asset_prefix}_asset_manifest.json"

    @property
    def project_manifest_name(self) -> str:
        return f"{self.asset_prefix}_manifest.json"

    @property
    def import_script_name(self) -> str:
        if self.env_style == "airport":
            return AIRPORT_IMPORT_SCRIPT_NAME
        if self.env_style == "heliport":
            return HELIPORT_IMPORT_SCRIPT_NAME
        return f"import_acesim_{self.asset_prefix}_assets.py"

    @property
    def validation_name(self) -> str:
        return f"{self.asset_prefix}_import_validation.json"

    @property
    def ue_destination_path(self) -> str:
        return f"/Game/ACESim/Environment/{self.content_folder}/Model"


ENVIRONMENT_PROFILES = {
    "airport": EnvironmentAssetProfile(
        env_style="airport",
        display_name="low poly airport",
        content_folder="Airport",
        model_uid=DEFAULT_AIRPORT_MODEL_UID,
        model_url=DEFAULT_AIRPORT_MODEL_URL,
        default_pack_root=DEFAULT_AIRPORT_PACK_ROOT,
        triangle_budget=(200_000, 600_000),
    ),
    "heliport": EnvironmentAssetProfile(
        env_style="heliport",
        display_name="Heliport Helipad air base helicopter",
        content_folder="Heliport",
        model_uid=DEFAULT_HELIPORT_MODEL_UID,
        model_url=DEFAULT_HELIPORT_MODEL_URL,
        default_pack_root=DEFAULT_HELIPORT_PACK_ROOT,
        triangle_budget=(50_000, 200_000),
    ),
}
TRIANGLE_BUDGET = ENVIRONMENT_PROFILES["airport"].triangle_budget


@dataclass(frozen=True)
class AirportAssetResult:
    cache_dir: Path
    project_airport_dir: Path
    source_scene: Path
    manifest_path: Path
    import_script_path: Path


def _profile_for(env_style: str) -> EnvironmentAssetProfile:
    normalized = env_style.strip().lower()
    try:
        return ENVIRONMENT_PROFILES[normalized]
    except KeyError as exc:
        raise ValueError(f"Unsupported ACESim UE environment style: {env_style}") from exc


def _request_json(url: str, token: str, *, auth_scheme: str, urlopen: UrlOpen) -> dict[str, Any]:
    request = urllib.request.Request(url, headers={"Authorization": f"{auth_scheme} {token}"})
    with urlopen(request, timeout=30.0) as response:
        return json.loads(response.read().decode("utf-8"))


def _response_content_length(response: Any) -> int | None:
    headers = getattr(response, "headers", None)
    if headers is None:
        return None
    try:
        value = headers.get("Content-Length")
    except AttributeError:
        return None
    if value is None:
        return None
    try:
        return int(value)
    except (TypeError, ValueError):
        return None


def _download_to_file(
    url: str,
    destination: Path,
    *,
    urlopen: UrlOpen,
    progress_callback: ProgressCallback | None = None,
    chunk_size: int = 1024 * 1024,
) -> str:
    part_path = destination.with_suffix(destination.suffix + ".part")
    destination.parent.mkdir(parents=True, exist_ok=True)
    hasher = hashlib.sha256()
    downloaded = 0
    if part_path.exists():
        part_path.unlink()

    with urlopen(url, timeout=120.0) as response, part_path.open("wb") as output:
        total_size = _response_content_length(response)
        while True:
            chunk = response.read(chunk_size)
            if not chunk:
                break
            output.write(chunk)
            hasher.update(chunk)
            downloaded += len(chunk)
            if progress_callback is not None:
                progress_callback(downloaded, total_size)
            elif total_size:
                print(
                    "ACESim Sketchfab download progress: "
                    f"{downloaded / (1024 * 1024):.1f}/{total_size / (1024 * 1024):.1f} MiB",
                    flush=True,
                )
            else:
                print(
                    f"ACESim Sketchfab download progress: {downloaded / (1024 * 1024):.1f} MiB",
                    flush=True,
                )

    part_path.replace(destination)
    if progress_callback is not None and downloaded == 0:
        progress_callback(downloaded, None)
    return hasher.hexdigest()


def _sha256(payload: bytes) -> str:
    return hashlib.sha256(payload).hexdigest()


def _safe_extract_zip(archive_path: Path, destination: Path) -> None:
    if destination.exists():
        shutil.rmtree(destination)
    destination.mkdir(parents=True, exist_ok=True)
    with zipfile.ZipFile(archive_path) as archive:
        for member in archive.infolist():
            target = (destination / member.filename).resolve()
            if not str(target).startswith(str(destination.resolve())):
                raise RuntimeError(f"Sketchfab archive contains unsafe path: {member.filename}")
            archive.extract(member, destination)


def _find_gltf_scene(extracted_dir: Path) -> Path:
    candidates = [*extracted_dir.rglob("*.gltf"), *extracted_dir.rglob("*.glb")]
    if not candidates:
        raise RuntimeError(f"Sketchfab environment archive did not contain a glTF/GLB scene under {extracted_dir}")
    return sorted(candidates, key=lambda path: (path.suffix != ".gltf", len(path.parts), path.name))[0]


def _manifest_is_valid(cache_dir: Path, uid: str, profile: EnvironmentAssetProfile) -> bool:
    manifest_path = cache_dir / profile.cache_manifest_name
    if not manifest_path.is_file():
        return False
    try:
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    except json.JSONDecodeError:
        return False
    scene = cache_dir / manifest.get("source_scene", "")
    return manifest.get("uid") == uid and manifest.get("license") and manifest.get("author") and scene.is_file()


def _copy_cache_to_project(
    cache_dir: Path,
    project_environment_dir: Path,
    profile: EnvironmentAssetProfile,
) -> tuple[Path, dict[str, Any]]:
    manifest = json.loads((cache_dir / profile.cache_manifest_name).read_text(encoding="utf-8"))
    source_scene = cache_dir / manifest["source_scene"]
    project_source_root = project_environment_dir / "SourceModel"
    if project_source_root.exists():
        shutil.rmtree(project_source_root)
    shutil.copytree(source_scene.parent, project_source_root)
    project_scene = project_source_root / source_scene.name
    project_environment_dir.mkdir(parents=True, exist_ok=True)
    (project_environment_dir / profile.project_manifest_name).write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    attribution = (
        f"{manifest['name']} by {manifest['author']}\n"
        f"Source: {manifest['source_url']}\n"
        f"License: {manifest['license']}\n"
        f"Sketchfab UID: {manifest['uid']}\n"
        f"Budget: {manifest.get('triangle_count', 'unknown')} triangles, "
        f"{manifest.get('vertex_count', 'unknown')} vertices\n"
    )
    (project_environment_dir / "ATTRIBUTION.txt").write_text(attribution, encoding="utf-8")
    return project_scene, manifest


def _write_import_script(
    project_environment_dir: Path,
    project_scene: Path,
    profile: EnvironmentAssetProfile,
) -> Path:
    script_path = project_environment_dir / profile.import_script_name
    style = profile.env_style
    style_title = profile.content_folder
    script = f"""import json
import pathlib
import unreal

unreal.SystemLibrary.execute_console_command(None, "Interchange.FeatureFlags.Import.SyncToBrowser 0")

environment_dir = pathlib.Path({str(project_environment_dir)!r})
source_scene = pathlib.Path({str(project_scene)!r})
manifest_path = environment_dir / {profile.project_manifest_name!r}
destination_path = {profile.ue_destination_path!r}
asset_tools = unreal.AssetToolsHelpers.get_asset_tools()

if not source_scene.is_file():
    raise RuntimeError(f"ACESim {style} glTF scene missing: {{source_scene}}")
if not manifest_path.is_file():
    raise RuntimeError(f"ACESim {style} manifest missing: {{manifest_path}}")

manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
if "CC Attribution" not in manifest.get("license", ""):
    raise RuntimeError(f"ACESim {style} asset must be CC Attribution compatible: {{manifest.get('license')}}")

task = unreal.AssetImportTask()
task.filename = str(source_scene)
task.destination_path = destination_path
task.automated = True
task.save = True
task.replace_existing = True

asset_tools.import_asset_tasks([task])

assets = unreal.EditorAssetLibrary.list_assets(destination_path, recursive=True, include_folder=False)
mesh_assets = [
    asset
    for asset in assets
    if unreal.EditorAssetLibrary.find_asset_data(asset).asset_class_path.asset_name == "StaticMesh"
]
material_assets = [
    asset
    for asset in assets
    if "Material" in str(unreal.EditorAssetLibrary.find_asset_data(asset).asset_class_path.asset_name)
]
if not mesh_assets:
    raise RuntimeError("ACESim {style} import produced no StaticMesh assets")
if not material_assets:
    raise RuntimeError("ACESim {style} import produced no material assets")

invalid_material_slot_count = 0
default_material_slot_count = 0
for mesh_asset in mesh_assets:
    static_mesh = unreal.EditorAssetLibrary.load_asset(mesh_asset)
    if static_mesh is None:
        raise RuntimeError(f"ACESim {style} static mesh failed to load: {{mesh_asset}}")
    for slot_index, slot in enumerate(static_mesh.static_materials):
        material_interface = slot.material_interface
        if material_interface is None:
            invalid_material_slot_count += 1
            raise RuntimeError(
                f"ACESim {style} mesh uses invalid/default material: {{mesh_asset}} slot={{slot_index}} material=None"
            )
        material_name = material_interface.get_name()
        material_path = material_interface.get_path_name()
        if (
            "DefaultMaterial" in material_name
            or "WorldGridMaterial" in material_name
            or "WorldGridMaterial" in material_path
        ):
            default_material_slot_count += 1
            raise RuntimeError(
                "ACESim {style} mesh uses invalid/default material: "
                f"{{mesh_asset}} slot={{slot_index}} material={{material_path}}"
            )

validation_path = environment_dir / {profile.validation_name!r}
validation_path.write_text(
    json.dumps(
        {{
            "static_mesh_count": len(mesh_assets),
            "material_asset_count": len(material_assets),
            "invalid_material_slot_count": invalid_material_slot_count,
            "default_material_slot_count": default_material_slot_count,
        }},
        indent=2,
        sort_keys=True,
    )
    + "\\n",
    encoding="utf-8",
)
unreal.EditorAssetLibrary.save_directory("/Game/ACESim/Environment/{style_title}")
unreal.log("ACESim {style} assets imported")
"""
    script_path.write_text(script, encoding="utf-8")
    return script_path


def _download_to_cache(
    *,
    uid: str,
    cache_dir: Path,
    token: str,
    auth_scheme: str,
    profile: EnvironmentAssetProfile,
    progress_callback: ProgressCallback | None,
    urlopen: UrlOpen,
) -> None:
    metadata = _request_json(f"{SKETCHFAB_API_ROOT}/{uid}", token, auth_scheme=auth_scheme, urlopen=urlopen)
    download_payload = _request_json(
        f"{SKETCHFAB_API_ROOT}/{uid}/download",
        token,
        auth_scheme=auth_scheme,
        urlopen=urlopen,
    )
    gltf_info = download_payload.get("gltf")
    if not isinstance(gltf_info, dict) or not gltf_info.get("url"):
        raise RuntimeError("Sketchfab download response did not include a glTF archive URL")

    cache_dir.mkdir(parents=True, exist_ok=True)
    archive_path = cache_dir / f"{profile.asset_prefix}_gltf.zip"
    archive_sha = _download_to_file(
        str(gltf_info["url"]),
        archive_path,
        urlopen=urlopen,
        progress_callback=progress_callback,
    )
    extracted_dir = cache_dir / "gltf"
    _safe_extract_zip(archive_path, extracted_dir)
    scene = _find_gltf_scene(extracted_dir)

    manifest = {
        "uid": uid,
        "name": metadata.get("name", profile.display_name),
        "author": (metadata.get("user") or {}).get("displayName", "Sketchfab artist"),
        "source_url": metadata.get("viewerUrl") or profile.model_url,
        "license": (metadata.get("license") or {}).get("label", "CC Attribution"),
        "downloaded_at_unix": int(time.time()),
        "archive_sha256": archive_sha,
        "source_scene": str(scene.relative_to(cache_dir)),
        "triangle_count": int(metadata.get("faceCount") or metadata.get("triangleCount") or 0),
        "vertex_count": int(metadata.get("vertexCount") or 0),
        "triangle_budget": list(profile.triangle_budget),
        "env_style": profile.env_style,
    }
    if "CC Attribution" not in manifest["license"]:
        raise RuntimeError(f"Unsupported {profile.env_style} model license: {manifest['license']}")
    (cache_dir / profile.cache_manifest_name).write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )


def prepare_airport_assets(
    *,
    pack_root: Path | None = None,
    project_content_dir: Path = DEFAULT_PROJECT_CONTENT_DIR,
    uid: str | None = None,
    env_style: str = "airport",
    token: str | None = None,
    auth_scheme: str | None = None,
    force_download: bool = False,
    progress_callback: ProgressCallback | None = None,
    urlopen: UrlOpen = urllib.request.urlopen,
) -> AirportAssetResult:
    profile = _profile_for(env_style)
    uid = uid or profile.model_uid
    pack_root = pack_root or profile.default_pack_root
    cache_dir = pack_root.expanduser().resolve() / uid
    project_environment_dir = (
        project_content_dir.expanduser().resolve() / "ACESim" / "Environment" / profile.content_folder
    )
    token = token if token is not None else os.environ.get("SKETCHFAB_API_TOKEN")
    auth_scheme = (
        auth_scheme if auth_scheme is not None else os.environ.get("SKETCHFAB_AUTH_SCHEME", DEFAULT_AUTH_SCHEME)
    )
    if auth_scheme not in {"Token", "Bearer"}:
        raise RuntimeError(f"Unsupported SKETCHFAB_AUTH_SCHEME={auth_scheme}; expected Token or Bearer")

    if force_download and cache_dir.exists():
        shutil.rmtree(cache_dir)
    if not _manifest_is_valid(cache_dir, uid, profile):
        if not token:
            raise RuntimeError(
                f"SKETCHFAB_API_TOKEN is required to download the ACESim {profile.env_style} asset. "
                f"Set it once, or pre-populate the cache at {cache_dir}."
            )
        _download_to_cache(
            uid=uid,
            cache_dir=cache_dir,
            token=token,
            auth_scheme=auth_scheme,
            profile=profile,
            progress_callback=progress_callback,
            urlopen=urlopen,
        )

    project_scene, _manifest = _copy_cache_to_project(cache_dir, project_environment_dir, profile)
    import_script = _write_import_script(project_environment_dir, project_scene, profile)
    return AirportAssetResult(
        cache_dir=cache_dir,
        project_airport_dir=project_environment_dir,
        source_scene=project_scene,
        manifest_path=project_environment_dir / profile.project_manifest_name,
        import_script_path=import_script,
    )


def _default_env_style() -> str:
    return os.environ.get("ACESIM_UE_ENV_STYLE", "heliport").strip().lower()


def _default_uid_for(env_style: str) -> str:
    profile = _profile_for(env_style)
    if profile.env_style == "heliport":
        return os.environ.get("ACESIM_UE_HELIPORT_MODEL_UID", profile.model_uid)
    return os.environ.get("ACESIM_UE_AIRPORT_MODEL_UID", profile.model_uid)


def _default_pack_root_for(env_style: str) -> Path:
    profile = _profile_for(env_style)
    if profile.env_style == "heliport":
        return Path(os.environ.get("ACESIM_UE_HELIPORT_PACK_ROOT", str(profile.default_pack_root)))
    return Path(os.environ.get("ACESIM_UE_AIRPORT_PACK_ROOT", str(profile.default_pack_root)))


def main() -> None:
    parser = argparse.ArgumentParser(description="Prepare Sketchfab environment assets for ACESim UE packaging.")
    parser.add_argument("--env-style", choices=sorted(ENVIRONMENT_PROFILES), default=_default_env_style())
    parser.add_argument("--pack-root", type=Path, default=None)
    parser.add_argument("--project-content-dir", type=Path, default=DEFAULT_PROJECT_CONTENT_DIR)
    parser.add_argument("--uid", default=None)
    parser.add_argument(
        "--auth-scheme",
        choices=("Token", "Bearer"),
        default=os.environ.get("SKETCHFAB_AUTH_SCHEME", DEFAULT_AUTH_SCHEME),
    )
    parser.add_argument(
        "--force-download",
        action="store_true",
        default=(
            os.environ.get("ACESIM_UE_AIRPORT_FORCE_DOWNLOAD") == "1"
            or os.environ.get("ACESIM_UE_HELIPORT_FORCE_DOWNLOAD") == "1"
        ),
    )
    args = parser.parse_args()

    pack_root = args.pack_root if args.pack_root is not None else _default_pack_root_for(args.env_style)
    uid = args.uid if args.uid is not None else _default_uid_for(args.env_style)
    result = prepare_airport_assets(
        pack_root=pack_root,
        project_content_dir=args.project_content_dir,
        uid=uid,
        env_style=args.env_style,
        auth_scheme=args.auth_scheme,
        force_download=args.force_download,
    )
    print(f"Prepared ACESim UE {args.env_style} assets: {result.project_airport_dir}")
    print(f"ACESim {args.env_style} attribution: {result.project_airport_dir / 'ATTRIBUTION.txt'}")


if __name__ == "__main__":
    main()
