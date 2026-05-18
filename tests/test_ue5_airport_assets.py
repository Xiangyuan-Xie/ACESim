from __future__ import annotations

import io
import json
import zipfile
from pathlib import Path

import pytest

from acesim.tools.ue5 import prepare_ue_airport_assets


class _FakeResponse:
    def __init__(self, payload: bytes) -> None:
        self._payload = payload
        self._offset = 0
        self.headers = {"Content-Length": str(len(payload))}

    def __enter__(self) -> "_FakeResponse":
        return self

    def __exit__(self, *_exc: object) -> None:
        return None

    def read(self, size: int = -1) -> bytes:
        if self._offset >= len(self._payload):
            return b""
        if size is None or size < 0:
            chunk = self._payload[self._offset :]
        else:
            chunk = self._payload[self._offset : self._offset + size]
        self._offset += len(chunk)
        return chunk


class _ChunkedFakeResponse:
    def __init__(self, payload: bytes, chunk_size: int = 7) -> None:
        self._payload = payload
        self._chunk_size = chunk_size
        self._offset = 0
        self.headers = {"Content-Length": str(len(payload))}

    def __enter__(self) -> "_ChunkedFakeResponse":
        return self

    def __exit__(self, *_exc: object) -> None:
        return None

    def read(self, size: int = -1) -> bytes:
        if self._offset >= len(self._payload):
            return b""
        chunk_size = self._chunk_size if size is None or size < 0 else min(size, self._chunk_size)
        chunk = self._payload[self._offset : self._offset + chunk_size]
        self._offset += len(chunk)
        return chunk


def _gltf_zip() -> bytes:
    payload = io.BytesIO()
    with zipfile.ZipFile(payload, "w") as archive:
        archive.writestr(
            "scene.gltf", '{"asset":{"version":"2.0"},"meshes":[{"primitives":[{"attributes":{"POSITION":0}}]}]}'
        )
        archive.writestr("scene.bin", b"mesh-data")
        archive.writestr("textures/albedo.png", b"png-data")
    return payload.getvalue()


def test_prepare_airport_assets_requires_token_without_cache(tmp_path: Path) -> None:
    with pytest.raises(RuntimeError, match="SKETCHFAB_API_TOKEN"):
        prepare_ue_airport_assets.prepare_airport_assets(
            pack_root=tmp_path / "pack",
            project_content_dir=tmp_path / "Content",
            token=None,
            env_style="airport",
        )


def test_prepare_airport_assets_uses_valid_cache_without_token(tmp_path: Path) -> None:
    uid = prepare_ue_airport_assets.DEFAULT_AIRPORT_MODEL_UID
    cache_dir = tmp_path / "pack" / uid
    extracted_dir = cache_dir / "gltf"
    extracted_dir.mkdir(parents=True)
    (extracted_dir / "scene.gltf").write_text('{"asset":{"version":"2.0"}}', encoding="utf-8")
    manifest = {
        "uid": uid,
        "name": "low poly airport",
        "license": "CC Attribution",
        "author": "cached author",
        "source_url": prepare_ue_airport_assets.DEFAULT_AIRPORT_MODEL_URL,
        "source_scene": "gltf/scene.gltf",
        "archive_sha256": "cached",
        "triangle_budget": [200000, 600000],
    }
    (cache_dir / "airport_asset_manifest.json").write_text(json.dumps(manifest), encoding="utf-8")

    result = prepare_ue_airport_assets.prepare_airport_assets(
        pack_root=tmp_path / "pack",
        project_content_dir=tmp_path / "Content",
        token=None,
        env_style="airport",
    )

    airport_dir = tmp_path / "Content" / "ACESim" / "Environment" / "Airport"
    assert result.cache_dir == cache_dir
    assert result.source_scene == airport_dir / "SourceModel" / "scene.gltf"
    assert (airport_dir / "ATTRIBUTION.txt").read_text(encoding="utf-8").startswith("low poly airport by cached author")
    assert "airport_manifest.json" in (airport_dir / "import_acesim_airport_assets.py").read_text(encoding="utf-8")


def test_prepare_heliport_assets_uses_heliport_uid_paths_and_markers(tmp_path: Path) -> None:
    uid = prepare_ue_airport_assets.DEFAULT_HELIPORT_MODEL_UID
    cache_dir = tmp_path / "pack" / uid
    extracted_dir = cache_dir / "gltf"
    extracted_dir.mkdir(parents=True)
    (extracted_dir / "scene.gltf").write_text('{"asset":{"version":"2.0"}}', encoding="utf-8")
    manifest = {
        "uid": uid,
        "name": "Heliport Helipad air base helicopter",
        "license": "CC Attribution",
        "author": "heli artist",
        "source_url": prepare_ue_airport_assets.DEFAULT_HELIPORT_MODEL_URL,
        "source_scene": "gltf/scene.gltf",
        "archive_sha256": "cached-heli",
    }
    (cache_dir / "heliport_asset_manifest.json").write_text(json.dumps(manifest), encoding="utf-8")

    result = prepare_ue_airport_assets.prepare_airport_assets(
        pack_root=tmp_path / "pack",
        project_content_dir=tmp_path / "Content",
        token=None,
        env_style="heliport",
    )

    heliport_dir = tmp_path / "Content" / "ACESim" / "Environment" / "Heliport"
    import_script = result.import_script_path.read_text(encoding="utf-8")
    assert result.cache_dir == cache_dir
    assert result.source_scene == heliport_dir / "SourceModel" / "scene.gltf"
    assert (
        (heliport_dir / "ATTRIBUTION.txt")
        .read_text(encoding="utf-8")
        .startswith("Heliport Helipad air base helicopter by heli artist")
    )
    assert "heliport_manifest.json" in import_script
    assert "heliport_import_validation.json" in import_script
    assert "/Game/ACESim/Environment/Heliport/Model" in import_script
    assert "ACESim heliport assets imported" in import_script


def test_prepare_airport_assets_downloads_gltf_archive_and_writes_import_script(
    tmp_path: Path,
) -> None:
    token = "token-123"
    uid = prepare_ue_airport_assets.DEFAULT_AIRPORT_MODEL_UID
    calls: list[str] = []
    archive_payload = _gltf_zip()

    def fake_urlopen(request: object, timeout: float = 30.0) -> _FakeResponse:
        url = getattr(request, "full_url", request)
        calls.append(str(url))
        if str(url).endswith(f"/v3/models/{uid}"):
            headers = dict(getattr(request, "header_items")())
            assert headers["Authorization"] == f"Token {token}"
            return _FakeResponse(
                json.dumps(
                    {
                        "uid": uid,
                        "name": "low poly airport",
                        "user": {"displayName": "Example Artist"},
                        "license": {"label": "CC Attribution"},
                        "viewerUrl": prepare_ue_airport_assets.DEFAULT_AIRPORT_MODEL_URL,
                        "faceCount": 194800,
                        "vertexCount": 109000,
                    }
                ).encode("utf-8")
            )
        if str(url).endswith(f"/v3/models/{uid}/download"):
            return _FakeResponse(
                json.dumps(
                    {"gltf": {"url": "https://download.example/airport.zip", "size": len(archive_payload)}}
                ).encode("utf-8")
            )
        if str(url) == "https://download.example/airport.zip":
            return _FakeResponse(archive_payload)
        raise AssertionError(f"Unexpected URL: {url}")

    result = prepare_ue_airport_assets.prepare_airport_assets(
        pack_root=tmp_path / "pack",
        project_content_dir=tmp_path / "Content",
        token=token,
        env_style="airport",
        urlopen=fake_urlopen,
    )

    manifest = json.loads((result.cache_dir / "airport_asset_manifest.json").read_text(encoding="utf-8"))
    airport_dir = tmp_path / "Content" / "ACESim" / "Environment" / "Airport"
    import_script = (airport_dir / "import_acesim_airport_assets.py").read_text(encoding="utf-8")
    assert calls == [
        f"https://api.sketchfab.com/v3/models/{uid}",
        f"https://api.sketchfab.com/v3/models/{uid}/download",
        "https://download.example/airport.zip",
    ]
    assert manifest["license"] == "CC Attribution"
    assert manifest["author"] == "Example Artist"
    assert manifest["triangle_count"] == 194800
    assert manifest["vertex_count"] == 109000
    assert "194800 triangles" in (airport_dir / "ATTRIBUTION.txt").read_text(encoding="utf-8")
    assert "Interchange.FeatureFlags.Import.SyncToBrowser 0" in import_script
    assert "/Game/ACESim/Environment/Airport/Model" in import_script
    assert "/Game/ACESim/Environment/Airport" in import_script
    assert "airport_manifest.json" in import_script
    assert "AssetImportTask()" in import_script
    assert "manifest_task" not in import_script
    assert "ACESim airport assets imported" in import_script


def test_prepare_airport_assets_streams_archive_to_part_file_and_reports_progress(tmp_path: Path) -> None:
    token = "token-123"
    uid = prepare_ue_airport_assets.DEFAULT_HELIPORT_MODEL_UID
    archive_payload = _gltf_zip()
    progress: list[tuple[int, int | None]] = []

    def fake_urlopen(request: object, timeout: float = 30.0) -> _FakeResponse | _ChunkedFakeResponse:
        url = getattr(request, "full_url", request)
        if str(url).endswith(f"/v3/models/{uid}"):
            return _FakeResponse(
                json.dumps(
                    {
                        "uid": uid,
                        "name": "Heliport Helipad air base helicopter",
                        "user": {"displayName": "Example Artist"},
                        "license": {"label": "CC Attribution"},
                        "viewerUrl": prepare_ue_airport_assets.DEFAULT_HELIPORT_MODEL_URL,
                    }
                ).encode("utf-8")
            )
        if str(url).endswith(f"/v3/models/{uid}/download"):
            return _FakeResponse(json.dumps({"gltf": {"url": "https://download.example/heliport.zip"}}).encode("utf-8"))
        if str(url) == "https://download.example/heliport.zip":
            return _ChunkedFakeResponse(archive_payload)
        raise AssertionError(f"Unexpected URL: {url}")

    result = prepare_ue_airport_assets.prepare_airport_assets(
        pack_root=tmp_path / "pack",
        project_content_dir=tmp_path / "Content",
        token=token,
        env_style="heliport",
        urlopen=fake_urlopen,
        progress_callback=lambda downloaded, total: progress.append((downloaded, total)),
    )

    assert result.cache_dir.joinpath("heliport_gltf.zip").read_bytes() == archive_payload
    assert not result.cache_dir.joinpath("heliport_gltf.zip.part").exists()
    assert progress
    assert progress[-1] == (len(archive_payload), len(archive_payload))


def test_prepare_airport_assets_supports_oauth_bearer_scheme_override(tmp_path: Path) -> None:
    token = "oauth-token-123"
    uid = prepare_ue_airport_assets.DEFAULT_AIRPORT_MODEL_UID
    archive_payload = _gltf_zip()
    auth_headers: list[str] = []

    def fake_urlopen(request: object, timeout: float = 30.0) -> _FakeResponse:
        url = getattr(request, "full_url", request)
        if hasattr(request, "header_items"):
            auth_headers.append(dict(getattr(request, "header_items")())["Authorization"])
        if str(url).endswith(f"/v3/models/{uid}"):
            return _FakeResponse(
                json.dumps(
                    {
                        "uid": uid,
                        "name": "low poly airport",
                        "user": {"displayName": "Example Artist"},
                        "license": {"label": "CC Attribution"},
                        "viewerUrl": prepare_ue_airport_assets.DEFAULT_AIRPORT_MODEL_URL,
                    }
                ).encode("utf-8")
            )
        if str(url).endswith(f"/v3/models/{uid}/download"):
            return _FakeResponse(json.dumps({"gltf": {"url": "https://download.example/airport.zip"}}).encode("utf-8"))
        if str(url) == "https://download.example/airport.zip":
            return _FakeResponse(archive_payload)
        raise AssertionError(f"Unexpected URL: {url}")

    prepare_ue_airport_assets.prepare_airport_assets(
        pack_root=tmp_path / "pack",
        project_content_dir=tmp_path / "Content",
        token=token,
        auth_scheme="Bearer",
        env_style="airport",
        urlopen=fake_urlopen,
    )

    assert auth_headers == [f"Bearer {token}", f"Bearer {token}"]


def test_airport_import_script_rejects_default_or_empty_material_slots(tmp_path: Path) -> None:
    uid = prepare_ue_airport_assets.DEFAULT_AIRPORT_MODEL_UID
    cache_dir = tmp_path / "pack" / uid
    extracted_dir = cache_dir / "gltf"
    extracted_dir.mkdir(parents=True)
    (extracted_dir / "scene.gltf").write_text('{"asset":{"version":"2.0"}}', encoding="utf-8")
    manifest = {
        "uid": uid,
        "name": "low poly airport",
        "license": "CC Attribution",
        "author": "cached author",
        "source_url": prepare_ue_airport_assets.DEFAULT_AIRPORT_MODEL_URL,
        "source_scene": "gltf/scene.gltf",
    }
    (cache_dir / "airport_asset_manifest.json").write_text(json.dumps(manifest), encoding="utf-8")

    result = prepare_ue_airport_assets.prepare_airport_assets(
        pack_root=tmp_path / "pack",
        project_content_dir=tmp_path / "Content",
        token=None,
        env_style="airport",
    )

    import_script = result.import_script_path.read_text(encoding="utf-8")
    assert "for mesh_asset in mesh_assets:" in import_script
    assert "static_mesh.static_materials" in import_script
    assert "material_interface is None" in import_script
    assert "DefaultMaterial" in import_script
    assert "WorldGridMaterial" in import_script
    assert "ACESim airport mesh uses invalid/default material" in import_script
    assert "airport_import_validation.json" in import_script
    assert "invalid_material_slot_count" in import_script
    assert "default_material_slot_count" in import_script
    assert "static_mesh_count" in import_script
    assert "material_asset_count" in import_script
