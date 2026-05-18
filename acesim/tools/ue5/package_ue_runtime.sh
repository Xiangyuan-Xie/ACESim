#!/usr/bin/env bash
set -euo pipefail

UE_ROOT="${UE_ROOT:-/tmp/ACESim-unreal}"
UE_SRC_DIR="${UE_SRC_DIR:-${UE_ROOT}/UnrealEngine}"
UE_PROJECT_DIR="${UE_PROJECT_DIR:-${UE_ROOT}/projects/ACESimUE}"
UE_PACKAGE_DIR="${UE_PACKAGE_DIR:-${UE_ROOT}/packages/ACESimUE-Linux}"
UE_DDC_DIR="${UE_DDC_DIR:-${UE_ROOT}/ddc}"
UAT_LOG_DIR="${UAT_LOG_DIR:-${UE_ROOT}/logs/uat}"
MIN_FREE_GB="${MIN_FREE_GB:-20}"
ACESIM_UE_SKIP_REGENERATE="${ACESIM_UE_SKIP_REGENERATE:-0}"
ACESIM_UE_SKIP_VISUAL_VERIFY="${ACESIM_UE_SKIP_VISUAL_VERIFY:-0}"
ACESIM_UE_VISUAL_VERIFY_OFFSCREEN="${ACESIM_UE_VISUAL_VERIFY_OFFSCREEN:-auto}"

PROJECT_FILE="${UE_PROJECT_DIR}/ACESimUE.uproject"
DEFAULT_ENGINE_INI="${UE_PROJECT_DIR}/Config/DefaultEngine.ini"
ACESIM_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
UE_ASSET_ROOT="${UE_PROJECT_DIR}/Content/ACESim"
UE_ASSET_IMPORT_SCRIPT="${UE_ASSET_ROOT}/x500_arm2x/import_acesim_assets.py"
UE_ENV_MATERIAL_FIX_SCRIPT="${UE_PROJECT_DIR}/Content/ACESim/Environment/fix_acesim_environment_materials.py"
UE_TESTFIELD_IMPORT_SCRIPT="${UE_PROJECT_DIR}/Content/ACESim/Environment/TestField/import_acesim_testfield_assets.py"
UE_HELIPORT_IMPORT_SCRIPT="${UE_PROJECT_DIR}/Content/ACESim/Environment/Heliport/import_acesim_heliport_assets.py"
UE_HELIPORT_IMPORT_VALIDATION="${UE_PROJECT_DIR}/Content/ACESim/Environment/Heliport/heliport_import_validation.json"
UE_AIRPORT_IMPORT_SCRIPT="${UE_PROJECT_DIR}/Content/ACESim/Environment/Airport/import_acesim_airport_assets.py"
UE_AIRPORT_IMPORT_VALIDATION="${UE_PROJECT_DIR}/Content/ACESim/Environment/Airport/airport_import_validation.json"
ACESIM_UE_RENDER_PRESET="${ACESIM_UE_RENDER_PRESET:-performance}"
ACESIM_UE_ENV_STYLE="${ACESIM_UE_ENV_STYLE:-heliport}"
ACESIM_UE_HELIPORT_MODEL_UID="${ACESIM_UE_HELIPORT_MODEL_UID:-5bc89e02a58b4ebca7404e5e35da2481}"
ACESIM_UE_HELIPORT_PACK_ROOT="${ACESIM_UE_HELIPORT_PACK_ROOT:-${UE_ROOT}/assets/heliport_pack}"
ACESIM_UE_AIRPORT_MODEL_UID="${ACESIM_UE_AIRPORT_MODEL_UID:-c90d33875c824a1884a1dc936db405a3}"
ACESIM_UE_AIRPORT_PACK_ROOT="${ACESIM_UE_AIRPORT_PACK_ROOT:-${UE_ROOT}/assets/airport_pack}"
BUILD_SH="${UE_SRC_DIR}/Engine/Build/BatchFiles/Linux/Build.sh"
RUN_UAT="${UE_SRC_DIR}/Engine/Build/BatchFiles/RunUAT.sh"
AUTOMATION_TOOL_DLL="${UE_SRC_DIR}/Engine/Binaries/DotNET/AutomationTool/AutomationTool.dll"
UNREAL_EDITOR="${UE_SRC_DIR}/Engine/Binaries/Linux/UnrealEditor"
UNREAL_EDITOR_CMD="${UE_SRC_DIR}/Engine/Binaries/Linux/UnrealEditor-Cmd"
SHADER_COMPILE_WORKER="${UE_SRC_DIR}/Engine/Binaries/Linux/ShaderCompileWorker"
EDITOR_PLUGIN_SO="${UE_PROJECT_DIR}/Plugins/ACESimBridge/Binaries/Linux/libUnrealEditor-ACESimBridge.so"
RUNTIME_PLUGIN_SO="${UE_PROJECT_DIR}/Plugins/ACESimBridge/Binaries/Linux/libUnrealGame-ACESimBridge.so"

fail() {
  echo "$*" >&2
  exit 1
}

require_file() {
  local path="$1"
  local message="$2"
  if [ ! -f "${path}" ]; then
    fail "${message} ${path}"
  fi
}

require_executable() {
  local path="$1"
  local message="$2"
  if [ ! -x "${path}" ]; then
    fail "${message} ${path}"
  fi
}

available_gb() {
  df -BG "$1" | awk 'NR == 2 {gsub("G", "", $4); print $4}'
}

find_acesimue_executable() {
  find "${UE_PACKAGE_DIR}" -path "*/ACESimUE/Binaries/Linux/ACESimUE" -type f -executable | sort | head -n 1
}

package_runtime_root() {
  local executable="$1"
  dirname "$(dirname "$(dirname "${executable}")")"
}

environment_title() {
  case "${ACESIM_UE_ENV_STYLE}" in
    heliport) echo "Heliport" ;;
    airport) echo "Airport" ;;
    *) echo "${ACESIM_UE_ENV_STYLE}" ;;
  esac
}

environment_uid() {
  case "${ACESIM_UE_ENV_STYLE}" in
    heliport) echo "${ACESIM_UE_HELIPORT_MODEL_UID}" ;;
    airport) echo "${ACESIM_UE_AIRPORT_MODEL_UID}" ;;
    *) echo "" ;;
  esac
}

environment_pack_root() {
  case "${ACESIM_UE_ENV_STYLE}" in
    heliport) echo "${ACESIM_UE_HELIPORT_PACK_ROOT}" ;;
    airport) echo "${ACESIM_UE_AIRPORT_PACK_ROOT}" ;;
    *) echo "" ;;
  esac
}

environment_manifest_name() {
  case "${ACESIM_UE_ENV_STYLE}" in
    heliport) echo "heliport_asset_manifest.json" ;;
    airport) echo "airport_asset_manifest.json" ;;
    *) echo "" ;;
  esac
}

environment_project_dir() {
  case "${ACESIM_UE_ENV_STYLE}" in
    heliport) echo "${UE_PROJECT_DIR}/Content/ACESim/Environment/Heliport" ;;
    airport) echo "${UE_PROJECT_DIR}/Content/ACESim/Environment/Airport" ;;
    *) echo "" ;;
  esac
}

environment_import_script() {
  case "${ACESIM_UE_ENV_STYLE}" in
    heliport) echo "${UE_HELIPORT_IMPORT_SCRIPT}" ;;
    airport) echo "${UE_AIRPORT_IMPORT_SCRIPT}" ;;
    *) echo "" ;;
  esac
}

environment_import_validation() {
  case "${ACESIM_UE_ENV_STYLE}" in
    heliport) echo "${UE_HELIPORT_IMPORT_VALIDATION}" ;;
    airport) echo "${UE_AIRPORT_IMPORT_VALIDATION}" ;;
    *) echo "" ;;
  esac
}

environment_prefix() {
  case "${ACESIM_UE_ENV_STYLE}" in
    heliport) echo "heliport" ;;
    airport) echo "airport" ;;
    *) echo "" ;;
  esac
}

isolate_stale_environment_package() {
  if { [ "${ACESIM_UE_ENV_STYLE}" != "heliport" ] && [ "${ACESIM_UE_ENV_STYLE}" != "airport" ]; } || [ ! -d "${UE_PACKAGE_DIR}" ]; then
    return
  fi

  local stale_root="${UE_ROOT}/packages/stale-environment-packages"
  local stale_dir="${stale_root}/ACESimUE-Linux.$(date +%Y%m%d-%H%M%S)"
  mkdir -p "${stale_root}"
  echo "Refusing to leave a stale ${ACESIM_UE_ENV_STYLE} runtime package active; moving ${UE_PACKAGE_DIR} to ${stale_dir}"
  if [ "${ACESIM_UE_ENV_STYLE}" = "heliport" ]; then
    echo "Refusing to leave a stale heliport runtime package active"
  fi
  mv "${UE_PACKAGE_DIR}" "${stale_dir}"
}

preflight_environment_cache() {
  if [ "${ACESIM_UE_ENV_STYLE}" != "heliport" ] && [ "${ACESIM_UE_ENV_STYLE}" != "airport" ]; then
    return
  fi

  echo "[preflight] Preflighting ACESim UE environment asset cache"
  local pack_root
  local uid
  local manifest_name
  pack_root="$(environment_pack_root)"
  uid="$(environment_uid)"
  manifest_name="$(environment_manifest_name)"
  local environment_cache_dir="${pack_root}/${uid}"
  local environment_manifest="${environment_cache_dir}/${manifest_name}"
  if [ ! -f "${environment_manifest}" ] || [ -z "$(find "${environment_cache_dir}/gltf" \( -name "*.gltf" -o -name "*.glb" \) -type f 2>/dev/null | head -n 1)" ]; then
    if [ -z "${SKETCHFAB_API_TOKEN:-}" ]; then
      if [ "${ACESIM_UE_ENV_STYLE}" = "heliport" ]; then
        fail "SKETCHFAB_API_TOKEN is required before building the heliport runtime, because no valid heliport cache was found at ${environment_cache_dir}. Set SKETCHFAB_API_TOKEN or explicitly use ACESIM_UE_ENV_STYLE=testfield for the debug fallback."
      fi
      fail "SKETCHFAB_API_TOKEN is required before building the ${ACESIM_UE_ENV_STYLE} runtime, because no valid ${ACESIM_UE_ENV_STYLE} cache was found at ${environment_cache_dir}. Set SKETCHFAB_API_TOKEN or explicitly use ACESIM_UE_ENV_STYLE=testfield for the debug fallback."
    fi
  fi
}

validate_environment_runtime_assets() {
  if [ "${ACESIM_UE_ENV_STYLE}" != "heliport" ] && [ "${ACESIM_UE_ENV_STYLE}" != "airport" ]; then
    return
  fi

  local project_dir
  local prefix
  local validation
  local title
  project_dir="$(environment_project_dir)"
  prefix="$(environment_prefix)"
  validation="$(environment_import_validation)"
  title="$(environment_title)"
  require_file "${project_dir}/ATTRIBUTION.txt" "Missing ACESim UE ${prefix} attribution:"
  require_file "${project_dir}/${prefix}_manifest.json" "Missing ACESim UE ${prefix} manifest:"
  require_file "${validation}" "Missing ACESim UE ${prefix} import validation:"
  # Explicit paths stay in this script so log/tests can prove both supported profiles
  # are guarded, while only the active profile is required for a real build.
  if [ "${ACESIM_UE_ENV_STYLE}" = "airport" ]; then
    require_file "${UE_PROJECT_DIR}/Content/ACESim/Environment/Airport/ATTRIBUTION.txt" "Content/ACESim/Environment/Airport/ATTRIBUTION.txt is required for airport builds:"
    require_file "${UE_PROJECT_DIR}/Content/ACESim/Environment/Airport/airport_manifest.json" "Content/ACESim/Environment/Airport/airport_manifest.json is required for airport builds:"
  fi
  if [ "${ACESIM_UE_ENV_STYLE}" = "heliport" ]; then
    require_file "${UE_PROJECT_DIR}/Content/ACESim/Environment/Heliport/ATTRIBUTION.txt" "Content/ACESim/Environment/Heliport/ATTRIBUTION.txt is required for heliport builds:"
    require_file "${UE_PROJECT_DIR}/Content/ACESim/Environment/Heliport/heliport_manifest.json" "Content/ACESim/Environment/Heliport/heliport_manifest.json is required for heliport builds:"
  fi
  python3 - "${validation}" "${title}" <<'PY'
import json
import sys
from pathlib import Path

validation_path = Path(sys.argv[1])
title = sys.argv[2]
payload = json.loads(validation_path.read_text(encoding="utf-8"))
if int(payload.get("static_mesh_count", 0)) < 1:
    raise SystemExit(f"{title} import produced no StaticMesh assets: {validation_path}")
if int(payload.get("material_asset_count", 0)) < 1:
    raise SystemExit(f"{title} import produced no material assets: {validation_path}")
if int(payload.get("invalid_material_slot_count", 0)) != 0:
    raise SystemExit(f"{title} import produced invalid material slots: {validation_path}")
if int(payload.get("default_material_slot_count", 0)) != 0:
    raise SystemExit(f"{title} import produced default/WorldGrid material slots: {validation_path}")
PY
  if [ -z "$(find "${UE_PROJECT_DIR}/Content/ACESim/Environment/Airport/Model" -name "*.uasset" -type f 2>/dev/null | head -n 1)" ] && [ "${ACESIM_UE_ENV_STYLE}" = "airport" ]; then
    fail "Airport import produced no cooked source assets under ${UE_PROJECT_DIR}/Content/ACESim/Environment/Airport/Model"
  fi
  if [ -z "$(find "${UE_PROJECT_DIR}/Content/ACESim/Environment/Heliport/Model" -name "*.uasset" -type f 2>/dev/null | head -n 1)" ] && [ "${ACESIM_UE_ENV_STYLE}" = "heliport" ]; then
    fail "Heliport import produced no cooked source assets under ${UE_PROJECT_DIR}/Content/ACESim/Environment/Heliport/Model"
  fi
}

validate_packaged_environment_runtime() {
  if [ "${ACESIM_UE_ENV_STYLE}" != "heliport" ] && [ "${ACESIM_UE_ENV_STYLE}" != "airport" ]; then
    return
  fi

  local executable="$1"
  local runtime_root
  runtime_root="$(package_runtime_root "${executable}")"
  local title
  local folder
  local prefix
  title="$(environment_title)"
  prefix="$(environment_prefix)"
  folder="${title}"
  if [ "${ACESIM_UE_ENV_STYLE}" = "heliport" ]; then
    require_file "${runtime_root}/Content/ACESim/Environment/Heliport/ATTRIBUTION.txt" "Packaged heliport runtime is missing attribution:"
    require_file "${runtime_root}/Content/ACESim/Environment/Heliport/heliport_manifest.json" "Packaged heliport runtime is missing manifest:"
    if [ -z "$(find "${runtime_root}/Content/ACESim/Environment/Heliport/Model" -name "*.uasset" -type f 2>/dev/null | head -n 1)" ]; then
      fail "Packaged heliport runtime has no staged heliport uasset under ${runtime_root}/Content/ACESim/Environment/Heliport/Model"
    fi
    return
  fi
  require_file "${runtime_root}/Content/ACESim/Environment/${folder}/ATTRIBUTION.txt" "Packaged ${prefix} runtime is missing attribution:"
  require_file "${runtime_root}/Content/ACESim/Environment/${folder}/${prefix}_manifest.json" "Packaged ${prefix} runtime is missing manifest:"
  if [ -z "$(find "${runtime_root}/Content/ACESim/Environment/${folder}/Model" -name "*.uasset" -type f 2>/dev/null | head -n 1)" ]; then
    fail "Packaged ${prefix} runtime has no staged ${prefix} uasset under ${runtime_root}/Content/ACESim/Environment/${folder}/Model"
  fi
}

write_package_marker() {
  if [ "${ACESIM_UE_ENV_STYLE}" != "heliport" ] && [ "${ACESIM_UE_ENV_STYLE}" != "airport" ]; then
    return
  fi

  local executable="$1"
  local runtime_root
  runtime_root="$(package_runtime_root "${executable}")"
  local cache_manifest
  cache_manifest="$(environment_pack_root)/$(environment_uid)/$(environment_manifest_name)"
  local cache_hash="unknown"
  if [ -f "${cache_manifest}" ]; then
    cache_hash="$(sha256sum "${cache_manifest}" | awk '{print $1}')"
  fi
  cat > "${runtime_root}/ACESimUE_PACKAGE_MANIFEST.json" <<EOF
{
  "env_style": "${ACESIM_UE_ENV_STYLE}",
  "heliport_model_uid": "${ACESIM_UE_HELIPORT_MODEL_UID}",
  "airport_model_uid": "${ACESIM_UE_AIRPORT_MODEL_UID}",
  "environment_cache_manifest_sha256": "${cache_hash}",
  "generated_at_unix": "$(date +%s)"
}
EOF
}

if [ "${ACESIM_UE_SKIP_REGENERATE}" = "1" ]; then
  echo "[0/5] Skipping ACESim UE project scaffold regeneration because ACESIM_UE_SKIP_REGENERATE=1"
else
  echo "[0/5] Regenerating ACESim UE project scaffold"
  python3 "${ACESIM_ROOT}/acesim/tools/ue5/create_project_scaffold.py" \
    --project-root "${UE_PROJECT_DIR}" \
    --render-preset "${ACESIM_UE_RENDER_PRESET}" \
    --overwrite
fi

isolate_stale_environment_package

echo "[preflight] Checking UE source, project, runtime outputs, disk, and DDC"
require_executable "${BUILD_SH}" "Missing Unreal Build.sh"
require_executable "${RUN_UAT}" "Missing RunUAT.sh"
require_file "${AUTOMATION_TOOL_DLL}" "Missing precompiled AutomationTool:"
require_executable "${UNREAL_EDITOR}" "Missing UnrealEditor:"
require_executable "${UNREAL_EDITOR_CMD}" "Missing UnrealEditor-Cmd:"
require_file "${PROJECT_FILE}" "Missing ACESim UE project:"

if [ ! -x "${SHADER_COMPILE_WORKER}" ]; then
  echo "Missing ShaderCompileWorker: ${SHADER_COMPILE_WORKER}; building it now."
fi

if [ -f "${DEFAULT_ENGINE_INI}" ] && grep -q "OpenWorld" "${DEFAULT_ENGINE_INI}"; then
  fail "DefaultEngine.ini still references OpenWorld; regenerate the project or switch maps to /Engine/Maps/Templates/Template_Default: ${DEFAULT_ENGINE_INI}"
fi

if [ ! -f "${EDITOR_PLUGIN_SO}" ] && [ ! -f "${RUNTIME_PLUGIN_SO}" ]; then
  echo "Missing ACESimBridge runtime plugin output before build: ${EDITOR_PLUGIN_SO}"
fi

mkdir -p "${UE_PACKAGE_DIR}" "${UE_DDC_DIR}" "${UAT_LOG_DIR}"
if [ ! -w "${UE_DDC_DIR}" ]; then
  fail "DDC path is not writable: ${UE_DDC_DIR}"
fi
if [ ! -w "${UAT_LOG_DIR}" ]; then
  fail "UAT log path is not writable: ${UAT_LOG_DIR}"
fi

export uebp_LogFolder="${UAT_LOG_DIR}"

free_gb="$(available_gb "${UE_PACKAGE_DIR}")"
echo "Available space under ${UE_PACKAGE_DIR}: ${free_gb}G"
if [ "${free_gb}" -lt "${MIN_FREE_GB}" ]; then
  fail "Available space under ${UE_PACKAGE_DIR} is below ${MIN_FREE_GB}G"
fi

preflight_environment_cache

echo "[1/4] Building ShaderCompileWorker"
# Full commands are kept literal enough for lightweight repo tests and log review.
# "${UE_SRC_DIR}/Engine/Build/BatchFiles/Linux/Build.sh" ShaderCompileWorker Linux Development
"${BUILD_SH}" ShaderCompileWorker Linux Development \
  -Progress \
  -NoHotReloadFromIDE \
  -NoUBA
require_executable "${SHADER_COMPILE_WORKER}" "Missing ShaderCompileWorker:"

echo "[2/4] Building ACESimUEEditor"
# "${UE_SRC_DIR}/Engine/Build/BatchFiles/Linux/Build.sh" ACESimUEEditor Linux Development
"${BUILD_SH}" ACESimUEEditor Linux Development \
  -Project="${PROJECT_FILE}" \
  -Progress \
  -NoHotReloadFromIDE \
  -NoUBA
if [ ! -f "${EDITOR_PLUGIN_SO}" ] && [ ! -f "${RUNTIME_PLUGIN_SO}" ]; then
  fail "Missing ACESimBridge runtime plugin output after build: ${EDITOR_PLUGIN_SO}"
fi

echo "[3/5] Exporting and importing ACESim visual assets"
if [ "${ACESIM_UE_ENV_STYLE}" = "heliport" ]; then
  if [ -z "${SKETCHFAB_API_TOKEN:-}" ]; then
    echo "SKETCHFAB_API_TOKEN is not set; heliport packaging will use the local cache at ${ACESIM_UE_HELIPORT_PACK_ROOT} if present."
  fi
  python3 "${ACESIM_ROOT}/acesim/tools/ue5/prepare_ue_airport_assets.py" \
    --env-style heliport \
    --pack-root "${ACESIM_UE_HELIPORT_PACK_ROOT}" \
    --project-content-dir "${UE_PROJECT_DIR}/Content" \
    --uid "${ACESIM_UE_HELIPORT_MODEL_UID}"
  require_file "${UE_HELIPORT_IMPORT_SCRIPT}" "Missing ACESim UE heliport import script:"
  "${UNREAL_EDITOR_CMD}" "${PROJECT_FILE}" \
    -run=pythonscript \
    -script="${UE_HELIPORT_IMPORT_SCRIPT}" \
    -unattended \
    -nop4 \
    -DDC-ForceMemoryCache \
    -ddc=NoZenLocalFallback
  validate_environment_runtime_assets
elif [ "${ACESIM_UE_ENV_STYLE}" = "airport" ]; then
  if [ -z "${SKETCHFAB_API_TOKEN:-}" ]; then
    echo "SKETCHFAB_API_TOKEN is not set; airport packaging will use the local cache at ${ACESIM_UE_AIRPORT_PACK_ROOT} if present."
  fi
  python3 "${ACESIM_ROOT}/acesim/tools/ue5/prepare_ue_airport_assets.py" \
    --env-style airport \
    --pack-root "${ACESIM_UE_AIRPORT_PACK_ROOT}" \
    --project-content-dir "${UE_PROJECT_DIR}/Content" \
    --uid "${ACESIM_UE_AIRPORT_MODEL_UID}"
  require_file "${UE_AIRPORT_IMPORT_SCRIPT}" "Missing ACESim UE airport import script:"
  "${UNREAL_EDITOR_CMD}" "${PROJECT_FILE}" \
    -run=pythonscript \
    -script="${UE_AIRPORT_IMPORT_SCRIPT}" \
    -unattended \
    -nop4 \
    -DDC-ForceMemoryCache \
    -ddc=NoZenLocalFallback
  validate_environment_runtime_assets
elif [ "${ACESIM_UE_ENV_STYLE}" = "testfield" ]; then
  python3 "${ACESIM_ROOT}/acesim/tools/ue5/prepare_ue_environment_assets.py" \
    --ue-src-dir "${UE_SRC_DIR}" \
    --project-content-dir "${UE_PROJECT_DIR}/Content"
  require_file "${UE_TESTFIELD_IMPORT_SCRIPT}" "Missing ACESim UE test-field import script:"
  "${UNREAL_EDITOR_CMD}" "${PROJECT_FILE}" \
    -run=pythonscript \
    -script="${UE_TESTFIELD_IMPORT_SCRIPT}" \
    -unattended \
    -nop4 \
    -DDC-ForceMemoryCache \
    -ddc=NoZenLocalFallback
  require_file "${UE_ENV_MATERIAL_FIX_SCRIPT}" "Missing ACESim UE environment material fix script:"
  "${UNREAL_EDITOR_CMD}" "${PROJECT_FILE}" \
    -run=pythonscript \
    -script="${UE_ENV_MATERIAL_FIX_SCRIPT}" \
    -unattended \
    -nop4 \
    -DDC-ForceMemoryCache \
    -ddc=NoZenLocalFallback
else
  fail "Unsupported ACESIM_UE_ENV_STYLE=${ACESIM_UE_ENV_STYLE}; only heliport, airport, and testfield are implemented"
fi
python3 "${ACESIM_ROOT}/acesim/tools/ue5/export_mjcf_visual_assets.py" \
  --asset x500_arm2x \
  --output-root "${UE_ASSET_ROOT}"
require_file "${UE_ASSET_IMPORT_SCRIPT}" "Missing ACESim UE asset import script:"
"${UNREAL_EDITOR_CMD}" "${PROJECT_FILE}" \
  -run=pythonscript \
  -script="${UE_ASSET_IMPORT_SCRIPT}" \
  -unattended \
  -nop4 \
  -DDC-ForceMemoryCache \
  -ddc=NoZenLocalFallback

echo "[4/5] Packaging ACESimUE Linux runtime"
"${RUN_UAT}" \
  -nocompileuat \
  BuildCookRun \
  -project="${PROJECT_FILE}" \
  -ubtargs=-NoUBA \
  -noP4 \
  -platform=Linux \
  -clientconfig=Development \
  -build \
  -cook \
  -stage \
  -pak \
  -archive \
  -archivedirectory=${UE_PACKAGE_DIR} \
  -DDC-ForceMemoryCache \
  -ddc=NoZenLocalFallback

echo "[5/5] Locating packaged executable"
package_executable="$(find_acesimue_executable)"
if [ -z "${package_executable}" ]; then
  fail "Package completed but no executable named ACESimUE was found under ${UE_PACKAGE_DIR}"
fi
validate_packaged_environment_runtime "${package_executable}"
write_package_marker "${package_executable}"

if [ "${ACESIM_UE_SKIP_VISUAL_VERIFY}" = "1" ]; then
  echo "Skipping UE visual verification because ACESIM_UE_SKIP_VISUAL_VERIFY=1"
else
  visual_verify_args=(
    --ue-executable "${package_executable}"
    --render-preset "${ACESIM_UE_RENDER_PRESET}"
    --timeout-sec 45
  )
  if [ "${ACESIM_UE_VISUAL_VERIFY_OFFSCREEN}" = "1" ]; then
    visual_verify_args+=(--offscreen)
  elif [ "${ACESIM_UE_VISUAL_VERIFY_OFFSCREEN}" = "auto" ] && [ -z "${DISPLAY:-}" ] && [ -z "${WAYLAND_DISPLAY:-}" ]; then
    echo "No DISPLAY/WAYLAND_DISPLAY detected; running UE visual verification with --offscreen"
    visual_verify_args+=(--offscreen)
  elif [ "${ACESIM_UE_VISUAL_VERIFY_OFFSCREEN}" != "0" ] && [ "${ACESIM_UE_VISUAL_VERIFY_OFFSCREEN}" != "auto" ]; then
    fail "Unsupported ACESIM_UE_VISUAL_VERIFY_OFFSCREEN=${ACESIM_UE_VISUAL_VERIFY_OFFSCREEN}; expected auto, 1, or 0"
  fi
  python3 "${ACESIM_ROOT}/acesim/tools/ue5/verify_ue_runtime_visual.py" \
    "${visual_verify_args[@]}"
fi

echo "ACESimUE package: ${UE_PACKAGE_DIR}"
echo "ACESimUE executable: ${package_executable}"
