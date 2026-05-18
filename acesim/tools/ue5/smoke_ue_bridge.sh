#!/usr/bin/env bash
set -euo pipefail

UE_ROOT="${UE_ROOT:-/tmp/ACESim-unreal}"
UE_SRC_DIR="${UE_SRC_DIR:-${UE_ROOT}/UnrealEngine}"
UE_PROJECT_DIR="${UE_PROJECT_DIR:-${UE_ROOT}/projects/ACESimUE}"
UE_SMOKE_HOME="${UE_SMOKE_HOME:-${UE_ROOT}/smoke-home}"
RUN_LIVE_BRIDGE_SMOKE="${RUN_LIVE_BRIDGE_SMOKE:-0}"
UE_LOAD_TIMEOUT_SEC="${UE_LOAD_TIMEOUT_SEC:-120}"
UE_BRIDGE_TIMEOUT_SEC="${UE_BRIDGE_TIMEOUT_SEC:-25}"

PROJECT_FILE="${UE_PROJECT_DIR}/ACESimUE.uproject"
BUILD_SH="${UE_SRC_DIR}/Engine/Build/BatchFiles/Linux/Build.sh"
EDITOR_CMD="${UE_SRC_DIR}/Engine/Binaries/Linux/UnrealEditor-Cmd"
LOG_FILE="${UE_PROJECT_DIR}/Saved/Logs/ACESimUE.log"
BRIDGE_OUTPUT_LOG="${UE_PROJECT_DIR}/Saved/Logs/ACESimUEBridgeSmoke.log"

mkdir -p "${UE_SMOKE_HOME}/.config" "${UE_SMOKE_HOME}/.cache"
export HOME="${UE_SMOKE_HOME}"
export XDG_CONFIG_HOME="${UE_SMOKE_HOME}/.config"
export XDG_CACHE_HOME="${UE_SMOKE_HOME}/.cache"

if [ ! -x "${BUILD_SH}" ]; then
  echo "Missing Unreal Build.sh: ${BUILD_SH}" >&2
  exit 1
fi

if [ ! -x "${EDITOR_CMD}" ]; then
  echo "Missing UnrealEditor-Cmd: ${EDITOR_CMD}" >&2
  exit 1
fi

if [ ! -f "${PROJECT_FILE}" ]; then
  echo "Missing ACESim UE project: ${PROJECT_FILE}" >&2
  exit 1
fi

echo "[1/3] Building ACESimUEEditor"
"${BUILD_SH}" ACESimUEEditor Linux Development \
  -Project="${PROJECT_FILE}" \
  -Progress \
  -NoHotReloadFromIDE \
  -NoUBA

echo "[2/3] Loading project with UnrealEditor-Cmd SmokeTest"
/usr/bin/timeout "${UE_LOAD_TIMEOUT_SEC}s" \
  "${EDITOR_CMD}" \
  "${PROJECT_FILE}" \
  -run=SmokeTest \
  -DDC-ForceMemoryCache \
  -ddc=NoZenLocalFallback \
  -NullRHI \
  -Unattended \
  -NoSplash \
  -NoSound

if [ "${RUN_LIVE_BRIDGE_SMOKE}" != "1" ]; then
  echo "[3/3] Skipping live bridge smoke; set RUN_LIVE_BRIDGE_SMOKE=1 with ACESim headless already publishing."
  exit 0
fi

echo "[3/3] Running live ACESim -> UE bridge smoke"
mkdir -p "$(dirname "${BRIDGE_OUTPUT_LOG}")"
: > "${BRIDGE_OUTPUT_LOG}"

/usr/bin/timeout "${UE_BRIDGE_TIMEOUT_SEC}s" \
  "${EDITOR_CMD}" \
  "${PROJECT_FILE}" \
  -game \
  -DDC-ForceMemoryCache \
  -ddc=NoZenLocalFallback \
  -NullRHI \
  -Unattended \
  -NoSplash \
  -NoSound \
  > "${BRIDGE_OUTPUT_LOG}" 2>&1 &
BridgePid="$!"

BridgeDeadline=$((SECONDS + UE_BRIDGE_TIMEOUT_SEC))
SawConnected=0
SawApplied=0

while [ "${SECONDS}" -lt "${BridgeDeadline}" ]; do
  if [ -f "${BRIDGE_OUTPUT_LOG}" ]; then
    if grep -q "ACESim visual stream connected" "${BRIDGE_OUTPUT_LOG}"; then
      SawConnected=1
    fi
    if grep -q "ACESim visual state applied" "${BRIDGE_OUTPUT_LOG}"; then
      SawApplied=1
    fi
  fi

  if [ "${SawConnected}" = "1" ] && [ "${SawApplied}" = "1" ]; then
    kill "${BridgePid}" >/dev/null 2>&1 || true
    wait "${BridgePid}" >/dev/null 2>&1 || true
    echo "Live bridge smoke passed"
    exit 0
  fi

  if ! kill -0 "${BridgePid}" >/dev/null 2>&1; then
    wait "${BridgePid}" >/dev/null 2>&1 || true
    break
  fi

  sleep 0.25
done

kill "${BridgePid}" >/dev/null 2>&1 || true
wait "${BridgePid}" >/dev/null 2>&1 || true

if [ "${SawConnected}" != "1" ]; then
  echo "UE log did not show ACESim visual stream connected: ${BRIDGE_OUTPUT_LOG}" >&2
fi

if [ "${SawApplied}" != "1" ]; then
  echo "UE log did not show ACESim visual state applied: ${BRIDGE_OUTPUT_LOG}" >&2
fi

exit 1
