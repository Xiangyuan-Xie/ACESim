#!/usr/bin/env bash
set -euo pipefail

UE_ROOT="${UE_ROOT:-/tmp/ACESim-unreal}"
UE_REPO_URL="${UE_REPO_URL:-git@github.com:EpicGames/UnrealEngine.git}"
UE_REF="${UE_REF:-5.7.4-release}"
UE_LOG_DIR="${UE_LOG_DIR:-${UE_ROOT}/logs}"
LOG_FILE="${UE_LOG_DIR}/host_check.txt"

mkdir -p "${UE_LOG_DIR}"

{
  echo "== ACESim UE5 Host Check =="
  echo "timestamp=$(date --iso-8601=seconds)"
  echo "ue_root=${UE_ROOT}"
  echo "ue_repo_url=${UE_REPO_URL}"
  echo "ue_ref=${UE_REF}"
  echo

  echo "[disk]"
  df -h "${UE_ROOT%/*}" /tmp /home || true
  echo

  echo "[toolchain]"
  command -v git || true
  command -v git-lfs || true
  command -v clang || true
  command -v cmake || true
  command -v dotnet || true
  echo

  echo "[sudo]"
  if sudo -n true >/dev/null 2>&1; then
    echo "sudo_non_interactive=ok"
  else
    echo "sudo_non_interactive=blocked"
  fi
  echo

  echo "[graphics]"
  if nvidia-smi >/dev/null 2>&1; then
    echo "nvidia_smi=ok"
  else
    echo "nvidia_smi=blocked"
  fi
  if command -v rg >/dev/null 2>&1; then
    lsmod | rg '^nvidia' || true
  else
    lsmod | grep -E '^nvidia' || true
  fi
  ls -l /dev/nvidia* 2>/dev/null || true
  echo

  echo "[repo_access]"
  repo_ref="$(git ls-remote --tags "${UE_REPO_URL}" "${UE_REF}" 2>/dev/null || true)"
  if [ -n "${repo_ref}" ]; then
    echo "repo_access=ok"
  else
    echo "repo_access=blocked"
  fi
} | tee "${LOG_FILE}"

echo "Host check log written to ${LOG_FILE}"
