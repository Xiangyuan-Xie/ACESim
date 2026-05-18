#!/usr/bin/env bash
set -euo pipefail

UE_ROOT="${UE_ROOT:-/tmp/ACESim-unreal}"
UE_REPO_URL="${UE_REPO_URL:-git@github.com:EpicGames/UnrealEngine.git}"
UE_REF="${UE_REF:-5.7.4-release}"
UE_SRC_DIR="${UE_SRC_DIR:-${UE_ROOT}/UnrealEngine}"
UE_PROJECT_DIR="${UE_PROJECT_DIR:-${UE_ROOT}/projects/ACESimUE}"
UE_LOG_DIR="${UE_LOG_DIR:-${UE_ROOT}/logs}"
UE_REQUIRE_NVIDIA="${UE_REQUIRE_NVIDIA:-1}"
ACESIM_UE_SKIP_REGENERATE="${ACESIM_UE_SKIP_REGENERATE:-0}"
ACESIM_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"

if ! command -v sudo >/dev/null 2>&1; then
  echo "sudo is required to install Unreal Engine build dependencies on Ubuntu."
  exit 1
fi

if [ -t 0 ]; then
  sudo -v
elif ! sudo -n true >/dev/null 2>&1; then
  echo "sudo requires an interactive password, but this shell has no TTY."
  echo "Run this script from a terminal, or refresh sudo first with: sudo -v"
  exit 1
fi

echo "[1/5] Preparing workspace at ${UE_ROOT}"
mkdir -p "${UE_ROOT}" "${UE_ROOT}/projects" "${UE_LOG_DIR}"

echo "[2/5] Installing Ubuntu build prerequisites"
sudo apt-get update
sudo apt-get install -y \
  build-essential \
  clang \
  cmake \
  curl \
  dotnet-sdk-8.0 \
  git \
  git-lfs \
  libglib2.0-dev \
  libgtk-3-dev \
  libnss3-dev \
  libssl-dev \
  libvulkan-dev \
  libxcb-xinerama0 \
  libxkbcommon-x11-0 \
  lld \
  mesa-utils \
  mono-complete \
  python3 \
  rsync \
  unzip \
  uuid-dev \
  xdg-user-dirs \
  zlib1g-dev \
  libzmq3-dev

echo "[3/5] Verifying graphics driver"
if [ "${UE_REQUIRE_NVIDIA}" = "1" ] && ! nvidia-smi >/dev/null 2>&1; then
  echo "NVIDIA driver is not healthy. Stop here until the graphics stack is fixed."
  echo "Hint: the kernel module may be loaded but /dev/nvidia* nodes are still missing."
  echo "For compile-only or NullRHI smoke checks, re-run with UE_REQUIRE_NVIDIA=0."
  exit 2
fi

echo "[4/5] Cloning Unreal Engine source"
UE_REMOTE_REF="$(git ls-remote --tags "${UE_REPO_URL}" "${UE_REF}" 2>/dev/null || true)"
if [ -z "${UE_REMOTE_REF}" ]; then
  echo "Cannot access ${UE_REPO_URL} (${UE_REF})."
  echo "Make sure this machine has EpicGames UnrealEngine repository access and valid Git credentials."
  exit 3
fi

if [ ! -d "${UE_SRC_DIR}/.git" ]; then
  git clone --branch "${UE_REF}" --depth 1 "${UE_REPO_URL}" "${UE_SRC_DIR}"
fi

cd "${UE_SRC_DIR}"
git lfs install
git fetch --tags --force --depth 1 origin "refs/tags/${UE_REF}:refs/tags/${UE_REF}"
git checkout "${UE_REF}"
./Setup.sh
./GenerateProjectFiles.sh
make -j"$(nproc)" UnrealEditor
"${UE_SRC_DIR}/Engine/Build/BatchFiles/Linux/Build.sh" ShaderCompileWorker Linux Development \
  -Progress \
  -NoHotReloadFromIDE \
  -NoUBA

echo "[5/5] Generating and building ACESim UE project scaffold"
if [ "${ACESIM_UE_SKIP_REGENERATE}" = "1" ]; then
  echo "Skipping ACESim UE project scaffold regeneration because ACESIM_UE_SKIP_REGENERATE=1"
else
  python3 "${ACESIM_ROOT}/acesim/tools/ue5/create_project_scaffold.py" --project-root "${UE_PROJECT_DIR}" --overwrite
fi
"${UE_SRC_DIR}/Engine/Build/BatchFiles/Linux/Build.sh" ACESimUEEditor Linux Development \
  -Project="${UE_PROJECT_DIR}/ACESimUE.uproject" \
  -Progress \
  -NoHotReloadFromIDE \
  -NoUBA

echo "UE source: ${UE_SRC_DIR}"
echo "UE project: ${UE_PROJECT_DIR}"
echo "Editor: ${UE_SRC_DIR}/Engine/Binaries/Linux/UnrealEditor"
echo "Bridge plugin: ${UE_PROJECT_DIR}/Plugins/ACESimBridge/Binaries/Linux/libUnrealEditor-ACESimBridge.so"
