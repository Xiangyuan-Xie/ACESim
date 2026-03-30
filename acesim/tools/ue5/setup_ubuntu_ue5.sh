#!/usr/bin/env bash
set -euo pipefail

UE_ROOT="${UE_ROOT:-/tmp/ACESim-unreal}"
UE_REPO_URL="${UE_REPO_URL:-git@github.com:EpicGames/UnrealEngine.git}"
UE_REF="${UE_REF:-5.4.4-release}"
UE_SRC_DIR="${UE_SRC_DIR:-${UE_ROOT}/UnrealEngine}"
UE_PROJECT_DIR="${UE_PROJECT_DIR:-${UE_ROOT}/projects/ACESimUE}"
UE_LOG_DIR="${UE_LOG_DIR:-${UE_ROOT}/logs}"

if ! command -v sudo >/dev/null 2>&1; then
  echo "sudo is required to install Unreal Engine build dependencies on Ubuntu."
  exit 1
fi

if ! sudo -n true >/dev/null 2>&1; then
  echo "sudo requires an interactive password on this machine."
  echo "Please run this script manually in a terminal where you can enter your password."
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
if ! nvidia-smi >/dev/null 2>&1; then
  echo "NVIDIA driver is not healthy. Stop here until the graphics stack is fixed."
  echo "Hint: the kernel module may be loaded but /dev/nvidia* nodes are still missing."
  exit 2
fi

echo "[4/5] Cloning Unreal Engine source"
if ! git ls-remote --heads "${UE_REPO_URL}" "${UE_REF}" >/dev/null 2>&1; then
  echo "Cannot access ${UE_REPO_URL} (${UE_REF})."
  echo "Make sure this machine has EpicGames UnrealEngine repository access and valid Git credentials."
  exit 3
fi

if [ ! -d "${UE_SRC_DIR}/.git" ]; then
  git clone --branch "${UE_REF}" --depth 1 "${UE_REPO_URL}" "${UE_SRC_DIR}"
fi

cd "${UE_SRC_DIR}"
git lfs install
git fetch --tags --force
git checkout "${UE_REF}"
./Setup.sh
./GenerateProjectFiles.sh
make -j"$(nproc)" UnrealEditor

echo "[5/5] Generating ACESim UE project scaffold"
python3 /home/xxy/ACESim/acesim/tools/ue5/create_project_scaffold.py --project-root "${UE_PROJECT_DIR}" --overwrite

echo "UE source: ${UE_SRC_DIR}"
echo "UE project: ${UE_PROJECT_DIR}"
echo "Editor: ${UE_SRC_DIR}/Engine/Binaries/Linux/UnrealEditor"
