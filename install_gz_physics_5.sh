#!/usr/bin/env bash
set -euo pipefail

# -----------------------------------------------------------------------------
# install_gz_physics_5.sh
#
# Purpose:
#   Build and install Ignition Physics 5 ("ign-physics5") from source on Ubuntu 22.04,
#   packaging the install with checkinstall so it is tracked by dpkg.
#
# Notes:
#   - This script is intended for Ignition Fortress users (ign gazebo).
#   - It installs into /usr (system prefix).
#   - It avoids "make install" unmanaged installs by creating a .deb via checkinstall.
#   - It is non-interactive (checkinstall --default + empty description injected).
# -----------------------------------------------------------------------------

REPO_DIR="${HOME}/lib/gz/gz-physics"
BUILD_DIR="${REPO_DIR}/build"
INSTALL_PREFIX="/usr"

# You can change this repo if you maintain a fork.
REPO_URL="https://github.com/gazebosim/gz-physics.git"

echo "[+] Installing build prerequisites"
sudo apt update
sudo apt install -y \
  git ca-certificates \
  build-essential cmake pkg-config ninja-build \
  checkinstall \
  doxygen graphviz \
  libeigen3-dev \
  libsdformat13-dev \
  libbullet-dev \
  libdart-dev \
  libfcl-dev libccd-dev liboctomap-dev \
  libgz-cmake3-dev \
  libgz-common5-dev \
  libgz-math7-dev \
  libgz-plugin2-dev \
  libgz-utils2-dev

echo "[+] Preparing source directory: ${REPO_DIR}"
mkdir -p "$(dirname "${REPO_DIR}")"
if [[ ! -d "${REPO_DIR}/.git" ]]; then
  git clone "${REPO_URL}" "${REPO_DIR}"
fi

cd "${REPO_DIR}"

echo "[+] Fetching latest refs"
git fetch --all --tags --prune

echo "[+] Checking out an ign-physics5-compatible ref"
# Try common branch names first. If they don't exist, keep current branch.
if git show-ref --verify --quiet "refs/remotes/origin/ign-physics5"; then
  git checkout -B ign-physics5 origin/ign-physics5
elif git show-ref --verify --quiet "refs/remotes/origin/ignition-physics5"; then
  git checkout -B ignition-physics5 origin/ignition-physics5
else
  echo "[!] No explicit ign-physics5 branch found. Using current default branch."
  echo "    (If this doesn't match your Fortress stack, set REPO_URL to your fork/branch.)"
fi

echo "[+] Creating build directory: ${BUILD_DIR}"
mkdir -p "${BUILD_DIR}"
cd "${BUILD_DIR}"

echo "[+] Configuring CMake"
cmake .. \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="${INSTALL_PREFIX}" \
  -DBUILD_TESTING=OFF

# Derive the project version from CMakeCache after configure.
PROJECT_VERSION="$(grep -E '^PROJECT_VERSION:STATIC=' CMakeCache.txt | cut -d= -f2 || true)"
if [[ -z "${PROJECT_VERSION}" ]]; then
  # Fallback: try any *_VERSION entry.
  PROJECT_VERSION="$(grep -E 'VERSION:STATIC=' CMakeCache.txt | head -n1 | cut -d= -f2 || true)"
fi
if [[ -z "${PROJECT_VERSION}" ]]; then
  echo "[!] Could not derive PROJECT_VERSION from CMakeCache.txt"
  PROJECT_VERSION="5.0.0"
fi
echo "[+] Project version: ${PROJECT_VERSION}"

echo "[+] Building"
make -j"$(nproc)"

# Ensure tag/doc destination exists (some cmake installs expect this directory tree).
echo "[+] Ensuring documentation destination exists"
sudo mkdir -p /usr/share/ignition/ign-physics5

echo "[+] Packaging + Installing with checkinstall (tracked by dpkg)"
# checkinstall always asks for a package description; feed an empty line to finish immediately.
# Use --install=yes so the resulting .deb is installed automatically.
sudo checkinstall \
  --type=debian \
  --default \
  --fstrans=no \
  --install=yes \
  -D \
  --pkgname=libignition-physics5 \
  --pkgversion="${PROJECT_VERSION}" \
  --pkgrelease=1 \
  --provides=libignition-physics5 \
  make install

echo "[+] Done."
echo "    Verify:"
echo "      dpkg -l | grep -E 'libignition-physics5|ignition-physics5' || true"
echo "      ls -l /usr/lib/x86_64-linux-gnu/ign-physics-5/engine-plugins || true"

