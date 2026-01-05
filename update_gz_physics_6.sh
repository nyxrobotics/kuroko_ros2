#!/usr/bin/env bash
set -euo pipefail

# ==============================================================================
# update_gz_physics_6.sh
#
# Rebuild and update gz-physics6 AFTER you modify the source code.
#
# Environment:
# - Ubuntu 22.04 (Jammy)
# - ROS 2 Humble
# - Ignition Gazebo Fortress (ign gazebo) is assumed to be installed already
#
# Assumptions:
# - gz-physics is cloned at: ~/lib/gz/gz-physics
# - Installation prefix is /usr (Option B)
# - You already replaced apt-managed libgz-physics6* once (install script)
#
# Behavior:
# - Does NOT run git pull / checkout (local edits are preserved)
# - Cleans and rebuilds the project
# - Generates a new .deb with a unique local version string
# - Reinstalls it via dpkg
#
# Notes:
# - Runtime environment variables (e.g., IGN_GAZEBO_PHYSICS_ENGINE_PATH) are NOT
#   handled here because they are unrelated to rebuilding after code changes.
# ==============================================================================

# -----------------------------
# Paths and package info
# -----------------------------
SRC_DIR="${HOME}/lib/gz/gz-physics"
BUILD_DIR="${SRC_DIR}/build"
INSTALL_PREFIX="/usr"

PKG_NAME="libgz-physics6"
PKG_RELEASE="1"

# -----------------------------
# Helpers
# -----------------------------
log() { printf "\n[+] %s\n" "$*"; }

# -----------------------------
# 0) Sanity checks
# -----------------------------
if [[ ! -d "${SRC_DIR}/.git" ]]; then
  echo "[!] gz-physics repository not found at:"
  echo "    ${SRC_DIR}"
  echo "[!] Expected a clone at ~/lib/gz/gz-physics"
  exit 1
fi

# Optional: ensure required tools exist (best-effort)
command -v cmake >/dev/null || { echo "[!] cmake not found"; exit 1; }
command -v make  >/dev/null || { echo "[!] make not found"; exit 1; }
command -v checkinstall >/dev/null || { echo "[!] checkinstall not found"; exit 1; }

# -----------------------------
# 1) Derive a unique package version automatically
# -----------------------------
# Format:
#   <upstream_version>+local<YYYYMMDD.HHMM>.g<gitsha>
#
# Example:
#   6.7.0+local20260105.1832.g1a2b3c4
#
log "Deriving package version"

BASE_VER="$(
  grep -E 'project\(gz-physics[0-9]*[[:space:]]+VERSION[[:space:]]+' \
    "${SRC_DIR}/CMakeLists.txt" \
    | sed -n 's/.*VERSION[[:space:]]\+\([0-9]\+\.[0-9]\+\.[0-9]\+\).*/\1/p' \
    | head -n 1
)"

if [[ -z "${BASE_VER}" ]]; then
  # Fallback: keep a sensible default if parsing fails.
  BASE_VER="6.7.0"
fi

GIT_SHA="$(git -C "${SRC_DIR}" rev-parse --short HEAD 2>/dev/null || echo unknown)"
TS="$(date +%Y%m%d.%H%M)"
PKG_VERSION="${BASE_VER}+local${TS}.g${GIT_SHA}"

log "Package version: ${PKG_VERSION}"

# -----------------------------
# 2) Clean build directory
# -----------------------------
log "Cleaning build directory: ${BUILD_DIR}"
rm -rf "${BUILD_DIR}"
mkdir -p "${BUILD_DIR}"
cd "${BUILD_DIR}"

# -----------------------------
# 3) Configure
# -----------------------------
log "Configuring CMake"
cmake .. \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="${INSTALL_PREFIX}" \
  -DBUILD_TESTING=OFF

# -----------------------------
# 4) Build
# -----------------------------
log "Building"
make -j"$(nproc)"

# -----------------------------
# 5) Ensure install destination exists (prevents tag install errors)
# -----------------------------
log "Ensuring /usr/share/gz/gz-physics6 exists"
sudo mkdir -p /usr/share/gz/gz-physics6

# -----------------------------
# 6) Create .deb with checkinstall (no auto-install)
# -----------------------------
log "Creating Debian package with checkinstall (install=no)"
# Non-interactive defaults:
sudo checkinstall \
  --default \
  --type=debian \
  --install=no \
  --nodoc \
  --fstrans=no \
  --pkgname="${PKG_NAME}" \
  --pkgversion="${PKG_VERSION}" \
  --pkgrelease="${PKG_RELEASE}" \
  --provides="${PKG_NAME}" \
  --backup=no \
  make install

# -----------------------------
# 7) Install the generated .deb (upgrade in place)
# -----------------------------
log "Installing generated .deb via dpkg"
DEB_FILE="$(ls -1t ./*.deb | head -n 1)"
echo "[+] Using package: ${DEB_FILE}"

sudo dpkg -i "${DEB_FILE}" || true
sudo apt -f install -y
sudo ldconfig

# -----------------------------
# 8) Verification
# -----------------------------
log "Verification"
dpkg -l | grep -E "^ii\s+${PKG_NAME}\b" || true
dpkg -S /usr/lib/x86_64-linux-gnu/libgz-physics6.so 2>/dev/null || true
pkg-config --modversion gz-physics6 2>/dev/null || true

log "Update complete."
