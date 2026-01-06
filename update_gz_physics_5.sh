#!/usr/bin/env bash
set -euo pipefail

# -----------------------------------------------------------------------------
# update_ign_physics_5.sh
#
# Purpose:
#   Rebuild locally modified ign-physics5 source and update the installed
#   dpkg-managed package by generating a new .deb with checkinstall and installing
#   it via dpkg -i (so you can see install logs).
#
# Key points:
#   - Non-interactive checkinstall (--default) + auto-empty description.
#   - Creates a unique local version string so dpkg upgrades cleanly.
#   - Installs via dpkg -i (prints install logs).
# -----------------------------------------------------------------------------

REPO_DIR="${HOME}/lib/gz/gz-physics"
BUILD_DIR="${REPO_DIR}/build"
INSTALL_PREFIX="/usr"

if [[ ! -d "${REPO_DIR}/.git" ]]; then
  echo "[!] Repo not found: ${REPO_DIR}"
  echo "    Run install_gz_physics_5.sh first."
  exit 1
fi

cd "${REPO_DIR}"

# Derive a dpkg-friendly local version:
#   <base>+localYYYYMMDD.HHMM.g<sha>
# Base version will be derived after cmake configure; sha/date are from git/time.
GIT_SHA="$(git rev-parse --short HEAD)"
STAMP="$(date +%Y%m%d.%H%M)"
LOCAL_SUFFIX="+local${STAMP}.g${GIT_SHA}"

echo "[+] Cleaning build directory: ${BUILD_DIR}"
rm -rf "${BUILD_DIR}"
mkdir -p "${BUILD_DIR}"
cd "${BUILD_DIR}"

echo "[+] Configuring CMake"
cmake .. \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="${INSTALL_PREFIX}" \
  -DBUILD_TESTING=OFF

BASE_VERSION="$(grep -E '^PROJECT_VERSION:STATIC=' CMakeCache.txt | cut -d= -f2 || true)"
if [[ -z "${BASE_VERSION}" ]]; then
  BASE_VERSION="$(grep -E 'VERSION:STATIC=' CMakeCache.txt | head -n1 | cut -d= -f2 || true)"
fi
if [[ -z "${BASE_VERSION}" ]]; then
  echo "[!] Could not derive PROJECT_VERSION from CMakeCache.txt"
  BASE_VERSION="5.0.0"
fi

PKG_VERSION="${BASE_VERSION}${LOCAL_SUFFIX}"
echo
echo "[+] Package version: ${PKG_VERSION}"
echo

echo "[+] Building"
make -j"$(nproc)"

echo "[+] Ensuring documentation destination exists"
sudo mkdir -p /usr/share/ignition/ign-physics5

echo "[+] Creating Debian package with checkinstall (install=no)"
# We create the .deb first (install=no), then install with dpkg -i to show install logs clearly.
sudo checkinstall \
  --type=debian \
  --default \
  --fstrans=no \
  --install=no \
  -D \
  --pkgname=libignition-physics5 \
  --pkgversion="${PKG_VERSION}" \
  --pkgrelease=1 \
  --provides=libignition-physics5 \
  make install

DEB_PATH="$(ls -1 "${BUILD_DIR}"/libignition-physics5_"${PKG_VERSION}"-1_*.deb 2>/dev/null | tail -n1 || true)"
if [[ -z "${DEB_PATH}" ]]; then
  # Fallback: find latest matching deb.
  DEB_PATH="$(ls -1t "${BUILD_DIR}"/libignition-physics5_*_*.deb 2>/dev/null | head -n1 || true)"
fi

if [[ -z "${DEB_PATH}" ]]; then
  echo "[!] Could not find generated .deb under: ${BUILD_DIR}"
  exit 1
fi

echo
echo "[+] Installing package via dpkg (showing install logs)"
sudo dpkg -i "${DEB_PATH}"

echo
echo "[+] Done."
echo "    Installed package:"
dpkg -l | grep -E 'libignition-physics5' || true
echo
echo "    Physics engine plugins directory (Fortress/ign gazebo typically uses this):"
ls -l /usr/lib/x86_64-linux-gnu/ign-physics-5/engine-plugins 2>/dev/null || true

