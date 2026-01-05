#!/usr/bin/env bash
set -euo pipefail

# ==============================================================================
# Build + install gz-physics6 from source on Ubuntu 22.04 (Jammy)
# using checkinstall (Option B: replace apt-managed gz-physics6 under /usr).
#
# - ROS 2 Humble / Gazebo Fortress family
# - Clone destination: ~/lib/gz/gz-physics
# - Explicit git checkout to gz-physics6 branch
# - sdformat13 + full DARTSim support enabled
# - checkinstall used to create a .deb, installed via dpkg
# ==============================================================================

# -----------------------------
# User settings
# -----------------------------
CLONE_DIR="${HOME}/lib/gz/gz-physics"
GIT_REF="gz-physics6"        # branch or tag (EXPLICIT checkout)
INSTALL_PREFIX="/usr"

PKG_NAME="libgz-physics6"
PKG_VERSION="6.7.0"          # must match cmake-reported version
PKG_RELEASE="1"

ADD_OSRF_REPO="true"

# -----------------------------
# Helper
# -----------------------------
log() { printf "\n[+] %s\n" "$*"; }

# -----------------------------
# 0) Base tools
# -----------------------------
log "Installing base build tools"
sudo apt update
sudo apt install -y \
  gnupg lsb-release wget \
  git cmake build-essential checkinstall

# -----------------------------
# 1) OSRF Gazebo repository
# -----------------------------
if [[ "${ADD_OSRF_REPO}" == "true" ]]; then
  log "Configuring OSRF Gazebo apt repository"
  sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
    > /etc/apt/sources.list.d/gazebo-stable.list'
  wget -qO- http://packages.osrfoundation.org/gazebo.key | sudo apt-key add -
  sudo apt update
fi

# -----------------------------
# 2) Build dependencies (discovered during troubleshooting)
# -----------------------------
log "Installing build dependencies (sdformat13 + full DART support)"
sudo apt install -y \
  libgz-cmake3-dev \
  libgz-common5-dev \
  libgz-common5-graphics-dev \
  libgz-common5-geospatial-dev \
  libgz-common5-profiler-dev \
  libgz-math7-dev \
  libgz-plugin2-dev \
  libgz-utils2-dev \
  libsdformat13-dev \
  libbullet-dev \
  doxygen \
  libdart-dev \
  libdart-collision-bullet-dev \
  libdart-collision-ode-dev \
  libdart-utils-dev \
  libdart-utils-urdf-dev

# -----------------------------
# 3) Clone or update source
# -----------------------------
log "Preparing source tree at ${CLONE_DIR}"
mkdir -p "$(dirname "${CLONE_DIR}")"

if [[ -d "${CLONE_DIR}/.git" ]]; then
  log "Updating existing repository"
  git -C "${CLONE_DIR}" fetch --all --tags
else
  git clone https://github.com/gazebosim/gz-physics "${CLONE_DIR}"
fi

# -----------------------------
# 4) EXPLICIT checkout (FIX)
# -----------------------------
log "Checking out ${GIT_REF}"
cd "${CLONE_DIR}"
git checkout "${GIT_REF}"
git pull --ff-only || true

# -----------------------------
# 5) Clean build directory
# -----------------------------
log "Cleaning build directory"
rm -rf build
mkdir build
cd build

# -----------------------------
# 6) Configure
# -----------------------------
log "Configuring CMake"
cmake .. \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="${INSTALL_PREFIX}" \
  -DBUILD_TESTING=OFF

# -----------------------------
# 7) Build
# -----------------------------
log "Building"
make -j"$(nproc)"

# -----------------------------
# 8) Option B: remove apt-managed gz-physics6
# -----------------------------
log "Removing apt-managed libgz-physics6* packages"
sudo apt remove -y 'libgz-physics6*' || true
sudo apt autoremove -y || true

# -----------------------------
# 9) Ensure install directories exist
# -----------------------------
log "Ensuring /usr/share/gz/gz-physics6 exists"
sudo mkdir -p /usr/share/gz/gz-physics6

# -----------------------------
# 10) Create Debian package with checkinstall
# -----------------------------
log "Creating .deb with checkinstall (no auto-install)"
yes "" | sudo checkinstall \
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
# 11) Install generated .deb
# -----------------------------
log "Installing generated .deb via dpkg"
DEB_FILE="$(ls -1t ./*.deb | head -n 1)"
echo "[+] Using package: ${DEB_FILE}"

sudo dpkg -i "${DEB_FILE}" || true
sudo apt -f install -y
sudo ldconfig

# -----------------------------
# 12) Verification
# -----------------------------
log "Verification"
dpkg -l | grep -E "libgz-physics6" || true
dpkg -S /usr/lib/x86_64-linux-gnu/libgz-physics6.so 2>/dev/null || true
pkg-config --modversion gz-physics6 2>/dev/null || true

log "Done."

