#!/usr/bin/env bash
set -euo pipefail

# ==============================================================================
# install_gz_physics_6.sh
#
# Ubuntu 22.04 (Jammy) + ROS 2 Humble environment:
# - Install Ignition Gazebo Fortress (ign gazebo)
# - Build + install gz-physics6 from source into /usr (Option B)
# - Use checkinstall to generate a .deb and install it via dpkg
# - Enable SDF / Bullet / Bullet-Featherstone / TPE / DARTSim plugins by ensuring
#   sdformat13 and DART component dependencies are installed
#
# NOTE (Option B / replacement):
# - This script REMOVES apt-managed "libgz-physics6*" packages to avoid dpkg file
#   conflicts, then installs the source-built version under /usr.
# - This may break other packages that expect distro-provided libgz-physics6*.
#
# Clone destination (fixed):
#   ~/lib/gz/gz-physics
#
# Also configures:
#   IGN_GAZEBO_PHYSICS_ENGINE_PATH=/usr/lib/x86_64-linux-gnu/gz-physics-6/engine-plugins
# so that "ign gazebo" can find the physics engine plugins.
# ==============================================================================

# -----------------------------
# User settings
# -----------------------------
CLONE_DIR="${HOME}/lib/gz/gz-physics"
GIT_REF="gz-physics6"                 # branch or tag; must exist
INSTALL_PREFIX="/usr"

PKG_NAME="libgz-physics6"
PKG_VERSION="6.7.0"                   # should match cmake's reported version
PKG_RELEASE="1"

ENGINE_PLUGIN_DIR="/usr/lib/x86_64-linux-gnu/gz-physics-6/engine-plugins"

# If you already manage this repo elsewhere, set to "false"
ADD_OSRF_REPO="true"

# -----------------------------
# Helpers
# -----------------------------
log() { printf "\n[+] %s\n" "$*"; }

# -----------------------------
# 0) Base tools
# -----------------------------
log "Installing base tools (git/cmake/checkinstall, etc.)"
sudo apt update
sudo apt install -y \
  gnupg lsb-release wget \
  git cmake build-essential \
  pkg-config checkinstall \
  doxygen

# -----------------------------
# 1) OSRF Gazebo repository (needed for many Gazebo/Ignition packages)
# -----------------------------
if [[ "${ADD_OSRF_REPO}" == "true" ]]; then
  log "Configuring OSRF Gazebo apt repository"
  sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" > /etc/apt/sources.list.d/gazebo-stable.list'
  # NOTE: apt-key is deprecated but used here for compatibility with OSRF instructions.
  wget -qO- http://packages.osrfoundation.org/gazebo.key | sudo apt-key add -
  sudo apt update
else
  log "Skipping OSRF Gazebo apt repository setup (ADD_OSRF_REPO=false)"
fi

# -----------------------------
# 2) Install Ignition Gazebo Fortress (ROS 2 Humble default sim)
# -----------------------------
log "Installing Ignition Gazebo Fortress (ign gazebo)"
sudo apt install -y ignition-fortress

# -----------------------------
# 3) Install build dependencies for gz-physics6 (including SDF + DARTSim extras)
# -----------------------------
log "Installing build dependencies for gz-physics6 (sdformat13 + DARTSim extras)"
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
  libdart-dev \
  libdart-collision-bullet-dev \
  libdart-collision-ode-dev \
  libdart-utils-dev \
  libdart-utils-urdf-dev

# -----------------------------
# 4) Clone or update source tree (fixed path)
# -----------------------------
log "Preparing source tree at ${CLONE_DIR}"
mkdir -p "$(dirname "${CLONE_DIR}")"

if [[ -d "${CLONE_DIR}/.git" ]]; then
  log "Repository exists; fetching updates"
  git -C "${CLONE_DIR}" fetch --all --tags
else
  log "Cloning gz-physics into ${CLONE_DIR}"
  git clone https://github.com/gazebosim/gz-physics "${CLONE_DIR}"
fi

# -----------------------------
# 5) Explicit checkout (critical)
# -----------------------------
log "Checking out ${GIT_REF}"
cd "${CLONE_DIR}"
git checkout "${GIT_REF}"
# If this is a branch, update it; if it's a tag, pull will fail harmlessly.
git pull --ff-only || true

# -----------------------------
# 6) Clean build directory (important after dependency changes)
# -----------------------------
log "Cleaning build directory"
rm -rf build
mkdir -p build
cd build

# -----------------------------
# 7) Configure (tests OFF to avoid test build failures)
# -----------------------------
log "Configuring CMake (Release, prefix=${INSTALL_PREFIX}, BUILD_TESTING=OFF)"
cmake .. \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="${INSTALL_PREFIX}" \
  -DBUILD_TESTING=OFF

# -----------------------------
# 8) Build
# -----------------------------
log "Building"
make -j"$(nproc)"

# -----------------------------
# 9) Option B: remove apt-managed libgz-physics6* to avoid dpkg file conflicts
# -----------------------------
log "Removing apt-managed libgz-physics6* packages (Option B)"
sudo apt remove -y 'libgz-physics6*' || true
sudo apt autoremove -y || true

# -----------------------------
# 10) Ensure install destination directory exists (prevents tag install errors)
# -----------------------------
log "Ensuring /usr/share/gz/gz-physics6 exists"
sudo mkdir -p /usr/share/gz/gz-physics6

# -----------------------------
# 11) Create .deb with checkinstall (do NOT auto-install from checkinstall)
# -----------------------------
log "Creating Debian package with checkinstall (install=no), then install via dpkg"
# Use "yes" to accept defaults and avoid interactive prompts.
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
# 12) Install generated .deb using dpkg (then fix dependencies)
# -----------------------------
log "Installing generated .deb via dpkg"
DEB_FILE="$(ls -1t ./*.deb | head -n 1)"
echo "[+] Using package: ${DEB_FILE}"

sudo dpkg -i "${DEB_FILE}" || true
sudo apt -f install -y
sudo ldconfig

# -----------------------------
# 13) Configure Ignition Gazebo to find physics engine plugins
# -----------------------------
log "Configuring IGN_GAZEBO_PHYSICS_ENGINE_PATH for ign gazebo"
if [[ -d "${ENGINE_PLUGIN_DIR}" ]]; then
  # Export for the current shell (useful for immediate testing)
  export IGN_GAZEBO_PHYSICS_ENGINE_PATH="${ENGINE_PLUGIN_DIR}"
  echo "[+] Exported IGN_GAZEBO_PHYSICS_ENGINE_PATH=${IGN_GAZEBO_PHYSICS_ENGINE_PATH}"

  # Persist system-wide
  echo "export IGN_GAZEBO_PHYSICS_ENGINE_PATH=${ENGINE_PLUGIN_DIR}" | \
    sudo tee /etc/profile.d/ign-gazebo-physics-engine-path.sh >/dev/null
else
  echo "[!] Engine plugin directory not found: ${ENGINE_PLUGIN_DIR}"
  echo "[!] Adjust ENGINE_PLUGIN_DIR if your multiarch path differs."
fi

# -----------------------------
# 14) Verification
# -----------------------------
log "Verification"
echo "[+] ign gazebo:"
which ign || true
ign gazebo --help | head -n 5 || true

echo "[+] gz-physics6 package:"
dpkg -l | grep -E "^ii\s+${PKG_NAME}\b" || true
dpkg -S /usr/lib/x86_64-linux-gnu/libgz-physics6.so 2>/dev/null || true
pkg-config --modversion gz-physics6 2>/dev/null || true

echo "[+] Engine plugins:"
ls -l "${ENGINE_PLUGIN_DIR}" 2>/dev/null || true

log "Done."
echo
echo "[+] Minimal runtime test (creates a tiny SDF and runs ign gazebo):"
echo "    cat > /tmp/test_physics.sdf <<'EOF'"
echo "    <?xml version=\"1.0\" ?>"
echo "    <sdf version=\"1.8\">"
echo "      <world name=\"default\">"
echo "        <plugin filename=\"ignition-gazebo-physics-system\" name=\"ignition::gazebo::systems::Physics\">"
echo "          <engine>"
echo "            <filename>gz-physics6-bullet-featherstone-plugin</filename>"
echo "          </engine>"
echo "        </plugin>"
echo "      </world>"
echo "    </sdf>"
echo "    EOF"
echo "    ign gazebo -v 4 -r /tmp/test_physics.sdf"
