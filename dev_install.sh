#!/usr/bin/env bash
#
#  dev_install.sh ─ FloSystemV2 developer bootstrap (Ubuntu 22.04)
#
#  ▸ Installs Docker & X helpers (for container runs / RViz).
#  ▸ Installs ROS-Noetic *binaries* even on Jammy by pinning Focal repos
#    (Noetic went EOL on 31 May 2025 but binaries still build for Focal).   :contentReference[oaicite:0]{index=0}
#  ▸ Creates ~/catkin_ws, links the FloSystemV2 source tree, resolves rosdeps.
#  ▸ Adds Astra 3-D camera stack (OpenNI2 + ros_astra_camera + Orbbec SDK).
#  ▸ Leaves out: RealSense, rosbridge_suite, webrtc_ros, LetsEncrypt helpers, etc.

set -euo pipefail

# ───────────────────────────── helper fns ──────────────────────────────
need_root() {
  [[ $EUID -eq 0 ]] || {
    echo "❌  Run this script with sudo:" >&2
    echo "    sudo $0" >&2
    exit 1
  }
}

apt_install() { apt-get -qq install -y "$@"; }

# ───────────────────────────── 1. base OS pkgs ─────────────────────────
need_root
echo "📦  Updating APT cache & upgrading..."
apt-get -qq update && apt-get -qq dist-upgrade -y

echo "📦  Installing Docker + Compose + X11 utils + dev tools..."
apt_install docker.io docker-compose-plugin x11-xserver-utils lsyncd \
            build-essential python3-pip curl git

# make docker usable without sudo
if ! id -nG "${SUDO_USER:-$USER}" | grep -qw docker ; then
  usermod -aG docker "${SUDO_USER:-$USER}"
  NEED_NEWGRP=1
fi

# ───────────────────────────── 2. ROS-Noetic ───────────────────────────
echo "🐢  Installing ROS-Noetic desktop-full…"
UBU_CODENAME=$(lsb_release -cs)           # "jammy" on 22.04
ROS_APT_CODENAME="focal"                  # Noetic binary repo target
sh -c "echo 'deb http://packages.ros.org/ros/ubuntu ${ROS_APT_CODENAME} main' \
      > /etc/apt/sources.list.d/ros-latest.list"                     :contentReference[oaicite:1]{index=1}
apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' \
           --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654        :contentReference[oaicite:2]{index=2}
# pin Focal packages higher than Jammy to avoid mixing repos
cat >/etc/apt/preferences.d/ros-noetic <<EOF
Package: *
Pin: release n=${ROS_APT_CODENAME}
Pin-Priority: 600
EOF

apt-get -qq update
apt_install ros-noetic-desktop-full

# core build tools
apt_install python3-rosdep python3-catkin-tools \
            python3-rosinstall python3-wstool ros-noetic-rgbd-launch

# initialise rosdep
[[ -f /etc/ros/rosdep/sources.list.d/20-default.list ]] || rosdep init -q
rosdep update -q --include-eol-distros

# ───────────────────────────── 3. Astra stack ──────────────────────────
echo "🎥  Installing Astra camera support…"

# 3.1 runtime deps via apt (OpenNI2)
apt_install ros-noetic-openni2-camera ros-noetic-openni2-launch \
            libopenni2-0 libopenni-dev                               :contentReference[oaicite:3]{index=3}

# 3.2 clone ros_astra_camera into catkin_ws
WS_ROOT=${WS_ROOT:-$HOME/catkin_ws}
mkdir -p "${WS_ROOT}/src"
if [[ ! -d "${WS_ROOT}/src/ros_astra_camera" ]]; then
  git clone https://github.com/orbbec/ros_astra_camera.git \
            "${WS_ROOT}/src/ros_astra_camera"                       :contentReference[oaicite:4]{index=4}
fi

# 3.3 optional: Orbbec SDK (binary .deb)
ASTRA_SDK_VER=${ASTRA_SDK_VER:-1.2.15}
ASTRA_DEB="/tmp/OrbbecSDK_v${ASTRA_SDK_VER}_amd64.deb"
if [[ ! -f /usr/local/lib/libOrbbecSDK.so* ]]; then
  echo "⬇️   Downloading Orbbec SDK ${ASTRA_SDK_VER}…"
  curl -L -o "${ASTRA_DEB}" \
       "https://github.com/orbbec/OrbbecSDK/releases/download/v${ASTRA_SDK_VER}/orbbecsdk_${ASTRA_SDK_VER}_amd64.deb" \
       && dpkg -i "${ASTRA_DEB}" || apt-get -f install -y           :contentReference[oaicite:5]{index=5}
fi

# ───────────────────────────── 4. Workspace build ──────────────────────
echo "🔧  Building catkin workspace…"
if [[ ! -L ${WS_ROOT}/src/FloSystemV2_GameDemo ]]; then
  ln -sf "$HOME/Documents/git/FloSystemV2_GameDemo" "${WS_ROOT}/src/"
fi                                                             :contentReference[oaicite:6]{index=6}

cd "${WS_ROOT}"
rosdep install --from-paths src --ignore-src -r -y
catkin build

# ───────────────────────────── 5. Finishing up ─────────────────────────
echo "✅  Installing Python libs (mutagen pin for py2 compat)…"
pip3 install --quiet 'mutagen==1.43.0'                            :contentReference[oaicite:7]{index=7}

echo "✅  Installation complete."
if [[ ${NEED_NEWGRP:-0} -eq 1 ]]; then
  echo "ℹ️   Docker group added. Run 'newgrp docker' or log out/in for it to take effect."
fi
echo "ℹ️   Source your workspace with:"
echo "       source /opt/ros/noetic/setup.bash && source ${WS_ROOT}/devel/setup.bash"
