#!/usr/bin/env bash
set -euo pipefail

###############################################################################
#  Flo Simon Says – host-side auto-startup helper
#  → waits for Wi-Fi, then pulls code & fires up docker-compose
###############################################################################

# Log everything for post-mortem debugging
LOGFILE="$HOME/flo_game_startup.log"
exec &> >(tee -a "$LOGFILE")

run_privileged() {
  if [ "$(id -u)" -eq 0 ]; then
    "$@"
  else
    sudo -n "$@"
  fi
}

wait_for_path() {
  local label="$1"
  local path="$2"
  local attempts="${3:-30}"
  local sleep_seconds="${4:-1}"

  for ((i = 1; i <= attempts; i++)); do
    if [ -e "$path" ]; then
      echo "$(date '+%F %T') [INFO] ${label} available at ${path}"
      return 0
    fi
    echo "$(date '+%F %T') [WARN] Waiting for ${label} at ${path} (${i}/${attempts})"
    sleep "$sleep_seconds"
  done

  echo "$(date '+%F %T') [ERROR] ${label} missing at ${path}"
  return 1
}

###############################################################################
# Wait for Wi-Fi connectivity
###############################################################################
MAX_WAIT=90          # seconds before we give up
CHECK_INTERVAL=3     # seconds between checks
ELAPSED=0

echo "$(date '+%F %T') [INFO] Waiting for Wi-Fi connectivity…"

while ! ping -q -c1 -W1 8.8.8.8 &>/dev/null; do
  sleep "$CHECK_INTERVAL"
  ELAPSED=$((ELAPSED + CHECK_INTERVAL))
  if (( ELAPSED >= MAX_WAIT )); then
    echo "$(date '+%F %T') [ERROR] No Internet after ${MAX_WAIT}s – aborting startup."
    exit 1
  fi
done

echo "$(date '+%F %T') [INFO] Wi-Fi is online – continuing startup."

###############################################################################
# Git repo settings (unchanged)
###############################################################################
REPO_URL="https://github.com/anht-nguyen/FloSystemV2_GameDemo.git"
CLONE_DIR="$HOME/FloSystemV2_GameDemo"

# Clone or update the repo
if [ ! -d "$CLONE_DIR/.git" ]; then
  echo "$(date '+%F %T') [INFO] Cloning repository into $CLONE_DIR..."
  git clone --branch master "$REPO_URL" "$CLONE_DIR" || {
    echo "$(date '+%F %T') [ERROR] Git clone failed."
    exit 1
  }
else
  echo "$(date '+%F %T') [INFO] Updating repository in $CLONE_DIR..."
  cd "$CLONE_DIR" || exit 1
  git fetch origin master && git reset --hard origin/master || {
    echo "$(date '+%F %T') [ERROR] Git pull/reset failed."
    exit 1
  }
fi

###############################################################################
# Launch container (unchanged logic)
###############################################################################
cd "$CLONE_DIR" || {
  echo "[ERROR] Could not cd into project directory"
  exit 1
}

echo "$(date '+%F %T') [INFO] Waiting for FLO udev symlinks..."
if ! wait_for_path "camera symlink" /dev/flo_camera 30 1 \
    || ! wait_for_path "motors symlink" /dev/flo_motors 30 1 \
    || ! wait_for_path "face symlink" /dev/flo_face 30 1; then
  echo "$(date '+%F %T') [WARN] FLO symlinks did not appear after waiting."
  echo "$(date '+%F %T') [WARN] Attempting a udev retrigger."
  if run_privileged udevadm control --reload-rules && run_privileged udevadm trigger; then
    wait_for_path "camera symlink" /dev/flo_camera 15 1
    wait_for_path "motors symlink" /dev/flo_motors 15 1
    wait_for_path "face symlink" /dev/flo_face 15 1
  else
    echo "$(date '+%F %T') [ERROR] Could not retrigger udev automatically."
    exit 1
  fi
fi

echo "$(date '+%F %T') [INFO] Starting FloSystemV2 Simon Says container..."

# Ensure current user is in docker group
if ! id -nG "$USER" | grep -qw docker; then
  echo "[WARN] User '$USER' not in 'docker' group; this may require sudo rights."
fi

# Export GUI and audio environment for ROS inside container
export DISPLAY=:0
export UID
export GID="$(id -g)"
export XAUTHORITY="$HOME/.Xauthority"
export XDG_RUNTIME_DIR="/run/user/$(id -u)"
export PULSE_SERVER="unix:$XDG_RUNTIME_DIR/pulse/native"
export PULSE_DIR="$XDG_RUNTIME_DIR/pulse"
eval "$(python3 "$CLONE_DIR/utility/device_paths.py" --format shell)"

# Build and run in detached mode
/usr/bin/docker compose up --build -d || {
  echo "$(date '+%F %T') [ERROR] Failed to start the container."
  exit 1
}

echo "$(date '+%F %T') [INFO] Container launched successfully."
