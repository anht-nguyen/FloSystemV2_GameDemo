#!/usr/bin/env bash
set -Eeuo pipefail

###############################################################################
#  Flo Simon Says – host-side auto-startup helper
#  → waits for Wi-Fi, then pulls code & fires up docker-compose
###############################################################################

# Log everything for post-mortem debugging
LOGFILE="$HOME/flo_game_startup.log"
exec &> >(tee -a "$LOGFILE")

notify_desktop() {
  local urgency="$1"
  local title="$2"
  local message="$3"

  if ! command -v notify-send >/dev/null 2>&1; then
    return 0
  fi

  DBUS_SESSION_BUS_ADDRESS="${DBUS_SESSION_BUS_ADDRESS:-unix:path=/run/user/$(id -u)/bus}" \
    notify-send -u "$urgency" "$title" "$message" || true
}

announce_step() {
  local message="$1"
  echo "$(date '+%F %T') [INFO] ${message}"
  notify_desktop normal "Auto bring up robot" "$message"
}

fail_with_notification() {
  local message="$1"
  echo "$(date '+%F %T') [ERROR] ${message}"
  notify_desktop critical "Auto bring up robot failed" "$message"
  exit 1
}

on_error() {
  local exit_code="$1"
  local line_no="$2"
  notify_desktop critical "Auto bring up robot failed" \
    "Startup stopped near line ${line_no}. Check ${LOGFILE} for details."
  exit "$exit_code"
}

trap 'on_error $? $LINENO' ERR
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

announce_step "Auto bring up robot"
announce_step "Waiting for Wi-Fi connectivity"

while ! ping -q -c1 -W1 8.8.8.8 &>/dev/null; do
  sleep "$CHECK_INTERVAL"
  ELAPSED=$((ELAPSED + CHECK_INTERVAL))
  if (( ELAPSED >= MAX_WAIT )); then
    fail_with_notification "No Internet after ${MAX_WAIT}s. Please check Wi-Fi."
  fi
done

announce_step "Wi-Fi is online"

###############################################################################
# Git repo settings (unchanged)
###############################################################################
REPO_URL="https://github.com/anht-nguyen/FloSystemV2_GameDemo.git"
CLONE_DIR="$HOME/FloSystemV2_GameDemo"

# Clone or update the repo
if [ ! -d "$CLONE_DIR/.git" ]; then
  announce_step "Cloning robot software"
  git clone --branch master "$REPO_URL" "$CLONE_DIR" || {
    fail_with_notification "Git clone failed. Check network and repository access."
  }
else
  announce_step "Updating robot software"
  cd "$CLONE_DIR" || exit 1
  git fetch origin master && git reset --hard origin/master || {
    fail_with_notification "Git update failed. Check network and repository access."
  }
fi

###############################################################################
# Launch container (unchanged logic)
###############################################################################
cd "$CLONE_DIR" || {
  fail_with_notification "Could not open the project directory."
}

announce_step "Checking robot USB devices"
if ! wait_for_path "camera symlink" /dev/flo_camera 30 1 \
    || ! wait_for_path "motors symlink" /dev/flo_motors 30 1 \
    || ! wait_for_path "face symlink" /dev/flo_face 30 1; then
  echo "$(date '+%F %T') [WARN] FLO symlinks did not appear after waiting."
  echo "$(date '+%F %T') [WARN] Attempting a udev retrigger."
  notify_desktop normal "Auto bring up robot" \
    "Devices not found yet. Retrying udev and checking USB devices again."
  if run_privileged udevadm control --reload-rules && run_privileged udevadm trigger; then
    wait_for_path "camera symlink" /dev/flo_camera 15 1
    wait_for_path "motors symlink" /dev/flo_motors 15 1
    wait_for_path "face symlink" /dev/flo_face 15 1
  else
    fail_with_notification "Devices not found. Please check the USB cables and plug in the robot USB devices."
  fi
fi

announce_step "Robot devices detected"
announce_step "Starting robot Docker container"

# Ensure current user is in docker group
if ! id -nG "$USER" | grep -qw docker; then
  echo "[WARN] User '$USER' not in 'docker' group; this may require sudo rights."
  notify_desktop normal "Auto bring up robot" \
    "User '$USER' is not in the docker group. Startup may require sudo rights."
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
  fail_with_notification "Failed to start the robot container. Check ${LOGFILE} for details."
}

announce_step "Robot container launched successfully"
