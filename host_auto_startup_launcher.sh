#!/usr/bin/env bash
set -euo pipefail

###############################################################################
#  Flo Simon Says – host-side auto-startup helper
#  → waits for Wi-Fi, then pulls code & fires up docker-compose
###############################################################################

# Log everything for post-mortem debugging
LOGFILE="$HOME/flo_game_startup.log"
exec &> >(tee -a "$LOGFILE")

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

echo "$(date '+%F %T') [INFO] Starting FloSystemV2 Simon Says container..."

# Ensure current user is in docker group
if ! id -nG "$USER" | grep -qw docker; then
  echo "[WARN] User '$USER' not in 'docker' group; this may require sudo rights."
fi

# Export GUI and audio environment for ROS inside container
export DISPLAY=:0
export XAUTHORITY="$HOME/.Xauthority"
export XDG_RUNTIME_DIR="/run/user/$(id -u)"
export PULSE_SERVER="unix:$XDG_RUNTIME_DIR/pulse/native"

# Build and run in detached mode
/usr/bin/docker compose up --build -d || {
  echo "$(date '+%F %T') [ERROR] Failed to start the container."
  exit 1
}

echo "$(date '+%F %T') [INFO] Container launched successfully."
