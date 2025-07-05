#!/usr/bin/env bash
set -euo pipefail

# Log everything to a logfile for diagnostics
LOGFILE="$HOME/flo_game_startup.log"
exec &> >(tee -a "$LOGFILE")

# Enter project directory
cd "$HOME/Documents/git/FloSystemV2_GameDemo" || {
  echo "[ERROR] Could not cd into project directory";
  exit 1;
}

echo "$(date '+%F %T')  [INFO] Starting FloSystemV2 Simon Says container..."

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
  echo "$(date '+%F %T')  [ERROR] Failed to start the container."
  exit 1
}

echo "$(date '+%F %T')  [INFO] Container launched successfully."