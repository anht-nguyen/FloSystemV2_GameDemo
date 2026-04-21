#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Export GUI and audio environment for ROS inside the container.
export DISPLAY="${DISPLAY:-:0}"
export UID
export GID="${GID:-$(id -g)}"
export XAUTHORITY="${XAUTHORITY:-$HOME/.Xauthority}"
export XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/run/user/$(id -u)}"
export PULSE_SERVER="${PULSE_SERVER:-unix:${XDG_RUNTIME_DIR}/pulse/native}"
export PULSE_DIR="${PULSE_DIR:-${XDG_RUNTIME_DIR}/pulse}"
export FLO_RECORD_DEVICE="${FLO_RECORD_DEVICE:-pulse}"

eval "$(python3 "$SCRIPT_DIR/utility/device_paths.py" --format shell)"

cd "$SCRIPT_DIR"
exec /usr/bin/docker compose up --build -d "$@"
