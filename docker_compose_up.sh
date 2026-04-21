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

auto_camera_source="$(python3 "$SCRIPT_DIR/utility/inspect_video_devices.py" --recommended-only 2>/dev/null || true)"
if [ -n "${auto_camera_source}" ]; then
  if [ "${FLO_CAMERA_SOURCE:-}" != "$auto_camera_source" ]; then
    echo "Auto-configuring FLO camera source: ${FLO_CAMERA_SOURCE:-unset} -> ${auto_camera_source}"
  fi
  export FLO_CAMERA_SOURCE="$auto_camera_source"
elif [ -n "${FLO_CAMERA_SOURCE:-}" ]; then
  echo "Keeping existing FLO camera source: ${FLO_CAMERA_SOURCE}"
else
  echo "No capture-capable /dev/video* node detected; docker compose will use its existing camera path settings."
fi

cd "$SCRIPT_DIR"
exec /usr/bin/docker compose up --build -d "$@"
