# auto_startup_flogame_launcher.sh
#!/usr/bin/env bash
set -euo pipefail

# Where this script lives (project root)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

echo "$(date '+%F %T')  Starting FloSystemV2 Simon Says container..."

# Build (if changed) and run in detached mode
/usr/bin/docker compose up --build -d
if [ $? -ne 0 ]; then
    echo "$(date '+%F %T')  Error: Failed to start the container."
    exit 1
fi