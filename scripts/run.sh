#!/usr/bin/env bash
# Start (or rebuild) the ROS 2 Jazzy container and drop into a shell.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$HERE/../docker"

# X11 access for the container (Hyprland/XWayland). Harmless if already granted.
if command -v xhost >/dev/null 2>&1; then
    xhost +local:docker >/dev/null 2>&1 || true
fi

docker compose up -d --build

# Source ROS and the workspace on entry.
docker compose exec amr bash -lc '
  source /opt/ros/jazzy/setup.bash
  [ -f /work/install/setup.bash ] && source /work/install/setup.bash
  cd /work
  exec bash
'
