#!/usr/bin/env bash
# Record the demo (Hyprland/Wayland). Stop with Ctrl+C.
set -euo pipefail

OUT="${1:-docs/demo-$(date +%Y%m%d-%H%M%S).mp4}"
mkdir -p "$(dirname "$OUT")"

if ! command -v wf-recorder >/dev/null 2>&1; then
    echo "wf-recorder not found. Install it with:"
    echo "  sudo pacman -S --needed wf-recorder"
    exit 1
fi

echo "Recording to $OUT  (Ctrl+C to stop)"
exec wf-recorder -f "$OUT"
