#!/usr/bin/env bash
# Record one demo segment and keep every frame of it.
#
#   scripts/record_segment.sh static   clips/static
#   scripts/record_segment.sh dynamic  clips/dynamic
#
# The capture comes from an isolated X display (Xvfb :99 started inside the
# container), never from the host desktop: nothing private can end up in frame.
# The segment is a full mission: fresh container, fresh launch, RViz resized to
# exactly the capture size, ffmpeg grabbing that display from the host.
set -u

MODE=${1:-dynamic}
OUT=${2:-/tmp/opencode/clips/$MODE.mkv}
ROOT=$(cd "$(dirname "$0")/.." && pwd)
DC="docker compose -f $ROOT/docker/compose.yaml"
W=1280
H=720
FPS=25

# Deadline is wall seconds, the mission is simulation seconds, and RViz on
# llvmpipe drags the RTF down to ~0.3-0.5: a dynamic run is ~80 sim s, so 220
# wall seconds could cut the capture off before the robot arrives (and with it
# the hold -> mark -> replan event the video exists to show). The loop still
# breaks the moment the goal is reached, so a generous cap costs nothing.
if [ "$MODE" = dynamic ]; then DYN=true; DEADLINE=400; else DYN=false; DEADLINE=300; fi
mkdir -p "$(dirname "$OUT")"

in_display() { DISPLAY=:99 $DC exec -T amr bash -lc "export DISPLAY=:99; source /opt/ros/jazzy/setup.bash; source /work/install/setup.bash; cd /work; $1"; }

echo "== restart container"
$DC restart amr >/dev/null 2>&1
$DC exec -T amr mkdir -p /work/log >/dev/null 2>&1

echo "== isolated display :99 (1280x720), capture size $W x $H @ $FPS"
$DC exec -T -d amr bash -lc \
  "exec Xvfb :99 -screen 0 ${W}x${H}x24 -nolisten tcp > /work/log/xvfb.log 2>&1"
sleep 3

echo "== launch $MODE"
DISPLAY=:99 $DC exec -T -d -e DISPLAY=:99 amr bash -lc \
  "source /opt/ros/jazzy/setup.bash; source /work/install/setup.bash; cd /work; \
   exec ros2 launch amr_bringup demo.launch.py headless:=true rviz:=true dynamic:=$DYN \
   > /work/log/record_$MODE.log 2>&1"
sleep 35

echo "== fit RViz to the screen"
in_display 'win=$(xdotool search --name "RViz" | tail -1); \
  if [ -n "$win" ]; then xdotool windowmove "$win" 0 0; xdotool windowsize "$win" 1280 720; else echo "no RViz window"; fi' || true
sleep 4

echo "== capture -> $OUT"
ffmpeg -hide_banner -loglevel error -y -f x11grab -framerate $FPS \
  -video_size ${W}x${H} -i :99.0 -c:v libx264 -preset ultrafast -crf 18 \
  -pix_fmt yuv420p "$OUT" &
FFPID=$!
sleep 3

echo "== send goal (4, 4)"
in_display "ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: map}, pose: {position: {x: 4.0, y: 4.0}, orientation: {w: 1.0}}}'" >/dev/null 2>&1

for _ in $(seq 1 $((DEADLINE / 4))); do
  sleep 4
  if grep -q "Reached the goal" "$ROOT/log/record_$MODE.log" 2>/dev/null; then break; fi
done
sleep 6

kill -INT $FFPID 2>/dev/null
wait $FFPID 2>/dev/null
echo "== stop stack"
$DC exec -T amr bash -lc 'pkill -INT -f "demo[.]launch[.]py"; pkill -INT -f "Xvfb :99"' >/dev/null 2>&1
sleep 3

echo "== done: $(ls -la "$OUT" | awk '{print $5" bytes"}')"
grep -c "Reached the goal" "$ROOT/log/record_$MODE.log" 2>/dev/null || true