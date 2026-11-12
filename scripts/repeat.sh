#!/usr/bin/env bash
# Repeat one scenario N times, one measured run per repetition.
#
#   scripts/repeat.sh static 10
#   scripts/repeat.sh dynamic 10
#
# Each repetition: fresh container (no leftover Gazebo/ROS), fresh launch, a
# fresh log_run CSV, then eval_metrics. Evidence: docs/metrics/repeat/.
# The CSVs stay in log/repeat/ (gitignored) — the eval files carry the numbers.
set -u

MODE=${1:-static}
N=${2:-10}
GOAL_X=${3:-4.0}
GOAL_Y=${4:-4.0}
ROOT=$(cd "$(dirname "$0")/.." && pwd)
OUT="$ROOT/docs/metrics/repeat"
RAW="$ROOT/log/repeat"
DC="docker compose -f $ROOT/docker/compose.yaml"
mkdir -p "$OUT"
# log/ is owned by root (container writes it): create the raw-run directory the
# same way, or the log_run child cannot open its CSV.
$DC exec -T amr mkdir -p /work/log/repeat >/dev/null 2>&1 || true

if [ "$MODE" = dynamic ]; then DYN=true; LOG_S=240; else DYN=false; LOG_S=200; fi

# Simulation seconds to let AMCL settle before the goal is published. This is
# deliberately measured on /clock, not with `sleep`: the mover is phase-locked
# to the absolute simulation clock (obstacle_mover.py: t % period), so a
# wall-clock wait made the encounter depend on how fast the host happened to
# run the simulator, and the dynamic scenario was not reproducible.
SETTLE_S=${SETTLE_S:-40}

run_in() { $DC exec -T amr bash -lc "source /opt/ros/jazzy/setup.bash; source /work/install/setup.bash; cd /work; $1"; }

# Block until the simulation clock reaches $1 seconds (not wall seconds).
wait_sim() {
  target=$1
  i=0
  while [ "$i" -lt 180 ]; do
    c=$(run_in "timeout 5 ros2 topic echo --once /clock --field clock 2>/dev/null | sed -n 's/^sec: //p'" 2>/dev/null | tr -dc '0-9')
    [ -n "$c" ] && [ "$c" -ge "$target" ] && return 0
    i=$((i + 1))
    sleep 2
  done
  echo "warn: simulation clock never reached ${target}s" >&2
  return 1
}

for i in $(seq 1 "$N"); do
  tag=$(printf '%s_%02d' "$MODE" "$i")
  echo "=== $tag ==="

  $DC restart amr >/dev/null 2>&1
  run_in "colcon build --symlink-install >/dev/null 2>&1"

  $DC exec -T -d amr bash -lc \
    "source /opt/ros/jazzy/setup.bash; source /work/install/setup.bash; cd /work; \
     exec ros2 launch amr_bringup demo.launch.py headless:=true rviz:=false dynamic:=$DYN \
     > /work/log/repeat_$tag.log 2>&1"
  wait_sim "$SETTLE_S"

  $DC exec -T -d amr bash -lc \
    "source /opt/ros/jazzy/setup.bash; source /work/install/setup.bash; cd /work; \
     exec python3 scripts/log_run.py /work/log/repeat/$tag.csv $LOG_S \
     > /work/log/logrun_$tag.log 2>&1"
  sleep 6

  run_in "ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
    '{header: {frame_id: map}, pose: {position: {x: $GOAL_X, y: $GOAL_Y}, orientation: {w: 1.0}}}'" \
    >/dev/null 2>&1

  # Wait for the goal, up to 300 s wall (the logger counts simulation seconds:
  # static ~95 sim s, dynamic ~140 sim s, and the sim runs at RTF 0.3-0.65).
  for _ in $(seq 1 60); do
    sleep 5
    if grep -q "Reached the goal" "$ROOT/log/repeat_$tag.log" 2>/dev/null; then break; fi
  done
  sleep 5
  run_in 'pkill -INT -f "log_run[.]py"' >/dev/null 2>&1
  sleep 4
  $DC exec -T amr bash -lc 'pkill -INT -f "demo[.]launch[.]py"' >/dev/null 2>&1
  sleep 3

  run_in "PYTHONPATH=src/amr_planning python3 scripts/eval_metrics.py \
    /work/log/repeat/$tag.csv --goal $GOAL_X $GOAL_Y --name '$tag'" > "$OUT/$tag.txt" 2>&1
  echo "$tag: $(tail -1 "$OUT/$tag.txt")"
done

echo "done: $OUT"