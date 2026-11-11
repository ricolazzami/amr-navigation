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

run_in() { $DC exec -T amr bash -lc "source /opt/ros/jazzy/setup.bash; source /work/install/setup.bash; cd /work; $1"; }

for i in $(seq 1 "$N"); do
  tag=$(printf '%s_%02d' "$MODE" "$i")
  echo "=== $tag ==="

  $DC restart amr >/dev/null 2>&1
  run_in "colcon build --symlink-install >/dev/null 2>&1"

  $DC exec -T -d amr bash -lc \
    "source /opt/ros/jazzy/setup.bash; source /work/install/setup.bash; cd /work; \
     exec ros2 launch amr_bringup demo.launch.py headless:=true rviz:=false dynamic:=$DYN \
     > /work/log/repeat_$tag.log 2>&1"
  sleep 40

  $DC exec -T -d amr bash -lc \
    "source /opt/ros/jazzy/setup.bash; source /work/install/setup.bash; cd /work; \
     exec python3 scripts/log_run.py /work/log/repeat/$tag.csv $LOG_S \
     > /work/log/logrun_$tag.log 2>&1"
  sleep 6

  run_in "ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
    '{header: {frame_id: map}, pose: {position: {x: $GOAL_X, y: $GOAL_Y}, orientation: {w: 1.0}}}'" \
    >/dev/null 2>&1

  # Wait for the goal, up to 190 s (static ~95 s, dynamic ~140 s).
  for _ in $(seq 1 38); do
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