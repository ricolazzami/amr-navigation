#!/usr/bin/env bash
# Stitch the recorded segments into docs/video/demo.mp4.
#
#   scripts/compose_video.sh clips/static.mkv clips/dynamic.mkv out.mp4
#
# Title card + two labelled mission segments + results card, all 1280x720
# h264 yuv420p 25 fps, so the concat demuxer can join them without a re-mux
# surprise. The old demo.mp4 is left in place as demo.old.mp4 until the new one
# has been probed.
set -eu

STATIC=${1:-/tmp/opencode/clips/static.mkv}
DYNAMIC=${2:-/tmp/opencode/clips/dynamic.mkv}
OUT=${3:-/home/ricou/Projects/Project-3/docs/video/demo.mp4}
ROOT=$(cd "$(dirname "$0")/.." && pwd)
WORK=$(mktemp -d /tmp/opencode/compose.XXXXXX)
FONT_R=/usr/share/fonts/liberation/LiberationSans-Regular.ttf
FONT_B=/usr/share/fonts/liberation/LiberationSans-Bold.ttf
W=1280; H=720; FPS=25
# Keep the whole film inside the 3-5 min brief even if a segment ran long.
MAX_TOTAL=295

common=(-c:v libx264 -preset medium -crf 20 -pix_fmt yuv420p -r $FPS -an)

card() {  # card <out> <seconds> <line1> <line2> <line3>
  local out=$1 secs=$2 l1=$3 l2=$4 l3=$5
  ffmpeg -hide_banner -loglevel error -y -f lavfi \
    -i "color=c=0x101418:s=${W}x${H}:r=${FPS}:d=${secs}" \
    -vf "drawtext=fontfile=${FONT_B}:text='${l1}':fontcolor=white:fontsize=52:x=(w-text_w)/2:y=250,\
drawtext=fontfile=${FONT_R}:text='${l2}':fontcolor=0xbfd4e2:fontsize=30:x=(w-text_w)/2:y=340,\
drawtext=fontfile=${FONT_R}:text='${l3}':fontcolor=0x7d97a8:fontsize=22:x=(w-text_w)/2:y=400" \
    "${common[@]}" -t "$secs" "$out"
}

label() {  # label <in> <out> <text>
  local in=$1 out=$2 text=$3
  ffmpeg -hide_banner -loglevel error -y -i "$in" -vf \
    "drawtext=fontfile=${FONT_B}:text='${text}':fontcolor=white:fontsize=30:x=24:y=20:box=1:boxcolor=0x00000088:boxborderw=10" \
    "${common[@]}" "$out"
}

echo "== cards"
card "$WORK/00-title.mp4" 6 \
  "AMR Navigation — ROS 2 Jazzy" \
  "LiDAR mapping · EKF + AMCL · custom A* (Manhattan) · reactive avoidance" \
  "DecodeLabs Robotics & Automation — Project 3"

echo "== labels"
label "$STATIC"  "$WORK/01-static.mp4"  "Mission 1 — static maze, goal (4, 4)"
label "$DYNAMIC" "$WORK/02-dynamic.mp4" "Mission 2 — moving obstacle, dynamic avoidance + replan"

card "$WORK/03-end.mp4" 8 \
  "Measured results" \
  "static 10/10 · dynamic 10/10 · 0 contact · 0 oscillation per approach" \
  "every dynamic run held >= 1 s → marked the cell → replanned (docs/report.md)"

echo "== fit to ${MAX_TOTAL}s"
d_static=$(ffprobe -v error -show_entries format=duration -of csv=p=0 "$WORK/01-static.mp4")
d_dyn=$(ffprobe -v error -show_entries format=duration -of csv=p=0 "$WORK/02-dynamic.mp4")
total=$(python3 -c "print(6+$d_static+$d_dyn+8)")
echo "   title 6 + static ${d_static} + dynamic ${d_dyn} + end 8 = ${total}s"
if python3 -c "import sys; sys.exit(0 if $total > $MAX_TOTAL else 1)"; then
  # Trim the *head*, not the tail. `-t N` on the output keeps the first N
  # seconds, and each segment's payoff is at the end: the arrival, and on the
  # dynamic leg the hold -> mark -> replan the video exists to show. Dropping
  # the first seconds only costs idle RViz while the robot waits for its goal.
  keep=$(python3 -c "print(round(($MAX_TOTAL-14)*($d_static/ ($d_static+$d_dyn)),1))")
  keep2=$(python3 -c "print(round($MAX_TOTAL-14-$keep,1))")
  skip=$(python3 -c "print(max(0.0, round($d_static-$keep,1)))")
  skip2=$(python3 -c "print(max(0.0, round($d_dyn-$keep2,1)))")
  echo "   over budget -> static skip ${skip}s keep ${keep}s, dynamic skip ${skip2}s keep ${keep2}s"
  ffmpeg -hide_banner -loglevel error -y -ss "$skip" -i "$WORK/01-static.mp4" -t "$keep" "${common[@]}" "$WORK/01-static-t.mp4"
  ffmpeg -hide_banner -loglevel error -y -ss "$skip2" -i "$WORK/02-dynamic.mp4" -t "$keep2" "${common[@]}" "$WORK/02-dynamic-t.mp4"
  mv "$WORK/01-static-t.mp4" "$WORK/01-static.mp4"
  mv "$WORK/02-dynamic-t.mp4" "$WORK/02-dynamic.mp4"
fi

echo "== concat"
printf "file '%s'\n" "$WORK/00-title.mp4" "$WORK/01-static.mp4" "$WORK/02-dynamic.mp4" "$WORK/03-end.mp4" > "$WORK/list.txt"
mkdir -p "$(dirname "$OUT")"
[ -f "$OUT" ] && cp -f "$OUT" "$(dirname "$OUT")/demo.old.mp4"
ffmpeg -hide_banner -loglevel error -y -f concat -safe 0 -i "$WORK/list.txt" -c copy "$OUT"

ffprobe -v error -show_entries format=duration,size -show_entries stream=width,height,r_frame_rate,codec_name -of default=nw=1 "$OUT"
echo "== done: $OUT"
