#!/usr/bin/env bash
# Screen-record the can-packing demo for a short video: Gazebo on the left, the energy
# monitor (video view) on the right, one carton of 12 cans, sped up to ~75 s, 1080p mp4.
#
#   ~/ur5e_pp_ws/src/ur5e_pp_control/scripts/record_video.sh                  # -> ~/ur5e_energy_demo.mp4
#   ~/ur5e_pp_ws/src/ur5e_pp_control/scripts/record_video.sh my_video.mp4
#   CANS=4 LENGTH=40 ~/ur5e_pp_ws/src/ur5e_pp_control/scripts/record_video.sh   # shorter test
#
# Needs an X11 desktop (not Wayland), ffmpeg and wmctrl:  sudo apt install ffmpeg wmctrl
# Don't touch the windows while it records; it stops by itself when the carton is full.
set -euo pipefail

WS="$HOME/ur5e_pp_ws"
OUT="$(realpath -m "${1:-$HOME/ur5e_energy_demo.mp4}")"
CANS="${CANS:-12}"          # cans to pack (12 = one full carton)
LENGTH="${LENGTH:-75}"      # length of the final video in seconds (the recording is sped up to fit)
GZ_SHARE="${GZ_SHARE:-0.56}" # part of the screen width given to Gazebo
TMP="$(mktemp -d)"
RAW="$TMP/raw.mkv"
DONE_FILE="$WS/results/demo/demo_after.json"

[ "${XDG_SESSION_TYPE:-x11}" = "wayland" ] && { echo "This needs an X11 session (log in with 'Ubuntu on Xorg')."; exit 1; }
for tool in ffmpeg ffprobe wmctrl; do
  command -v "$tool" >/dev/null || { echo "Missing $tool:  sudo apt install ffmpeg wmctrl"; exit 1; }
done

set +u
source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"
set -u

# Background jobs of a script ignore SIGINT, so everything is stopped with SIGTERM
# (ffmpeg then closes the file properly; ros2 launch shuts its nodes down).
cleanup() {   # stop the recording and the whole demo (Gazebo, monitor, controllers)
  [ -n "${FF:-}" ] && kill -TERM "$FF" 2>/dev/null && wait "$FF" 2>/dev/null || true
  if [ -n "${LAUNCH:-}" ]; then
    # everything the demo started shares its session (Gazebo's server runs in its own group)
    pkill -TERM -s "$LAUNCH" 2>/dev/null || true
    for _ in $(seq 15); do pgrep -s "$LAUNCH" >/dev/null || break; sleep 1; done
    pkill -KILL -s "$LAUNCH" 2>/dev/null || true
  fi
}
trap 'exit 130' INT TERM
trap cleanup EXIT

titles() { wmctrl -l | awk '{t=$4; for (i=5;i<=NF;i++) t=t" "$i; print $1"\t"t}'; }
if titles | cut -f2 | grep -qE "^(Gazebo Sim|UR5e energy monitor)"; then
  echo "A Gazebo or energy monitor window is already open. Close it (Ctrl+C in its terminal) and run this again."
  exit 1
fi
if pgrep -f '(^|/)gz sim' >/dev/null; then
  echo "An old Gazebo process is still running. Stop it with:  pkill -f 'gz sim'   and run this again."
  exit 1
fi
rm -f "$DONE_FILE"
echo "Starting the demo (Gazebo + energy monitor, $CANS cans)..."
setsid ros2 launch ur5e_pp_control demo.launch.py version:=after style:=video cans:="$CANS" >"$TMP/launch.log" 2>&1 &
LAUNCH=$!

wait_window() {   # id of the window whose title starts with $1
  for _ in $(seq 180); do
    id=$(titles | awk -F'\t' -v p="$1" 'index($2, p) == 1 {print $1; exit}')
    [ -n "$id" ] && { echo "$id"; return 0; }
    sleep 1
  done
  echo "Window '$1' did not open; see $TMP/launch.log" >&2
  return 1
}
GZ=$(wait_window "Gazebo Sim")
MON=$(wait_window "UR5e energy monitor")
sleep 3

# usable screen area (without the top bar and dock): "WA: x,y wxh"
read -r WX WY WW WH < <(wmctrl -d | awk '$2=="*" {for (i=1;i<=NF;i++) if ($i=="WA:") {split($(i+1),p,","); split($(i+2),s,"x"); print p[1], p[2], s[1], s[2]}}')
GW=$(python3 -c "print(int($WW * $GZ_SHARE) // 2 * 2)")
for w in "$GZ" "$MON"; do wmctrl -i -r "$w" -b remove,maximized_vert,maximized_horz,fullscreen; done
wmctrl -i -r "$GZ"  -e "0,$WX,$WY,$GW,$WH"
wmctrl -i -r "$MON" -e "0,$((WX + GW)),$WY,$((WW - GW)),$WH"
wmctrl -i -a "$GZ"; wmctrl -i -a "$MON"
sleep 2

W=$((WW / 2 * 2)); H=$((WH / 2 * 2))
echo "Recording ${W}x${H} at ($WX,$WY). It stops by itself after can $CANS."
ffmpeg -loglevel error -y -f x11grab -draw_mouse 0 -framerate 30 -video_size "${W}x${H}" -i "${DISPLAY:-:0}+${WX},${WY}" \
  -c:v libx264 -preset ultrafast -crf 16 "$RAW" &
FF=$!

while [ ! -f "$DONE_FILE" ]; do
  kill -0 "$LAUNCH" 2>/dev/null || { echo "The demo stopped early; see $TMP/launch.log"; exit 1; }
  titles | cut -f1 | grep -q "^$GZ$" && titles | cut -f1 | grep -q "^$MON$" ||
    { echo "The Gazebo or monitor window was closed, so the recording stopped. Run the script again."; exit 1; }
  sleep 2
done
sleep 6                                  # show the final numbers for a moment
kill -TERM "$FF"; wait "$FF" 2>/dev/null || true
FF=""

DUR=$(ffprobe -v error -show_entries format=duration -of csv=p=0 "$RAW")
SPEED=$(python3 -c "print(max(1.0, round($DUR / $LENGTH, 2)))")
echo "Recorded ${DUR%.*} s; speeding up ${SPEED}x and encoding..."
ffmpeg -loglevel error -y -i "$RAW" -vf "setpts=PTS/${SPEED},fps=30,\
scale=1920:1080:force_original_aspect_ratio=decrease,pad=1920:1080:(ow-iw)/2:(oh-ih)/2:color=0xfcfcfb,\
tpad=stop_mode=clone:stop_duration=4,format=yuv420p" \
  -c:v libx264 -preset slow -crf 20 -movflags +faststart -an "$OUT"
echo "Video: $OUT"
echo "(raw recording kept at $RAW)"
