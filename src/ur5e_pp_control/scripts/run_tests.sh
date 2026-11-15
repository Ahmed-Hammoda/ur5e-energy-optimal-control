#!/usr/bin/env bash
# Run the energy test matrix headless and build the report.
#   nominal: 0.5 kg box, 0.15 m/s, no offset
#   varied : 0.3 kg box, 0.20 m/s, 5 mm start offset
# Usage: ./run_tests.sh [cycles]      (default 30; use 3 for a quick check)
#        MODES="scaled" SCENARIOS="varied" ./run_tests.sh   (run only some)
CYCLES=${1:-30}
MODES=${MODES:-"scaled trajectory"}
SCENARIOS=${SCENARIOS:-"nominal varied"}
WS="$(cd "$(dirname "$0")/../../.." && pwd)"
source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"
RESULTS="$WS/results"
# Private ROS domain + Gazebo partition so a simulation you have open
# (e.g. sim.launch.py with the GUI) can't interfere with the tests.
export ROS_DOMAIN_ID=${TEST_DOMAIN_ID:-77}
export GZ_PARTITION=ur5e_pp_tests
mkdir -p "$RESULTS"

run_scenario() {   # name box_mass speed offset_mm
  local name=$1 box=$2 speed=$3 offset=$4
  echo "=== scenario $name: box ${box} kg, ${speed} m/s, offset ${offset} mm, $CYCLES cycles"
  setsid ros2 launch ur5e_pp_control sim.launch.py gui:=false box_mass:="$box" > "$RESULTS/sim_$name.log" 2>&1 &
  local sim=$!
  # wait until the last controller is loaded (spawned inactive)
  for _ in $(seq 60); do
    timeout 5 ros2 service call /controller_manager/list_controllers \
      controller_manager_msgs/srv/ListControllers 2>/dev/null | grep -q scaled_joint_trajectory_controller && break
    sleep 1
  done
  for mode in $MODES; do
    ros2 launch ur5e_pp_control experiment.launch.py mode:=$mode cycles:="$CYCLES" speed:="$speed" \
      start_offset_mm:="$offset" payload_kg:="$(python3 -c "print(0.7+$box)")" \
      tag:="${name}_${mode}" results_dir:="$RESULTS" 2>&1 | grep -E "cycle|FAULT|Finished|ERROR"
  done
  kill -INT -- -$sim 2>/dev/null; sleep 3; kill -9 -- -$sim 2>/dev/null
  pkill -f "[g]z sim" 2>/dev/null; sleep 2
}

for sc in $SCENARIOS; do
  case $sc in
    nominal) run_scenario nominal 0.5 0.15 0.0 ;;
    varied)  run_scenario varied  0.3 0.20 5.0 ;;
  esac
done
ros2 run ur5e_pp_control validate --dir "$RESULTS"
