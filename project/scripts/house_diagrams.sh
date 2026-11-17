#!/usr/bin/env bash
# house_diagrams [algorithm ...]      default: vfh vfh_plus nearness_diagram
# house_diagrams plot                 only redraw the pictures from runs already saved
# Runs each algorithm ONCE in the house, records where the robot went, and draws the diagrams.
# Pictures go to:  ~/Documents/obstacle_avoidance_diagrams   (change it with  DEST=/some/folder house_diagrams)

PROJ="$HOME/ros2_ws/project"
PKG="$HOME/ros2_ws/src/obs_avoid_control/obs_avoid_control/algorithms"
SCENARIO="${SCENARIO:-living_room}"
SCEN_DIR="$PROJ/scenarios"
SCEN_FILE="$SCEN_DIR/$SCENARIO.json"
DEST="${DEST:-$HOME/Documents/obstacle_avoidance_diagrams}"
SEED="${SEED:-1}"
mkdir -p "$DEST/raw" "$DEST/logs"

if [ "$1" = "plot" ]; then
  shift
  ALGOS="${*:-vfh vfh_plus nearness_diagram}"
  python3 "$PROJ/scripts/plot_house.py" "$SCEN_FILE" "$DEST" $ALGOS
  exit $?
fi

ALGOS="${*:-vfh vfh_plus nearness_diagram}"
if [ ! -f "$SCEN_FILE" ] || [ ! -f "$SCEN_DIR/$SCENARIO.sdf" ]; then
  echo "There is no room '$SCENARIO' (looked for $SCEN_FILE and $SCEN_DIR/$SCENARIO.sdf)."; exit 1
fi
read -r GOAL_X GOAL_Y TIMEOUT < <(python3 -c "import json,sys; d=json.load(open(sys.argv[1])); print(float(d['goal'][0]), float(d['goal'][1]), float(d['timeout']))" "$SCEN_FILE")
export OBS_AVOID_WORLD="$SCEN_DIR/$SCENARIO.sdf"
MAX_WAIT=$(( ${TIMEOUT%.*} + 30 ))

[ -f /opt/ros/jazzy/setup.bash ] && source /opt/ros/jazzy/setup.bash
[ -f "$HOME/ros2_ws/install/setup.bash" ] && source "$HOME/ros2_ws/install/setup.bash"

REC_PID=""
stop_all() {
  [ -n "$REC_PID" ] && kill "$REC_PID" 2>/dev/null
  REC_PID=""
  pkill -f demo_markers; pkill -f rviz2; pkill -f sweep_scanner; pkill -f avoidance_node
  pkill -f "obs_avoid_control/judge"; pkill -f "judge --ros-args"; pkill -f parameter_bridge
  pkill -f robot_state_publisher; pkill -f "ros2 launch obs_avoid_description"; pkill -f "gz sim"
  sleep 3
}
trap stop_all EXIT
wait_for_topic() { for _ in $(seq 1 "$2"); do ros2 topic list 2>/dev/null | grep -qx "$1" && return 0; sleep 1; done; return 1; }

DONE=""
for ALGO in $ALGOS; do
  if [ ! -f "$PKG/$ALGO.py" ]; then echo "There is no algorithm '$ALGO' - skipping."; continue; fi
  echo; echo "################  $ALGO in the house  ################"
  stop_all
  if pgrep -f "gz sim" > /dev/null; then echo "Gazebo is still running. Close its window by hand, then try again."; exit 1; fi
  echo "[1/4] starting Gazebo"
  ros2 launch obs_avoid_description sim.launch.py > "$DEST/logs/${ALGO}_world.log" 2>&1 &
  if ! wait_for_topic /odom 90; then echo "No /odom after 90 s. See $DEST/logs/${ALGO}_world.log"; exit 1; fi
  sleep 5
  echo "[2/4] starting the sonar neck"
  ros2 run obs_avoid_control sweep_scanner --ros-args -p use_sim_time:=true > "$DEST/logs/${ALGO}_neck.log" 2>&1 &
  if ! wait_for_topic /scan_wide 30; then echo "No /scan_wide. See $DEST/logs/${ALGO}_neck.log"; exit 1; fi
  sleep 3
  echo "[3/4] driving: $ALGO   (the robot is recorded, do not touch the windows)"
  rm -f "$DEST/raw/$ALGO.csv"
  /usr/bin/python3 "$PROJ/scripts/record_odom.py" "$DEST/raw/$ALGO.csv" --ros-args -p use_sim_time:=true > "$DEST/logs/${ALGO}_rec.log" 2>&1 &
  REC_PID=$!
  sleep 1
  ALGO_SEED="$SEED" ros2 run obs_avoid_control avoidance_node --ros-args \
    -p use_sim_time:=true -p algorithm:="$ALGO" -p goal_x:="$GOAL_X" -p goal_y:="$GOAL_Y" -p timeout:="$TIMEOUT" \
    > "$DEST/logs/${ALGO}_brain.log" 2>&1 &
  START=$SECONDS
  while [ $((SECONDS - START)) -lt "$MAX_WAIT" ]; do
    grep -qE "GOAL REACHED|TIMEOUT after" "$DEST/logs/${ALGO}_brain.log" && break
    printf "\r      %3d s" $((SECONDS - START)); sleep 1
  done
  echo
  sleep 2
  if grep -q "GOAL REACHED" "$DEST/logs/${ALGO}_brain.log"; then echo ">>> $ALGO: reached the goal."
  elif grep -q "TIMEOUT after" "$DEST/logs/${ALGO}_brain.log"; then echo ">>> $ALGO: time ran out before the goal."
  else echo ">>> $ALGO: no result after $MAX_WAIT s (see $DEST/logs/${ALGO}_brain.log)."; fi
  echo "[4/4] saved $(($(wc -l < "$DEST/raw/$ALGO.csv") - 1)) positions"
  stop_all
  DONE="$DONE $ALGO"
done

echo; echo "Drawing the diagrams..."
python3 "$PROJ/scripts/plot_house.py" "$SCEN_FILE" "$DEST" vfh vfh_plus nearness_diagram
echo
echo "Saved in: $DEST"
ls -1 "$DEST"/*.png "$DEST"/*.pdf 2>/dev/null
