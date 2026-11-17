#!/usr/bin/env bash
# run_trial.sh <algorithm> <seed>
# Runs ONE trial from a clean start and saves one row in runs.csv.
# Example:  bash run_trial.sh random_turn 3
# Another room:  SCENARIO=living_room bash run_trial.sh fuzzy 1
#   (without SCENARIO the old obstacle course is used, exactly as before)

ALGO="$1"
SEED="$2"
if [ -z "$ALGO" ] || [ -z "$SEED" ]; then
  echo "usage: run_trial.sh <algorithm> <seed>    e.g.  run_trial.sh random_turn 3"
  echo "       SCENARIO=living_room bash run_trial.sh <algorithm> <seed>"
  exit 1
fi

SCENARIO="${SCENARIO:-obstacle_course}"
SCEN_DIR="$HOME/ros2_ws/project/scenarios"
if [ "$SCENARIO" = "obstacle_course" ]; then
  GOAL_X=3.0
  GOAL_Y=0.0
  TIMEOUT=120.0
  SCEN_FILE=""
  export OBS_AVOID_WORLD=""
  LABEL="${ALGO}_s${SEED}"
else
  SCEN_FILE="$SCEN_DIR/$SCENARIO.json"
  if [ ! -f "$SCEN_FILE" ] || [ ! -f "$SCEN_DIR/$SCENARIO.sdf" ]; then
    echo "There is no scenario '$SCENARIO' (looked for $SCEN_FILE and $SCEN_DIR/$SCENARIO.sdf)."
    exit 1
  fi
  read -r GOAL_X GOAL_Y TIMEOUT < <(python3 -c "import json,sys; d=json.load(open(sys.argv[1])); print(float(d['goal'][0]), float(d['goal'][1]), float(d['timeout']))" "$SCEN_FILE")
  export OBS_AVOID_WORLD="$SCEN_DIR/$SCENARIO.sdf"
  LABEL="${SCENARIO}__${ALGO}_s${SEED}"
fi

MAX_WAIT=$(( ${TIMEOUT%.*} + 60 ))    # give up after this many real seconds
RAW="$HOME/ros2_ws/project/results/raw"
CSV="$RAW/runs.csv"
LOGDIR="$RAW/logs/${LABEL}_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$LOGDIR"

[ -f /opt/ros/jazzy/setup.bash ] && source /opt/ros/jazzy/setup.bash
[ -f "$HOME/ros2_ws/install/setup.bash" ] && source "$HOME/ros2_ws/install/setup.bash"

TAIL_PID=""

stop_all() {
  [ -n "$TAIL_PID" ] && kill "$TAIL_PID" 2>/dev/null
  pkill -f sweep_scanner
  pkill -f avoidance_node
  pkill -f "obs_avoid_control/judge"
  pkill -f "judge --ros-args"
  pkill -f parameter_bridge
  pkill -f robot_state_publisher
  pkill -f "ros2 launch obs_avoid_description"
  pkill -f "gz sim"
  sleep 3
}
trap stop_all EXIT

count_rows() {
  if [ -f "$CSV" ]; then wc -l < "$CSV"; else echo 0; fi
}

wait_for_topic() {   # wait_for_topic <topic> <seconds>
  for _ in $(seq 1 "$2"); do
    ros2 topic list 2>/dev/null | grep -qx "$1" && return 0
    sleep 1
  done
  return 1
}

echo "=== trial: $LABEL   scenario: $SCENARIO   goal ($GOAL_X, $GOAL_Y)   timeout $TIMEOUT s   (logs in $LOGDIR) ==="

echo "[1/6] clearing leftovers from earlier runs"
stop_all
if pgrep -f "gz sim" > /dev/null; then
  echo "Gazebo is still running. Close it by hand, then try again."
  exit 1
fi

echo "[2/6] starting the world"
ros2 launch obs_avoid_description sim.launch.py > "$LOGDIR/world.log" 2>&1 &
if ! wait_for_topic /odom 90; then
  echo "The world did not start (no /odom after 90 s). See $LOGDIR/world.log"
  exit 1
fi
sleep 5

echo "[3/6] starting the translator and the neck scanner"
ros2 run ros_gz_bridge parameter_bridge \
  "/world/obstacle_course/dynamic_pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V" \
  > "$LOGDIR/bridge.log" 2>&1 &
ros2 run obs_avoid_control sweep_scanner --ros-args -p use_sim_time:=true \
  > "$LOGDIR/neck.log" 2>&1 &
if ! wait_for_topic /scan_wide 30; then
  echo "The neck scanner did not start (no /scan_wide). See $LOGDIR/neck.log"
  exit 1
fi
sleep 3

echo "[4/6] starting the referee with label $LABEL"
ros2 run obs_avoid_control judge --ros-args -p use_sim_time:=true -p label:="$LABEL" \
  -p goal_x:="$GOAL_X" -p goal_y:="$GOAL_Y" -p timeout:="$TIMEOUT" -p scenario_file:="$SCEN_FILE" \
  > "$LOGDIR/referee.log" 2>&1 &
sleep 2

ROWS_BEFORE=$(count_rows)

echo "[5/6] starting the brain: $ALGO, seed $SEED"
ALGO_SEED="$SEED" ros2 run obs_avoid_control avoidance_node --ros-args \
  -p use_sim_time:=true -p algorithm:="$ALGO" -p goal_x:="$GOAL_X" -p goal_y:="$GOAL_Y" -p timeout:="$TIMEOUT" \
  > "$LOGDIR/brain.log" 2>&1 &
sleep 1
tail -n +1 -f "$LOGDIR/brain.log" &
TAIL_PID=$!

echo "[6/6] waiting for the referee to write the result (up to $MAX_WAIT s)"
START=$SECONDS
FINISHED=0
while [ $((SECONDS - START)) -lt "$MAX_WAIT" ]; do
  NOW=$(count_rows)
  if [ "$NOW" -gt "$ROWS_BEFORE" ]; then FINISHED=1; break; fi
  sleep 1
done
sleep 2

if [ "$FINISHED" -eq 1 ]; then
  echo
  echo "RESULT:"
  tail -n 1 "$CSV"
  exit 0
else
  echo
  echo "No result after $MAX_WAIT s. Look at the logs in $LOGDIR"
  exit 2
fi
