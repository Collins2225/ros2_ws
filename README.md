# Comparative Analysis of Obstacle Avoidance Algorithms

Ten reactive obstacle-avoidance algorithms for a low-cost four-wheel ground robot, implemented behind one common
interface and run in the **Gazebo Harmonic** simulator with **ROS 2 Jazzy**. The same robot, the same sonar model and the
same test rooms are used for every algorithm, so the results can be compared fairly.

This repository is the practical part of the final-year project *"Conducting a comparative analysis of obstacle avoidance
algorithms"*. The simplest algorithm, Stop-and-turn, is also ported to the real Arduino robot (see [Hardware](#hardware)).

## Contents

- [The ten algorithms](#the-ten-algorithms)
- [How the simulated robot sees the world](#how-the-simulated-robot-sees-the-world)
- [Test rooms](#test-rooms)
- [Requirements](#requirements)
- [Build](#build)
- [Run an algorithm in Gazebo](#run-an-algorithm-in-gazebo)
- [Draw the robot's path](#draw-the-robots-path)
- [Results](#results)
- [Adding or changing an algorithm](#adding-or-changing-an-algorithm)
- [Hardware](#hardware)
- [Repository layout](#repository-layout)
- [References](#references)

## The ten algorithms

All of them are *reactive*: they decide from what the sonar sees now (and, for some, a short memory of it), with no global
map and no path planning.

| # | Name (`algorithm:=`) | Family | One-sentence idea |
|---|---|---|---|
| 1 | `stop_and_turn` | Threshold | Drive straight; if something is closer than 40 cm, stop, back up, look left and right, turn towards the side with more room. |
| 2 | `braitenberg` | Sensor-to-motor coupling | Every obstacle pushes the robot away, harder the closer it is; no stopping and no decisions. |
| 3 | `fuzzy` | Fuzzy logic | Takagi-Sugeno rules over soft words (NEAR, MEDIUM, FAR) give a blended speed and turn rate. |
| 4 | `potential_field` | Potential field | The goal pulls, obstacles push; the sum of the forces is the driving direction. Includes a way out of local minima (see below). |
| 5 | `follow_the_gap` | Gap based | Find runs of free directions, pick the gap that points closest to the goal, drive through it. |
| 6 | `vfh` | Histogram | Vector Field Histogram: a certainty grid is turned into a polar histogram; steer into the calm valley nearest the goal. |
| 7 | `vfh_plus` | Histogram | VFH+: robot-size enlargement, two thresholds (hysteresis), turning-circle masking and a cost function for choosing the direction. |
| 8 | `bug2` | Geometric | Drive on the start-to-goal line; follow the wall around an obstacle; leave it when back on the line and closer to the goal. |
| 9 | `nearness_diagram` | Gap based / situation based | Find gaps from jumps in the readings, choose the gap nearest the goal, then drive by one of five situations (HSGR, HSWR, HSNR, LS1, LS2). |
| 10 | `dwa` | Motion sampling | Dynamic Window Approach: try many (speed, turn rate) pairs, drop unsafe ones, score the rest by heading, clearance and speed. |

> An earlier *Random turn* algorithm was removed from the study and replaced by Nearness Diagram.

### Notes on individual algorithms

- **Stop-and-turn** is the baseline. It has no memory: the stop distance is 0.40 m, the same value used on the real robot.
- **Potential field** has the classic *local minimum* problem: when an obstacle sits between the robot and the goal, the push
  and the pull cancel and the robot stands still. This implementation breaks the tie in three ways: (1) the obstacle's push is
  turned sideways so it slides the robot around the obstacle, (2) if the goal has not become at least 0.15 m closer for 5 s
  (or the robot crawls in front of something for 2 s) it ignores the goal and walks out sideways for 4 s, (3) every stall
  switches to the other side. Set `ALGO_DEBUG=1` to see `STUCK` messages when it frees itself.
- **VFH / VFH+** keep a 5 cm certainty grid in room coordinates (the robot remembers what it saw a moment ago) and use 72
  sectors of 5 degrees. Both enlarge obstacles by the robot radius (0.14 m); VFH+ adds hysteresis (blocked above 0.90, free
  below 0.45) and the cost `5*goal + 2*heading + 2*previous`.
- **Nearness Diagram** is our version for a 150-degree sonar: readings are capped at 1.5 m, a "pretend wall" is placed beyond
  the edges of the fan so the robot never aims at what it cannot see, a jump larger than 0.35 m starts or ends a gap, gaps
  narrower than 0.40 m are ignored, and the safety zone is 0.35 m.
- **Bug2** is the only algorithm that also needs the robot's own position (to know where the start-to-goal line is).
- **DWA** keeps a small local memory map because the sonar sees only 150 degrees; speed limits are 0.20 m/s and 1.0 rad/s.

## How the simulated robot sees the world

Every algorithm implements the same two-method interface (`src/obs_avoid_control/obs_avoid_control/core/interface.py`):

```python
class Algorithm:
    def reset(self): ...
    def step(self, p: Perception) -> Command: ...   # called every 0.05 s
```

- `Perception` holds the sonar ranges, the robot pose (`x`, `y`, `yaw`), the goal, the time, and helpers such as
  `goal_bearing()`, `goal_distance()` and `sector_min(lo, hi)`.
- `Command` is a forward speed (`linear`, m/s) and a turn rate (`angular`, rad/s).

The simulated sonar imitates the real sensor (an HC-SR04 on a servo that sweeps left and right):

| Property | Value |
|---|---|
| Field of view | 150 degrees (from -75 to +75 degrees) |
| Directions | 31, one every 5 degrees |
| Beam width | about 15 degrees per direction |
| Maximum range | 2.0 m |
| Unknown / not measured | `nan`; the robot never drives into the unknown |
| Echo distance | speed of sound x time / 2 |
| Speed limit | 0.20 m/s linear, 1.0 rad/s angular |

## Test rooms

Rooms are described by a JSON file (goal, time limit, obstacles) and a Gazebo world (`.sdf`) in `project/scenarios/`.

| Scenario (`SCENARIO=`) | What it is | Goal | Time limit |
|---|---|---|---|
| `obstacle_course` | Default arena with four obstacles | (3.0, 0.0) | 120 s |
| `living_room` | A furnished house with many pieces of furniture and narrow passages | (4.7, 0.0) | 150 s |

The robot starts at the origin facing along +x. A run **ends** when the robot reaches the goal (`GOAL REACHED`) or when the
time limit passes (`TIMEOUT after ...`); the referee node prints one of these lines in the run log.

## Requirements

- Ubuntu 24.04
- ROS 2 Jazzy
- Gazebo Harmonic (with `ros_gz_bridge`)
- RViz 2
- Python 3.12 with `numpy` and `matplotlib` (for the diagrams)

## Build

This repository is a complete ROS 2 workspace, so clone it as `~/ros2_ws`:

```bash
git clone https://github.com/Collins2225/ros2_ws.git ~/ros2_ws
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Run an algorithm in Gazebo

Run **one algorithm at a time**. The demo script starts Gazebo, RViz (what the robot sees and its trail) and the chosen
algorithm, and stops everything when the run is over.

```bash
# default arena
bash ~/ros2_ws/project/scripts/demo.sh stop_and_turn

# the furnished living room
SCENARIO=living_room bash ~/ros2_ws/project/scripts/demo.sh vfh

# every algorithm, one after another (press Enter between them)
bash ~/ros2_ws/project/scripts/demo.sh all
```

Handy settings:

| Variable | Meaning |
|---|---|
| `SCENARIO` | Room to use (`obstacle_course`, `living_room`) |
| second argument | Random seed, for example `demo.sh fuzzy 2` |
| `ALGO_DEBUG=1` | Print what the algorithm decides about once a second |

To run trials of one algorithm without the demo windows (the arguments are the algorithm and the seed range; see the
header of the script for the exact options):

```bash
SCENARIO=living_room bash ~/ros2_ws/project/scripts/run_many.sh dwa 1 5
```

The nodes can also be started by hand:

```bash
ros2 launch obs_avoid_description sim.launch.py                        # Gazebo, robot, bridges
ros2 run obs_avoid_control sweep_scanner --ros-args -p use_sim_time:=true   # the sonar "neck"
ros2 run obs_avoid_control avoidance_node --ros-args \
  -p use_sim_time:=true -p algorithm:=nearness_diagram \
  -p goal_x:=4.7 -p goal_y:=0.0 -p timeout:=150.0
```

## Draw the robot's path

`house_diagrams.sh` runs algorithms once each in the living room, records the robot's position (`/odom`) and draws the
path on the room plan.

```bash
bash ~/ros2_ws/project/scripts/house_diagrams.sh                      # vfh, vfh_plus, nearness_diagram
bash ~/ros2_ws/project/scripts/house_diagrams.sh vfh                  # only one
bash ~/ros2_ws/project/scripts/house_diagrams.sh plot                 # redraw from saved runs
DEST=~/my_pictures bash ~/ros2_ws/project/scripts/house_diagrams.sh   # choose the output folder
```

Output (default `~/Documents/obstacle_avoidance_diagrams`): `house_<algorithm>.png`, `house_all.png`,
`house_overlay.png`, `house_diagrams.pdf`, plus `raw/*.csv` (the recorded positions) and `logs/`.

## Results

> **To be filled in from the runs in `results/`.** The numbers below are intentionally empty; do not publish a number that
> was not measured.

Scenario: `living_room`, seeds 1-5. Fill one row per algorithm.

| Algorithm | Reached goal (of 5) | Mean time to goal (s) | Mean path length (m) | Collisions | Timeouts / stuck |
|---|---|---|---|---|---|
| stop_and_turn |  |  |  |  |  |
| braitenberg |  |  |  |  |  |
| fuzzy |  |  |  |  |  |
| potential_field |  |  |  |  |  |
| follow_the_gap |  |  |  |  |  |
| vfh |  |  |  |  |  |
| vfh_plus |  |  |  |  |  |
| bug2 |  |  |  |  |  |
| nearness_diagram |  |  |  |  |  |
| dwa |  |  |  |  |  |

Path pictures: `docs/images/house_all.png` and `docs/images/house_overlay.png`.

## Adding or changing an algorithm

1. Create `src/obs_avoid_control/obs_avoid_control/algorithms/<name>.py` with a class `Controller(Algorithm)` that sets `name` and implements
   `step(self, p)` returning a `Command`.
2. Rebuild: `cd ~/ros2_ws && colcon build --packages-select obs_avoid_control --symlink-install`.
3. Run it: `bash project/scripts/demo.sh <name>`.

A controller must stay safe by itself: when the reading straight ahead is `nan` (unknown), return `Command(0, 0)`.

## Hardware

The same Stop-and-turn logic runs on the real robot.

| Part | Role |
|---|---|
| Arduino Uno + Adafruit-style L293D motor shield | Brain and motor driver |
| 4 yellow DC gear motors | Drive (M1 front right, M2 front left, M3 back left, M4 back right) |
| HC-SR04 ultrasonic sensor | Distance (Trig A0, Echo A1) |
| SG90 servo on header SER1 (pin 10) | Turns the sonar left and right |
| 2 x 18650 Li-ion cells + switch | Power (about 7.4 V) to the EXT_PWR terminal |

The Arduino sketch and its own README are in [`stop_and_turn/`](stop_and_turn/).

## Repository layout

The build output (`build/`, `install/`, `log/`) is generated by `colcon build` and is not stored in the repository.

```
src/
  obs_avoid_control/obs_avoid_control/   ROS 2 Python package
    core/interface.py                    Perception, Command, Algorithm
    algorithms/                          one file per algorithm
    experiments/, metrics/               trial runner and measurements
  obs_avoid_description/                 Gazebo worlds, URDF and meshes, launch files
project/
  scenarios/                             room descriptions (.json) and worlds (.sdf)
  scripts/                               demo.sh, run_many.sh, house_diagrams.sh, plotting and recording scripts
  results/                               plots/ and raw/ (logs and recordings of every run)
stop_and_turn/                           Arduino version of Algorithm 1 (sketch and README)
docs/images/                             path diagrams used in this README
```

## References

- V. Braitenberg, *Vehicles: Experiments in Synthetic Psychology*, MIT Press, 1984.
- O. Khatib, "Real-time obstacle avoidance for manipulators and mobile robots", *Int. J. Robotics Research*, 1986.
- T. Takagi, M. Sugeno, "Fuzzy identification of systems and its applications to modeling and control", *IEEE Trans. SMC*, 1985.
- V. Lumelsky, A. Stepanov, "Path-planning strategies for a point mobile automaton moving amidst unknown obstacles of
  arbitrary shape", *Algorithmica*, 1987.
- J. Borenstein, Y. Koren, "The Vector Field Histogram: fast obstacle avoidance for mobile robots", *IEEE Trans. Robotics and
  Automation*, 1991.
- I. Ulrich, J. Borenstein, "VFH+: reliable obstacle avoidance for fast mobile robots", *ICRA*, 1998.
- D. Fox, W. Burgard, S. Thrun, "The Dynamic Window Approach to collision avoidance", *IEEE Robotics & Automation Magazine*, 1997.
- J. Minguez, L. Montano, "Nearness Diagram (ND) navigation: collision avoidance in troublesome scenarios", *IEEE Trans.
  Robotics and Automation*, 2004.
- V. Sezer, M. Gokasan, "A novel obstacle avoidance algorithm: Follow the Gap Method", *Robotics and Autonomous Systems*, 2012.