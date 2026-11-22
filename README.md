# obs_avoid_description  (ROS 2 Jazzy + Gazebo Harmonic)

4-wheel skid-steer obstacle-avoidance robot with a servo-mounted HC-SR04, converted from a GrabCAD STEP model.
Units: metres/kg. Frames: x forward, y left, z up.

## Build
    mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src && unzip ~/Downloads/obs_avoid_description.zip
    sudo apt install ros-jazzy-joint-state-publisher-gui ros-jazzy-rviz2
    cd ~/ros2_ws && colcon build --packages-select obs_avoid_description
    source install/setup.bash

## 1) Look at the model in RViz (no simulator)
    ros2 launch obs_avoid_description display.launch.py
RViz: Fixed Frame = base_link, Add -> RobotModel (Description Topic /robot_description). Use the slider window to spin wheels/servo.

## 2) Simulate in Gazebo
    ros2 launch obs_avoid_description sim.launch.py
Drive:   ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}}"
Turn:    ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist "{angular: {z: 1.0}}"
Servo:   ros2 topic pub --once /servo_cmd std_msgs/msg/Float64 "{data: 0.6}"     (radians, -1.57 .. 1.57)
Sonar:   ros2 topic echo /sonar     (LaserScan, 5 rays over ~15 deg; distance = smallest value in `ranges`)

## Joints
wheel_{front,rear}_{left,right}_joint: continuous, axis y, positive = forward.
servo_joint: revolute +-90 deg about z. 0 = sonar straight ahead (the CAD snapshot had +42.8 deg = 0.747 rad).
sonar_link: x axis = beam direction.

## Estimates to replace with real values
Masses (base 0.359 kg, wheels 0.030 kg each, bracket 0.006, sonar 0.009). Inertias are computed for those masses.
wheel_separation (0.1374) is the geometric track; skid-steer turning usually needs a larger value - tune it.
Status: geometry checked offline; the Gazebo plugin setup has not been run on a live simulator.
