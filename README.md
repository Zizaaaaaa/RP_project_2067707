A ROS 2-based 2D kinematic simulator and navigation stack for a unicycle robot. This project features a custom lightweight simulator, distance map generation, and path planning capabilities.

This repository contains the following main ROS 2 packages:
rp_commons: Core libraries containing data structures for grid maps, distance maps, and 2D laser scans.
rp_simulator: A lightweight 2D kinematic simulator for a unicycle robot equipped with a LiDAR sensor. It visualizes the robot and its sensor data in a provided map.
dmap_navigation: (WIP) Navigation and distance map utilities.
fast-downward: Integrated path planning engine.

Make sure you have the following installed on your Ubuntu system:
ROS 2 Humble
colcon build system
OpenCV (for map image loading and canvas drawing)
Eigen3 (for linear algebra and transformations)
yaml-cpp (for parsing configuration files)

You can install the required ROS 2 standard messages with:
```bash
sudo apt update
sudo apt install ros-humble-nav-msgs ros-humble-sensor-msgs ros-humble-geometry-msgs


Build instruction:

Clone the repository into your workspace: 
git clone [https://github.com/YOUR_USERNAME/RP_project_2067707.git](https://github.com/YOUR_USERNAME/RP_project_2067707.git)
cd RP_project_2067707

Build the project using colcon (around 4-5 minutes): 
source /opt/ros/humble/setup.bash
colcon build

Source the local workspace:
source install/setup.bash


Usage instruction:

To start the simulator with the default DIAG department map, run:
ros2 run rp_simulator simulator --ros-args -p config_file:=src/dmap_navigation/maps/diag_single_robot.yaml

Keyboard Control (with I, J, K, L):
source /opt/ros/humble/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/robot_1/cmd_vel

To see the ground truth pose of the robot in real-time:
ros2 topic echo /robot_1/groundtruth
