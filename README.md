A ROS 2-based 2D kinematic simulator and a complete autonomous navigation stack for a differential/unicycle robot. This project features a custom lightweight simulator, distance map (DMap) generation, global path planning, and real-time laser-based localization.

This repository contains the following main ROS 2 packages:
rp_commons: Core libraries containing data structures for grid maps, distance maps, and 2D laser scans.
rp_simulator: A lightweight 2D kinematic simulator for a unicycle robot equipped with a LiDAR sensor. It visualizes the robot and its sensor data in a provided map.
dmap_navigation: The core navigation stack. It includes a custom Distance Map generator, an A* Global Planner with obstacle inflation, a closed-loop trajectory controller, and a Gauss-Newton based Localizer that estimates the robot's pose matching laser scans with map gradients.
fast-downward: Integrated path planning engine.

Prerequisites:
Make sure you have the following installed on your Ubuntu 22.04 (or WSL2) system:
ROS 2 Humble
colcon build system
OpenCV (for map image loading and canvas drawing)
Eigen3 (for linear algebra and transformations)
yaml-cpp (for parsing configuration files)

You can install the required ROS 2 standard messages and navigation tools with:
sudo apt update
sudo apt install ros-humble-nav-msgs ros-humble-sensor-msgs ros-humble-geometry-msgs ros-humble-visualization-msgs
sudo apt install ros-humble-nav2-map-server ros-humble-nav2-lifecycle-manager ros-humble-tf2-ros


Build Instructions:

Clone the repository into your workspace:
git clone [https://github.com/YOUR_USERNAME/RP_project_2067707.git](https://github.com/YOUR_USERNAME/RP_project_2067707.git)
cd RP_project_2067707

Build the project using colcon (around 4-5 minutes):
source /opt/ros/humble/setup.bash
colcon build

Source the local workspace:
source install/setup.bash


Usage Instructions:
To run the full autonomous navigation stack, you will need multiple terminal windows. Make sure to run source install/setup.bash in every new terminal.

1 Start the Simulator, Map Server, and RViz:
ros2 launch dmap_navigation nav_launch.py

2 Start the Localizer Node in a new terminal:
ros2 run dmap_navigation localizer_node

3 Start the Planner & Controller Node in another new terminal:
ros2 run dmap_navigation planner_node

Navigating in RViz2:
Since the simulator's Ground Truth is intentionally disabled to test the real localization algorithm, you must initialize the robot's pose manually:

Initialize Pose: Use the 2D Pose Estimate tool in RViz. Click and drag on the map at the robot's starting location (pointing the arrow to match its orientation). The Localizer will snap the laser scan to the walls and make the robot appear.

Navigate: Use the 2D Goal Pose tool to click a destination. The Planner will generate the path, and the robot will start moving autonomously.