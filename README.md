# TDR-SDC Formula Student Driverless Simulator
![ROS_CI](https://github.com/TDR-SDC/fsd_simulator/workflows/ROS_CI/badge.svg?branch=master)
### Welcome to the SLAM Simulation repo of TDR-SDC

This simulator uses gmapping for implementing SLAM on [eufs simulator](https://github.com/eufsa/eufs_sim)

## Prerequisites
- Ubuntu 20.04
- Install [ros-noetic](http://wiki.ros.org/noetic/Installation/Ubuntu)
- Packages:
  - ros-noetic-ackermann-msgs
  - ros-noetic-twist-mux
  - ros-noetic-joy
  - ros-noetic-controller-manager
  - ros-noetic-robotnik-msgs
  - ros-noetic-velodyne-simulator
  - ros-noetic-effort-controllers
  - ros-noetic-velocity-controllers
  - ros-noetic-joint-state-controller
  - ros-noetic-gazebo-ros-control

Here's a direct terminal command:
```bash
sudo apt install ros-noetic-ackermann-msgs ros-noetic-twist-mux ros-noetic-joy ros-noetic-controller-manager ros-noetic-velodyne-simulator ros-noetic-effort-controllers ros-noetic-velocity-controllers ros-noetic-joint-state-controller ros-noetic-gazebo-ros-control 
```

## How to build
```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/TDR-SDC/fsd_simulator
cd ..
catkin_make
```

## Running the simulation
1. Source the workspace:

- ```source ~/catkin_ws/devel/setup.bash```
2. Enter the three given commands in different terminals after using the sourcing the workspace
- ``` roslaunch eufs_gazebo small_track.launch```
- ``` roslaunch pointcloud_to_laserscan sample_node.launch```
- ``` roslaunch gmapping gmapping.launch```
