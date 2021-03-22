# TDR-SDC Formula Student Driverless Simulator
![ROS_CI](https://github.com/TDR-SDC/fsd_simulator/workflows/ROS_CI/badge.svg?branch=master)
### Welcome to the official simulation codebase of TDR-SDC for Formula Student Driverless

## Packages
1. `eufs_description`: contains the urdf of the model
2. `eufs_gazebo`: contains all the gazebo worlds and simulation requirements
3. `robot_control`: contains nodes to actuate the model in simulation
4. `gmapping`: openSLAM-gmapping package
5. `pure_pursuit`: Lateral controller

## Installation
### Prerequisites
- Ubuntu 20.04
- Install [ros-noetic](http://wiki.ros.org/noetic/Installation/Ubuntu)
- cmake minimum version 3.19.0
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
  - ros-noetic-navigation
  - ros-noetic-gmapping

Here's a direct terminal command to install all of them at once:
```bash
sudo apt install ros-noetic-ackermann-msgs ros-noetic-twist-mux ros-noetic-joy ros-noetic-controller-manager ros-noetic-velodyne-simulator ros-noetic-effort-controllers ros-noetic-velocity-controllers ros-noetic-joint-state-controller ros-noetic-gazebo-ros-control ros-noetic-navigation ros-noetic-gmapping
```

### How to build
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
```bash
roslaunch eufs_gazebo small_track.launch      # launch eufs simulator
roslaunch gmapping gmapping.launch            # launch gmapping
roslaunch pure_pursuit pure_pursuit.launch    # launch pure_pursuit controller
```
