# RRT star with multiple remote goals.

This repository contains package with code implementation of RRT star-based path planning algorithm for finding out a suitable path between the cones.

### Lidar based
This is completly based on lidar data (velodyne 16) and therefore there is a need to install pcl in your system for point cloud processong and conversion of point cloud to cones cluster and thereby finding out their coordinates.

### Notes
We just use LIDAR data to mark each cones with  a coordinate and covert each coordinate into list and use it for path planning.

#### Parameters to tune (main)
- `odom_topic` = `/odometry` - The topic to get the odometry information from
- planDistance = 12 m - Maximum length of tree branch
- expandDistance = 1 m - Length of tree node/step
- expandAngle = 20 deg - constraining angle for next tree nodes
- coneObstacleSize = 0.5 m - size of obstacles derived from cone position
- coneTargetsDistRatio = 0.5 - ratio to frontConesDist for deriving remote cone goals

### Commands 
- roslaunch eufs_gazebo small_track.launch 
- roslaunch robot_control robot_control.launch 
- rosrun pointcloud_process final_script.py
- roslaunch path_planning initiation.launch
