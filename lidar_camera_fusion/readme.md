# Lidar-Camera Fusion

In this package, we have fused output from YOLOv4 on camera with LIDAR Pointcloud to get colour information of cones.

- Install Darknet YOLOv4.
- Run EUFS simulator.
- Change the camera_info, raw image topics in pixel_cloud_fusion launch file.
- Run Laser2PC package.
- Run yolo eufs package.
- Run pixel_cloud_fusion package.
- Visualize the output in rviz.
