# parking_assist_ros2

A ROS2 Jazzy based parking reservation, guidance and self driving system. Includes a simulation of a car 
with packages that integrate them with a central parking coordinator system.

## Features:
- A Simulated car (toyota prius) with sensors on a parking garage using Gazebo
- Autonomous driving system
- Detached map publisher for multi vehicle access
- Localization using wheels, IMU, LiDAR and NavSat
- Parking coordinator system nodes (client and server) attached with sqlite3 database