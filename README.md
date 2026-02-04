# CoBRA-Challenge
ROS2 Node from PIDTeam for CoBRA-Challenge

Challenge with small autonomous airships for payload grasping and delivery.

This approach uses 3D localization and navigation with AprilTag-based vision and closed-loop control for guidance
and positioning.

## Prerequisites
Must be installed both on PC and remote Raspberry Pi (tested on Pi5) 
- Ubuntu 24.04 
- ROS2 Jazzy

## Additional ROS packages needed
```
sudo apt install ros-$ROS_DISTRO-image-transport-plugins
sudo apt install ros-$ROS_DISTRO-apriltag-ros
sudo apt install ros-$ROS_DISTRO-camera-ros
sudo apt install ros-$ROS_DISTRO-image-pipeline
sudo apt install ros-$ROS_DISTRO-tf-transformations
```

## Useful commands

- **Launch rviz:** 
```
ros2 run rviz2 rviz2
```

- **Launch rqtgraph:** 
```
rqt_graph
```

- **Launch rqt_plot:** 
```
ros2 run rqt_plot rqt_plot
```

- **Compile et launch nodes on Raspberry Pi:** 
```
colcon build && ros2 launch cobrapid_pi cobrapid_pi.launch.py
```

- **Display all avaliable nodes:**
```
ros2 node list
```

- **Display all avaliable topics:**
```
ros2 topic list
```

- **Display topic data type:**
```
ros2 topic info /name_of_topic
```

- **Display topic publication frequency:**
```
ros2 topic hz /name_of_topic
```

- **Display topic data in real time:**
```
ros2 topic echo /name_of_topic
```
