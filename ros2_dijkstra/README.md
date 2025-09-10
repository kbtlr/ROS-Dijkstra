###############################################################################
SUMMARY

A Dijkstra algorithm with the intention of finding the best path in an rViz environment. A 7*10 grid is used as a virtual map, with free spaces (0) and obstacles (1). A set of coordinates is published in a separate window, and a navigator moves around obstacles to reach it. For use with Ubuntu 24.04.2 LTS on a system with ROS2 Kilted installed. 
###############################################################################
REQUIREMENTS

A catkin workspace must exist as specified on "https://docs.ros.org/en/kilted/Tutorials/", with the stated dependencies. Follow the tutorials up until and including "Beginner: Client libraries/Creating a workspace" before running the package.
###############################################################################
INSTRUCTIONS

Command lines to run ON NEW TERMINAL (2 tabs):

cd ~/ros2_ws
colcon build
source /opt/ros/kilted/setup.bash
source install/setup.bash

In the first tab:
ros2 launch ros2_dijkstra rviz_launch.py

And the second (Change x and y as you wish):
ros2 topic pub -1 /goal_pose geometry_msgs/PoseStamped '{header: {frame_id: "map"}, pose: {position: {x: 4.0, y: 2.0, z: 0.0}, orientation: {w: 1.0}}}'
###############################################################################
BUGS

- Robot model does not move with navigator frame
- Navigator can move off the grid
