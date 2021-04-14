#!/bin/sh

xterm  -e  " source /opt/ros/noetic/setup.bash; roscore" & 
sleep 5

# launches gazebo and rviz
xterm -e " roslaunch home_service_robot world.launch" &
sleep 5

# AMCL, tuned for the skid-steer robot
xterm -e " roslaunch home_service_robot amcl.launch" &
sleep 5

# teleop
# xterm -e " rosrun teleop_twist_keyboard teleop_twist_keyboard.py"

# add_markers node
xterm -e " roslaunch add_markers add_markers.launch"
