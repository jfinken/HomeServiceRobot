#!/bin/sh
# launch the turtlebot in the Q802 world.  NOTE: it requires TURTLEBOT_GAZEBO_WORLD_FILE
# in your env, set to your world file
# export TURTLEBOT_GAZEBO_WORLD_FILE="/home/jfinken/projects/udacity/robotics-software-engineer/HomeServiceRobot/worlds/Q802.world"
# xterm  -e  " gazebo " &
# sleep 5
xterm  -e  " source /opt/ros/noetic/setup.bash; roscore" & 
sleep 5
# launches gazebo and rviz
xterm -e " roslaunch home_service_robot world.launch" &
sleep 5
# TODO: use launch file?
xterm -e " rosrun gmapping slam_gmapping scan:=scan _base_frame:=chassis"
sleep 5
