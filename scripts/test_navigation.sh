#!/bin/sh
# launch the turtlebot in the Q802 world.  NOTE: it requires TURTLEBOT_GAZEBO_WORLD_FILE
# in your env, set to your world file
# export TURTLEBOT_GAZEBO_WORLD_FILE="/home/jfinken/projects/udacity/robotics-software-engineer/HomeServiceRobot/worlds/Q802.world"
# xterm  -e  " gazebo " &
# sleep 5

xterm  -e  " roscore" & 
sleep 5

# launches gazebo and rviz
xterm -e " roslaunch home_service_robot world.launch" &
sleep 5

# AMCL, tuned for the skid-steer robot
xterm -e " roslaunch home_service_robot amcl.launch" &
sleep 5

# teleop
xterm -e " rosrun teleop_twist_keyboard teleop_twist_keyboard.py"
