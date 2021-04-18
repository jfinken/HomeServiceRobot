#!/bin/sh

#------------------------------------------------------------------------------
# Core launch script for the Home Service Robot
#
# - Launch the world.launch.  This will launch gazebo, the skid-steer robot,
#   and place it in the Q802 world model.  It also launches rviz auto-loading 
#   the navigation.rviz config.
#
# - Launch the AMCL and Navigation (move_base) stacks.  A map prior has been
#   built via RTAB-map in ROS1-noetic.  That map is loaded by the amcl node.
#
# - Launch the add_markers node.  This node subscribes to Zone messages on
#   the /home_service_robot/zone topic, and publishes visualization markers
#   to the /visualization_marker topic.  These markers are rendered in rviz.
#
# - Launch the pick_objects node.  This node contains hard-coded Poses directing
#   where the robot to destination locations via MoveBaseGoal messages.  Upon
#   successfully reaching each goal the node will publish a Zone message.
#------------------------------------------------------------------------------
xterm  -e  " roscore" & 
sleep 5

# launches gazebo and rviz with the navigation rviz config
xterm -e " roslaunch home_service_robot world.launch" &
sleep 5

# AMCL, tuned for the skid-steer robot
xterm -e " roslaunch home_service_robot amcl.launch" &
sleep 5

# add_markers node
xterm -e " roslaunch add_markers add_markers.launch" &
sleep 5

# pick objects node
xterm -e " roslaunch pick_objects pick_objects.launch" 

