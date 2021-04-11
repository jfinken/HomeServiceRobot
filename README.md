## Q802 Home Service Robot

### Installation

```bash

# system
$ sudo aptitude install ros-noetic-openslam-gmapping
$ sudo aptitude install ros-noetic-joy

$ mkdir -p catkin_ws/src && cd catkin_ws/src
$ catkin_init_workspace

# gmapping
$ git clone git@github.com:ros-perception/slam_gmapping.git

# manual keyboard control
# $ git clone git@github.com:turtlebot/turtlebot.git
$ git clone https://github.com/ros-teleop/teleop_twist_keyboard

# If using the TurtleBot consider the below.  
# However I am using the below a skid-steer robot in home_service_robot

# Utilizing the view_navigation.launch file, we'll load a preconfigured rviz workspace. 
# thus saving time as it will automatically load the robot model, trajectories, and map.
# $ git clone git@github.com:turtlebot/turtlebot_interactions.git

# With the turtlebot_world.launch we'll deploy a turtlebot in a gazebo environment 
# by linking the world file to it.
# $ git@github.com:turtlebot/turtlebot_simulator.git
```

### Run: WIP

**TODO:**
- Use a launch file for slam_gmapping. Change the map_update_interval to around 30.
- For the hokuyo view angle, consider -95 to 95 degree, or the defaults.
- Set the maxUrange to the max distance of the laser scanner.
- [ref](https://answers.ros.org/question/102966/gmapping-issues-with-laser-pose/)

```bash
$ roslaunch home_service_robot world.launch

# NOTE: the base_frame of my skid-steer robot is "chassis"
$ rosrun gmapping slam_gmapping scan:=scan _base_frame:=chassis
```