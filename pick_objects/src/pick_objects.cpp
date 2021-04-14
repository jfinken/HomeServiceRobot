#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  // Initialize the pick_objects node
  ros::init(argc, argv, "pick_objects");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  //---------------------------------------------------------------------------
  // Declare pickup zone #1
  //  Helpful: rviz + rostopic echo /move_base_simple/goal
  //---------------------------------------------------------------------------
  move_base_msgs::MoveBaseGoal goal1;

  // set up the frame parameters
  goal1.target_pose.header.frame_id = "map";
  goal1.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation for the robot to reach
  goal1.target_pose.pose.position.x = -6.51;
  goal1.target_pose.pose.position.y = -5.19;
  //goal1.target_pose.pose.orientation.z = -0.296;
  goal1.target_pose.pose.orientation.w = 0.95;

  //---------------------------------------------------------------------------
  // Declare interim goal A
  //---------------------------------------------------------------------------
  move_base_msgs::MoveBaseGoal goal2;
  goal2.target_pose.header.frame_id = "map";
  goal2.target_pose.header.stamp = ros::Time::now();
  goal2.target_pose.pose.position.x = -7.16;
  goal2.target_pose.pose.position.y = -6.75;
  goal2.target_pose.pose.orientation.z = 0.84;
  goal2.target_pose.pose.orientation.w = 0.5327;
  
  //---------------------------------------------------------------------------
  // Declare pickup zone 2
  //------------------------------------------------------------l---------------
  move_base_msgs::MoveBaseGoal goal3;
  goal3.target_pose.header.frame_id = "map";
  goal3.target_pose.header.stamp = ros::Time::now();
  goal3.target_pose.pose.position.x = -5.5;
  goal3.target_pose.pose.position.y = 3.7;
  goal3.target_pose.pose.orientation.w = 0.7;

  // More difficult pickup zone 2
  //goal2.target_pose.pose.position.x = -14.0;
  //goal2.target_pose.pose.position.y = -6.0;
  //goal2.target_pose.pose.orientation.z = 0.72;
  //goal2.target_pose.pose.orientation.w = 0.70;

  // Send goals
  // TODO: DRY-out the below result checking 
  ROS_INFO("Sending goal #1");
  ac.sendGoal(goal1);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if we've reached pickup zone 1, if so proceed to 2. 
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_INFO("The q802 robot successfully reached pickup zone 1");
  }
  else {
    ROS_INFO("The base failed to move to goal #1.  Exiting...");
    return 1;
  }

  // sleep before sending next goal
  ros::Duration(2.0).sleep();

  ROS_INFO("Sending interim goal A");
  ac.sendGoal(goal2);

  // Wait an infinite time for the results
  ac.waitForResult();
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("The q802 robot successfully reached interim goal A");
  else {
    ROS_INFO("The q802 robot failed to move to interim goal A.  Exiting...");
    return 1;
  }
  
  // sleep before sending next goal
  ros::Duration(2.0).sleep();

  ROS_INFO("Sending pickup zone 2");
  ac.sendGoal(goal3);
  
  ac.waitForResult();
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("The q802 robot successfully reached pickup zone 2");
  else {
    ROS_INFO("The q802 robot failed to move to pickup zone 2.  Exiting...");
    return 1;
  }

  // sleep before exiting...
  ros::Duration(5.0).sleep();
  return 0;
}
