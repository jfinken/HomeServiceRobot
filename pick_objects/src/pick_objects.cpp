#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <pick_objects/Zone.h>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){

  // Initialize the pick_objects node
  ros::init(argc, argv, "pick_objects");
  ros::NodeHandle n;
  // We'll publish this custom msg when we successfully reach a 
  // pickup or dropoff zone
  ros::Publisher zone_pub = n.advertise<pick_objects::Zone>("/home_service_robot/zone", 3);

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  //---------------------------------------------------------------------------
  // Declare pickup zone 
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
  // Declare dropoff zone
  //----------------------------------------------------------------------------
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
  
  //----------------------------------------------------------------------------
  // Declare Zone msgs for the pickup and drop-off zones that will be published 
  // on successfully arriving at the pickup zone.
  //----------------------------------------------------------------------------
  pick_objects::Zone pickup_zone;
  pickup_zone.id = 0;
  pickup_zone.meta = "add";
  pickup_zone.pose = goal1.target_pose.pose;
  
  // publish a zone msg to show the marker at the pickup zone
  ROS_INFO("Publishing pick-up zone marker.");
  zone_pub.publish(pickup_zone);

  // Send goals
  // TODO: DRY-out the below result checking 
  ROS_INFO("Sending goal: pick-up zone");
  ac.sendGoal(goal1);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if we've reached pickup zone 1, if so proceed to 2. 
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_INFO("The q802 robot successfully reached pickup zone.");

    // publish a zone msg to remove the marker from the pickup zone 
    ROS_INFO("Publishing to remove pick-up zone marker.");
    pickup_zone.meta = "remove";
    zone_pub.publish(pickup_zone);
  }
  else {
    ROS_INFO("The base failed to move to the pick-up zone.  Exiting...");
    return 1;
  }

  // sleep before sending next goal
  ros::Duration(1.0).sleep();

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
  ros::Duration(4.0).sleep();

  ROS_INFO("Sending goal: drop-off zone");
  ac.sendGoal(goal3);
  
  ac.waitForResult();
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_INFO("The q802 robot successfully reached the drop-off zone.");

    // show a marker at the drop off zone given the robot has successfully
    // reached it! 
    pickup_zone.meta = "add";
    pickup_zone.pose = goal3.target_pose.pose;
    ROS_INFO("Publishing to add drop-off zone marker.");
    zone_pub.publish(pickup_zone);
  } else {
    ROS_INFO("The q802 robot failed to move to the drop-off zone.  Exiting...");
    return 1;
  }

  ros::spin();

  return 0;
}
