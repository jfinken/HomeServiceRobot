#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <pick_objects/Zone.h>

class MarkerHandler 
{
public:
  MarkerHandler()
  {
    // publish viz markers to rviz
    pub_ = n_.advertise<visualization_msgs::Marker>("visualization_marker", 1);

    // subscribe to a custom zone msg on whether or not to show a marker
    // and if so at what pose.
    sub_ = n_.subscribe("/home_service_robot/zone", 3, &MarkerHandler::zoneCb, this);
  }

  void zoneCb(const pick_objects::Zone::ConstPtr& msg)
  {
    ROS_INFO("add_markers: received %s marker...", msg->meta.c_str());

    visualization_msgs::Marker marker;
    // The odom frame is the frame into which all data is transformed before
    // being displayed.
    marker.header.frame_id = "odom";
    marker.header.stamp = ros::Time::now();
    
    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.5;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 0.0f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0;

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "home-service-robot";
    marker.id = msg->id;
    marker.type = visualization_msgs::Marker::CUBE;

    // Set the marker action, ADD or DELETE, based on the metadata.
    if (msg->meta.compare("add") == 0)
      marker.action = visualization_msgs::Marker::ADD;
    else if (msg->meta.compare("remove") == 0)
      marker.action = visualization_msgs::Marker::DELETE;

    // pose from the msg
    marker.pose.position = msg->pose.position;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    
    pub_.publish(marker);
  }

private:
  ros::NodeHandle n_; 
  ros::Publisher pub_;
  ros::Subscriber sub_;

};//End of MarkerHandler 

int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");

  // Handle receiving Zone msgs and publishing markers
  MarkerHandler handler;

  ros::spin();

  return 0;
}
