#include <ros/ros.h>
#include "std_msgs/String.h"
#include "turtlebot_highlevel_controller/TurtlebotHighlevelController.hpp"
#include "sensor_msgs/LaserScan.h"

void chatterCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  //ROS_INFO("I heard: [%s]", msg->data.c_str());
  ROS_INFO("I heard: [%s]", msg->range_min);
  
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "turtlebot_highlevel_controller");
  //ros::NodeHandle nodeHandle("~");
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("scan", 10, chatterCallback);
  //turtlebot_highlevel_controller::TurtlebotHighlevelController TurtlebotHighlevelController(nodeHandle);

  ros::spin();
  return 0;
}