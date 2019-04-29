#include <ros/ros.h>
#include "std_msgs/String.h"
#include "turtlebot_highlevel_controller/TurtlebotHighlevelController.hpp"
#include "sensor_msgs/LaserScan.h"
#include <sstream>

void chatterCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  //ROS_INFO("I heard: [%s]", msg->data.c_str());
  ROS_INFO("Ausgabe: [%s]", msg->range_min);
  
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "listener");
  ros::NodeHandle mynode;
  ros::Subscriber subscriber = mynode.subscribe("scan", 3, chatterCallback);
  //turtlebot_highlevel_controller::TurtlebotHighlevelController TurtlebotHighlevelController(nodeHandle);

  ros::spin();
  return 0;
}