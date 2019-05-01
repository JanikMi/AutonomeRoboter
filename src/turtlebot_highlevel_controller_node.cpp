#include <ros/ros.h>
#include "std_msgs/String.h"
#include "turtlebot_highlevel_controller/TurtlebotHighlevelController.hpp"
#include "sensor_msgs/LaserScan.h"
#include <sstream>
#include <ros/param.h>
#include <iostream>

void chatterCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  //ROS_INFO("I heard: [%s]", msg->data.c_str());
  ROS_INFO("Ausgabe: [%s]", msg->range_min);
  
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "listener");
  ros::NodeHandle mynode;

  int queue_size;
  std::string topic;
  if(!mynode.getParam("/_subscriber/queue_size", queue_size))
  {
      ROS_ERROR_STREAM("queue_size wurde nicht gelesen.");
      return 0;
  }
  if(!mynode.getParam("/_subscriber/topic", topic))
  {
      ROS_ERROR_STREAM("topic wurde nicht gelesen.");
      return 0;
  }

  ros::Subscriber subscriber = mynode.subscribe(topic, queue_size, chatterCallback);
  //turtlebot_highlevel_controller::TurtlebotHighlevelController TurtlebotHighlevelController(nodeHandle);

  ros::spin();
  return 0;
}