#include <ros/ros.h>
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "turtlebot_highlevel_controller/TurtlebotHighlevelController.hpp"
#include "sensor_msgs/LaserScan.h"
#include <sstream>
#include <ros/param.h>
#include <iostream>

void chatterCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
std_msgs::Float32 smallest_range;
float all_distances[5];
float h = sizeof(msg->ranges);
smallest_range.data = msg -> ranges[1];

for (int i=1; i<h-1;i++)
{
  if (msg->ranges[i]<smallest_range.data)
  {
    //smallest_range.data;
    if (i < 3)
    {
      all_distances[0] = 0;
      all_distances[1] = 0;
    }
    else
    {
      all_distances[0] = msg->ranges[i-2];
      all_distances[1] = msg->ranges[i-1];
    }
    all_distances[2] = msg->ranges[i];
    all_distances[3] = msg->ranges[i+1];
    all_distances[4] = msg->ranges[i+2];
    ROS_INFO("Distanzen: [%f], [%f], [%f], [%f], [%f]", all_distances[0],all_distances[1],all_distances[2],all_distances[3],all_distances[4]);
  }
  
}


  //ROS_INFO("Ausgabe: [%f]", msg->range_max);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "listener");
  ros::NodeHandle mynode;

  int queue_size;
  std::string topic;
  if(!mynode.getParam("/queue_size", queue_size))
  {
      ROS_ERROR_STREAM("queue_size wurde nicht gelesen.");
      return 0;
  }
  if(!mynode.getParam("/topic", topic))
  {
      ROS_ERROR_STREAM("topic wurde nicht gelesen.");
      return 0;
  }

  ros::Subscriber subscriber = mynode.subscribe(topic, queue_size, chatterCallback);
  //ros::Subscriber subscriber = mynode.subscribe("/scan", 100, chatterCallback);
  //turtlebot_highlevel_controller::TurtlebotHighlevelController TurtlebotHighlevelController(nodeHandle);

  ros::spin();
  return 0;
}