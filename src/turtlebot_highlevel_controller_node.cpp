#include <ros/ros.h>
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "turtlebot_highlevel_controller/TurtlebotHighlevelController.hpp"
#include "sensor_msgs/LaserScan.h"
#include <sstream>
#include <ros/param.h>
#include <iostream>

sensor_msgs::LaserScan publisher_msg;
unsigned int SizeOfArray;
double ranges[5];

void chatterCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  SizeOfArray = sizeof(msg->ranges);
  //msg->ranges->resize(SizeOfArray);
  publisher_msg.ranges.resize(5);
  for (int i=1; i<SizeOfArray-1;i++)
  {
    if (msg->ranges[i]<msg->range_min)
    {
      if (i < 3)
      {
        publisher_msg.ranges[0] = 0;
        publisher_msg.ranges[1] = 0;
      }
      else 
      {
        publisher_msg.ranges[0] = msg->ranges[i-2];
        publisher_msg.ranges[1] = msg->ranges[i-1];
      }
      if (i == 640)
      {
        publisher_msg.ranges[3] = msg->ranges[i];
        publisher_msg.ranges[4] = msg->ranges[i];
      }
      else 
      {
        publisher_msg.ranges[3] = msg->ranges[i+1];
        publisher_msg.ranges[4] = msg->ranges[i+2];
      }
      publisher_msg.ranges[2] = msg->ranges[i];
    }  
    ROS_INFO("Distanzen: [%f], [%f], [%f], [%f], [%f]", publisher_msg.ranges[0],publisher_msg.ranges[1],publisher_msg.ranges[2],publisher_msg.ranges[3],publisher_msg.ranges[4]);
  }
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
  ros::Publisher publisher = mynode.advertise<sensor_msgs::LaserScan>("/Pub_scan", queue_size);

    //ros::Rate r(1.0);
    ros::Time scan_time = ros::Time::now();
    
    publisher_msg.header.stamp = scan_time;
    publisher_msg.header.frame_id = "laser_frame";
    // +/- 90°:
    publisher_msg.angle_min = -1.57;
    publisher_msg.angle_max = 1.57;
    publisher_msg.angle_increment = 3.14 / 5;
    //scan.time_increment = (1 / laser_frequency) / (num_readings);
    publisher_msg.range_min = 0.0;
    publisher_msg.range_max = 100.0;

    publisher.publish(publisher_msg);
    //ros::spinOnce();

    
   





  ros::spin();
  return 0;
}