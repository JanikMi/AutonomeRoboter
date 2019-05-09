#include <ros/ros.h>
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "sensor_msgs/LaserScan.h"
#include <sstream>
#include <ros/param.h>
#include <iostream>
#include <vector>
#include <time.h>
#include "turtlebot_highlevel_controller/TurtlebotHighlevelController.hpp"
#include <string>

sensor_msgs::LaserScan publisher_msg;
float SizeOfArray;
double ranges[5];

namespace HighlevelController {

TurtlebotHighlevelController::TurtlebotHighlevelController(ros::NodeHandle& nodeHandle): nodeHandle_(nodeHandle)
{
  int queue_size;
  std::string topic;
  if(!nodeHandle.getParam("/queue_size", queue_size))
  {
      ROS_ERROR_STREAM("queue_size wurde nicht gelesen.");
      ros::requestShutdown();
  }
  if(!nodeHandle.getParam("/topic", topic))
  {
      ROS_ERROR_STREAM("topic wurde nicht gelesen.");
      ros::requestShutdown();
  }
  
  ros::Subscriber subscriber = nodeHandle_.subscribe("/scan", queue_size, &TurtlebotHighlevelController::chatterCallback, this);
  ros::Publisher publisher = nodeHandle_.advertise<sensor_msgs::LaserScan>("/Pub_scan", queue_size);

  publisher.publish(publisher_msg);

  ROS_INFO("Successfully launched node.");
  ros::spin();
}

TurtlebotHighlevelController::~TurtlebotHighlevelController()
{
}

bool TurtlebotHighlevelController::readParameters()
{
}

void TurtlebotHighlevelController::topicCallback(const sensor_msgs::Temperature& message)
{
  //algorithm_.addData(message.temperature);
}

void TurtlebotHighlevelController::chatterCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{


  SizeOfArray = msg->ranges.size();
  //msg->ranges->resize(SizeOfArray);
  //publisher_msg.ranges.resize(5);
  float AusgabeSubscriber[5];
  unsigned int index;
  float Min_Distanz = msg->ranges[0];
  for (int i=0; i<SizeOfArray;i++)
  {
    if (msg->ranges[i] < Min_Distanz)
    {
      
      Min_Distanz = msg->ranges[i];
      index = i;
    } 
  }
  //ROS_INFO("Minimale Distanz: [%f]", Min_Distanz) ;

  /**/
  if (index < 3)
      {
        //publisher_msg.ranges[0] = 0;
        //publisher_msg.ranges[1] = 0;
        AusgabeSubscriber[0] = 0;
        AusgabeSubscriber[1] = 0;
      }
      else 
      {
        //publisher_msg.ranges[0] = msg->ranges[index-2];
        //publisher_msg.ranges[1] = msg->ranges[index-1];
        AusgabeSubscriber[0] = msg->ranges[index-2];
        AusgabeSubscriber[1] = msg->ranges[index-1];
      }
      if (index == 640)
      {
        //publisher_msg.ranges[3] = msg->ranges[index];
        //publisher_msg.ranges[4] = msg->ranges[index];
        AusgabeSubscriber[3] = msg->ranges[index];
        AusgabeSubscriber[4] = msg->ranges[index];
      }
      else 
      {
        //publisher_msg.ranges[3] = msg->ranges[index+1];
        //publisher_msg.ranges[4] = msg->ranges[index+2];
        AusgabeSubscriber[3] = msg->ranges[index+1];
        AusgabeSubscriber[4] = msg->ranges[index+2];
      }
      //publisher_msg.ranges[2] = msg->ranges[index];
      AusgabeSubscriber[2] = msg->ranges[index];

  ROS_INFO("Distanzen: [%f], [%f], [%f], [%f], [%f]", AusgabeSubscriber[0],AusgabeSubscriber[1],AusgabeSubscriber[2],AusgabeSubscriber[3],AusgabeSubscriber[4]);

 
  
      //ros::Rate r(1.0);
    ros::Time scan_time = ros::Time::now();
    publisher_msg.header.stamp = scan_time;
    publisher_msg.header.frame_id = "base_laser_link";
    // +/- 90°:
    publisher_msg.angle_increment = 0.00158417993225;
    publisher_msg.angle_min = (index-2)*publisher_msg.angle_increment*360/2/3.1415;
    publisher_msg.angle_max = (index+2)*publisher_msg.angle_increment*360/2/3.1415;
    //scan.time_increment = (1 / laser_frequency) / (num_readings);
    publisher_msg.range_min = 0.0;
    publisher_msg.range_max = 5.0;
    
}

/*
bool TurtlebotHighlevelController::serviceCallback(std_srvs::Trigger::Request& request,
                                         std_srvs::Trigger::Response& response)
{
  response.success = true;
  response.message = "The average is " + std::to_string(algorithm_.getAverage());
  return true;
}
*/
} /* namespace */
