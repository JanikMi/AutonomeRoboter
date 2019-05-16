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
#include <geometry_msgs/Twist.h>

sensor_msgs::LaserScan Pub_scan;

float SizeOfArray;
double ranges[5];

geometry_msgs::Twist base_cmd_turn_left;

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
  
  ros::Subscriber subscriber = nodeHandle_.subscribe(topic, queue_size, &TurtlebotHighlevelController::chatterCallback, this);
  publisher_ = nodeHandle_.advertise<sensor_msgs::LaserScan>("Pub_scan", queue_size);
  cmd_vel_pub_ = nodeHandle_.advertise<geometry_msgs::Twist>("cmd_vel", 100);
   //ros::Publisher cmd_vel_pub_ = nodeHandle_.advertise<geometry_msgs::Twist>("cmd_vel", 10);


  ROS_INFO("Successfully launched node.");

  ros::Rate r(10); // 100 hz
  while (ros::ok())
  {
   //publisher_.publish(Pub_scan);
    
   //cmd_vel_pub_.publish(base_cmd);
    //cmd_vel_pub_.publish(msg1);`

    ros::spinOnce();
    r.sleep();
  }
  ROS_INFO("Shutdown node");
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
  //ros::Publisher cmd_vel_pub;
  SizeOfArray = msg->ranges.size();
  //msg->ranges->resize(SizeOfArray);
  Pub_scan.ranges.resize(5);
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

  if (index < 0) index = 0;
  if (index > 640) index = 640;
  ROS_INFO("Aktueller Index: [%i]", index);
  
  /**/
  if (index < 3)
      {
        Pub_scan.ranges[0] = 0;
        Pub_scan.ranges[1] = 0;
        AusgabeSubscriber[0] = 0;
        AusgabeSubscriber[1] = 0;
      }
      else 
      {
        Pub_scan.ranges[0] = msg->ranges[index-2];
        Pub_scan.ranges[1] = msg->ranges[index-1];
        AusgabeSubscriber[0] = msg->ranges[index-2];
        AusgabeSubscriber[1] = msg->ranges[index-1];
      }
      if (index == 640)
      {
        Pub_scan.ranges[3] = msg->ranges[index];
        Pub_scan.ranges[4] = msg->ranges[index];
        AusgabeSubscriber[3] = msg->ranges[index];
        AusgabeSubscriber[4] = msg->ranges[index];
      }
      else 
      {
        Pub_scan.ranges[3] = msg->ranges[index+1];
        Pub_scan.ranges[4] = msg->ranges[index+2];
        AusgabeSubscriber[3] = msg->ranges[index+1];
        AusgabeSubscriber[4] = msg->ranges[index+2];
      }
      Pub_scan.ranges[2]   = msg->ranges[index];
      AusgabeSubscriber[2] = msg->ranges[index];

  //ROS_INFO("Distanzen: [%f], [%f], [%f], [%f], [%f]", Pub_scan.ranges[0],Pub_scan.ranges[1],Pub_scan.ranges[2],Pub_scan.ranges[3],Pub_scan.ranges[4]);
  ROS_INFO("Distanzen: [%f], [%f], [%f], [%f], [%f]", AusgabeSubscriber[0],AusgabeSubscriber[1],AusgabeSubscriber[2],AusgabeSubscriber[3],AusgabeSubscriber);

  /*
      //ros::Rate r(1.0);
    ros::Time scan_time = ros::Time::now();
    publisher_msg.header.stamp = scan_time;
    publisher_msg.header.frame_id = "base_laser_link";    
 */   
    Pub_scan.header.stamp = msg->header.stamp;
    Pub_scan.header.frame_id = msg->header.frame_id;

    // +/- 90°:
    //Pub_scan.angle_increment = 0.00158417993225;
    Pub_scan.angle_increment = msg->angle_increment;

    Pub_scan.angle_min = msg->angle_min+(index-2)*Pub_scan.angle_increment;
    Pub_scan.angle_max = msg->angle_max+(index+2)*Pub_scan.angle_increment;

    //scan.time_increment = (1 / laser_frequency) / (num_readings);
    Pub_scan.range_min = msg->range_min;
    Pub_scan.range_max = msg->range_max;



  geometry_msgs::Twist base_cmd;
  base_cmd.linear.x = 1;
  base_cmd.linear.y =0.6;
  base_cmd.linear.z=0;
  base_cmd.angular.x = 1.2;

  
  //publisher.publish(base_cmd);
  cmd_vel_pub_.publish(base_cmd);
  publisher_.publish(Pub_scan);
}
}

/*
bool TurtlebotHighlevelController::serviceCallback(std_srvs::Trigger::Request& request,
                                         std_srvs::Trigger::Response& response)
{
  response.success = true;
  response.message = "The average is " + std::to_string(algorithm_.getAverage());
  return true;
}
}
*/
// /* namespace */
