#include "turtlebot_highlevel_controller/TurtlebotHighlevelController.hpp"
#include <string>

namespace turtlebot_highlevel_controller {

TurtlebotHighlevelController::TurtlebotHighlevelController(ros::NodeHandle& nodeHandle) :
  nodeHandle_(nodeHandle)
{
  if (!readParameters()) {
    ROS_ERROR("Could not read parameters.");
    ros::requestShutdown();
  }
  subscriber_ = nodeHandle_.subscribe(subscriberTopic_, 1,
                                      &TurtlebotHighlevelController::topicCallback, this);
  serviceServer_ = nodeHandle_.advertiseService("get_average",
                                                &TurtlebotHighlevelController::serviceCallback, this);
  ROS_INFO("Successfully launched node.");

}

TurtlebotHighlevelController::~TurtlebotHighlevelController()
{
}

bool TurtlebotHighlevelController::readParameters()
{
  if (!nodeHandle_.getParam("subscriber_topic", subscriberTopic_)) return false;
  return true;
}

void TurtlebotHighlevelController::topicCallback(const sensor_msgs::Temperature& message)
{
  //algorithm_.addData(message.temperature);
}

bool TurtlebotHighlevelController::serviceCallback(std_srvs::Trigger::Request& request,
                                         std_srvs::Trigger::Response& response)
{
  response.success = true;
  //response.message = "The average is " + std::to_string(algorithm_.getAverage());
  return true;
}

} /* namespace */
