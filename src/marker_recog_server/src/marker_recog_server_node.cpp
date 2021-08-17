#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <action_msg/MarkerAction.h>
#include "marker_recog_server_node.h"

MarkerRecognitionServer::MarkerRecognitionServer(const std::string &name):
as_(nh_, name, boost::bind(&MarkerRecognitionServer::executeGoal, this, _1), false)
{
  as_.start();
  ROS_INFO("Dummy Action Server for Marker Recognition has started");
}

MarkerRecognitionServer::~MarkerRecognitionServer()
{

}

void MarkerRecognitionServer::executeGoal(const action_msg::MarkerGoalConstPtr &goal)
{
    ROS_INFO("Goal for Marker Recognition Received");
    ROS_INFO("Sleeping for 1 second");
    ros::Duration(1.0).sleep();  // sleep for 1 second;
    result_.done = true;
    as_.setSucceeded(result_);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "marker_recog_server");
  MarkerRecognitionServer markerServer("marker");
  ros::spin();
  return 0;
}