#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <action_msg/PlacementAction.h>
#include "placement_recog_server_node.h"

PlacementRecognitionServer::PlacementRecognitionServer(const std::string &name):
as_(nh_, name, boost::bind(&PlacementRecognitionServer::executeGoal, this, _1), false)
{
  as_.start();
  ROS_INFO("Dummy Action Server for Placement Recognition has started");
}

PlacementRecognitionServer::~PlacementRecognitionServer()
{

}

void PlacementRecognitionServer::executeGoal(const action_msg::PlacementGoalConstPtr &goal)
{
    ROS_INFO("Goal for Placement Recognition Received");
    ROS_INFO("Sleeping for 1 second");
    ros::Duration(1.0).sleep();  // sleep for 1 second;
    result_.done = true;
    as_.setSucceeded(result_);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "placement_recog_server");
  PlacementRecognitionServer placementServer("placement");
  ros::spin();
  return 0;
}