#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <action_msg/ObjectAction.h>
#include "object_recog_server_node.h"

ObjectRecognitionServer::ObjectRecognitionServer(const std::string &name):
as_(nh_, name, boost::bind(&ObjectRecognitionServer::executeGoal, this, _1), false)
{
  as_.start();
  ROS_INFO("Dummy Action Server for Object Recognition has started");
}

ObjectRecognitionServer::~ObjectRecognitionServer()
{

}

void ObjectRecognitionServer::executeGoal(const action_msg::ObjectGoalConstPtr &goal)
{
    ROS_INFO("Goal for Object Recognition Received");
    ROS_INFO("Sleeping for 1 second");
    ros::Duration(1.0).sleep();  // sleep for 1 second;
    result_.done = true;
    as_.setSucceeded(result_);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "object_recog_server");
  ObjectRecognitionServer objectServer("object");
  ros::spin();
  return 0;
}