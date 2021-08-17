#ifndef __MARKER_RECOG_SERVER_MARKER_RECOG_SERVER_NODE_H__
#define __MARKER_RECOG_SERVER_MARKER_RECOG_SERVER_NODE_H__

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <action_msg/MarkerAction.h>

class MarkerRecognitionServer
{
protected:
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<action_msg::MarkerAction> as_; 
  action_msg::MarkerResult result_;
  action_msg::MarkerFeedback feedback_;

public:
  MarkerRecognitionServer(const std::string &);
  ~MarkerRecognitionServer();
  void executeGoal(const action_msg::MarkerGoalConstPtr &);
};

#endif