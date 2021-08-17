#ifndef __PLACEMENT_RECOG_SERVER_PLACEMENT_RECOG_SERVER_NODE_H__
#define __PLACEMENT_RECOG_SERVER_PLACEMENT_RECOG_SERVER_NODE_H__

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <action_msg/PlacementAction.h>

class PlacementRecognitionServer
{
protected:
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<action_msg::PlacementAction> as_; 
  action_msg::PlacementResult result_;
  action_msg::PlacementFeedback feedback_;

public:
  PlacementRecognitionServer(const std::string &);
  ~PlacementRecognitionServer();
  void executeGoal(const action_msg::PlacementGoalConstPtr &);
};

#endif