#ifndef __OBJECT_RECOG_SERVER_OBJECT_RECOG_SERVER_NODE_H__
#define __OBJECT_RECOG_SERVER_OBJECT_RECOG_SERVER_NODE_H__

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <action_msg/ObjectAction.h>

class ObjectRecognitionServer
{
protected:
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<action_msg::ObjectAction> as_; 
  action_msg::ObjectResult result_;
  action_msg::ObjectFeedback feedback_;

public:
  ObjectRecognitionServer(const std::string &);
  ~ObjectRecognitionServer();
  void executeGoal(const action_msg::ObjectGoalConstPtr &);
};

#endif