#ifndef __MOVEIT_PNP_SERVER_MOVEIT_NODE_H__
#define __MOVEIT_PNP_SERVER_MOVEIT_NODE_H__

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <actionlib/server/simple_action_server.h>
#include <action_msg/MotionAction.h>
#include <geometry_msgs/Pose.h>

#include <xmlrpcpp/XmlRpcValue.h>

class MotionServer
{
protected:
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<action_msg::MotionAction> as_; 
  action_msg::MotionFeedback feedback_;
  action_msg::MotionResult result_;

public:
  MotionServer(const std::string &);
  ~MotionServer();

  // goal callback
  void executeGoal(const action_msg::MotionGoalConstPtr &);

  // Function to get stock shelf parameters
  void stock_shelf_param(const XmlRpc::XmlRpcValue &, geometry_msgs::Pose &,
                                          const std::string &, std::vector<double> &);

  // Function to get display shelf parameters
  void display_shelf_param(const XmlRpc::XmlRpcValue &, geometry_msgs::Pose &, const std::string &);

  // Function to move the robot using moveit
  void move(const geometry_msgs::Pose &, moveit::planning_interface::MoveItErrorCode &,
                                                        moveit::planning_interface::MoveItErrorCode &);

  // create an object in rviz
  void addCollisionObjects(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface, 
                                          const std::vector<double> &pos, const geometry_msgs::Pose &Pose);

  // pick object through move_it
  void pick_object(moveit::planning_interface::MoveGroupInterface& , const geometry_msgs::Pose &,
                                                            moveit::planning_interface::MoveItErrorCode &);

  // open and close the gripper
  void openGripper(trajectory_msgs::JointTrajectory& );
  void closedGripper(trajectory_msgs::JointTrajectory& );

  // place the object
  void place_object(moveit::planning_interface::MoveGroupInterface& , const geometry_msgs::Pose &,
                                                           moveit::planning_interface::MoveItErrorCode &);
};

#endif