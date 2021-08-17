#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <action_msg/MotionAction.h>
#include <action_msg/MarkerAction.h>
#include <action_msg/ObjectAction.h>
#include <action_msg/PlacementAction.h>
#include <geometry_msgs/Pose.h>
#include "pick_place_client_node.h"


PickPlaceClient::PickPlaceClient() : 
mc_("move", true),
mac_("marker", true),
oc_("object", true),
pc_("placement", true)
{
  ROS_INFO("Waiting for all action servers to start.");
  mc_.waitForServer();
  mac_.waitForServer();
  oc_.waitForServer();
  pc_.waitForServer();
  ROS_INFO("All action servers have started, state machines ACTIVATES");
}

PickPlaceClient::~PickPlaceClient()
{

}

void PickPlaceClient::stateMachines()
{
  // Move to stock
  action_msg::MotionGoal goal;
  createGoal(goal, "stock", 2, false, false);
  PickPlaceClient::move_stock(goal, 0);

  while (!(mc_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) && ros::ok())
  {
    ros::Duration(2.0).sleep();
    ROS_INFO("MOVING TO STOCK SHELF!");
  }
  if (mc_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("MOVING TO STOCK SHELF DONE!");

  // Recognize the marker
  PickPlaceClient::markerRecog();
  while (!(mac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED))
  {
    ros::Duration(2.0).sleep();
    ROS_INFO("RECOGNIZING THE MARKER!");
  }
  if (mac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("RECOGNIZING THE MARKER DONE!");

  // Recognize the object
  PickPlaceClient::objectRecog();
  while (!(oc_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED))
  {
    ros::Duration(2.0).sleep();
    ROS_INFO("RECOGNIZING THE OBJECT!");
  }
  if (oc_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("RECOGNIZING THE OBJECT DONE!");

  //pick the object
  action_msg::MotionGoal goal_pick;
  createGoal(goal_pick, "stock", -1, true, false);
  PickPlaceClient::move_stock(goal_pick, 2);

  while (!(mc_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED))
  {
    ros::Duration(2.0).sleep();
    ROS_INFO("PICKING THE OBJECT!");
  }
  if (mc_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("PICKING THE OBJECT DONE!");

  //move to display shelf
  action_msg::MotionGoal goal_display;
  createGoal(goal_display, "display", 1, false, false);
  PickPlaceClient::move_stock(goal_display, 1);

  while (!(mc_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED))
  {
    ros::Duration(2.0).sleep();
    ROS_INFO("MOVING TO DISPLAY SHELF!");
  }
  if (mc_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("MOVING TO DISPLAY SHELF DONE!");

  // Recognize the marker
  PickPlaceClient::markerRecog();
  while (!(mac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED))
  {
    ros::Duration(2.0).sleep();
    ROS_INFO("RECOGNIZING THE MARKER!");
  }
  if (mac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("RECOGNIZING THE MARKER DONE!");

  // Recognize the placement
  PickPlaceClient::placementRecog();
  while (!(pc_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED))
  {
    ros::Duration(2.0).sleep();
    ROS_INFO("RECOGNIZING THE PLACEMENT!");
  }
  if (pc_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("RECOGNIZING THE PLACEMENT DONE!");

  //place the object
  action_msg::MotionGoal goal_place;
  createGoal(goal_place, "display", -1, false, true);
  PickPlaceClient::move_stock(goal_place, 3);

  while (!(mc_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED))
  {
    ros::Duration(2.0).sleep();
    ROS_INFO("PLACING THE OBJECT!");
  }
  if (mc_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("PLACING THE OBJECT DONE!");
  else
    ROS_ERROR_STREAM("PICK AND PLACE FAILED");
}

void PickPlaceClient::move_stock(const action_msg::MotionGoal &goal, const int &num)
{
  switch (num)
  {
  case 0:
    mc_.sendGoal(goal,
              boost::bind(&PickPlaceClient::doneMoveStockCb, this, _1, _2),
              MotionClient::SimpleActiveCallback(),
              MotionClient::SimpleFeedbackCallback());
    break;

  case 1:
    mc_.sendGoal(goal,
              boost::bind(&PickPlaceClient::doneMoveDisplayCb, this, _1, _2),
              MotionClient::SimpleActiveCallback(),
              MotionClient::SimpleFeedbackCallback());
    break;

  case 2:
    mc_.sendGoal(goal,
              boost::bind(&PickPlaceClient::donePickCb, this, _1, _2),
              MotionClient::SimpleActiveCallback(),
              MotionClient::SimpleFeedbackCallback());
    break;

  case 3:
    mc_.sendGoal(goal,
              boost::bind(&PickPlaceClient::donePlaceCb, this, _1, _2),
              MotionClient::SimpleActiveCallback(),
              MotionClient::SimpleFeedbackCallback());
    break;
  default:
    ROS_ERROR_STREAM("Please enter valid action for motion execution");
    ros::shutdown();
    break;
  }
}

void PickPlaceClient::markerRecog()
{
  action_msg::MarkerGoal goal;
  goal.act = "detect marker";

  mac_.sendGoal(goal,
              boost::bind(&PickPlaceClient::doneMarkerCb, this, _1, _2),
              MarkerClient::SimpleActiveCallback(),
              MarkerClient::SimpleFeedbackCallback());
}

void PickPlaceClient::objectRecog()
{
  action_msg::ObjectGoal goal;
  goal.act = "detect object";

  oc_.sendGoal(goal,
              boost::bind(&PickPlaceClient::doneObjectCb, this, _1, _2),
              ObjectClient::SimpleActiveCallback(),
              ObjectClient::SimpleFeedbackCallback());
}

void PickPlaceClient::placementRecog()
{
  action_msg::PlacementGoal goal;
  goal.act = "detect Placement";

  pc_.sendGoal(goal,
              boost::bind(&PickPlaceClient::donePlacementCb, this, _1, _2),
              PlacementClient::SimpleActiveCallback(),
              PlacementClient::SimpleFeedbackCallback());
}

void PickPlaceClient::donePickCb(const actionlib::SimpleClientGoalState& state,
                                  const action_msg::MotionResultConstPtr& result)
{
  if(state.toString() == "SUCCEEDED")
    ROS_INFO("Finished in state [%s]", state.toString().c_str());
  else
    ROS_WARN("Finished in state [%s]", state.toString().c_str());

  if (result->done)
    ROS_INFO_NAMED("SUCCESS", "PICKED the object");
  else 
  {
    ROS_ERROR_STREAM("Pick Operation FAILED, , SHUTTING DOWN");
    ros::shutdown();
  }
}

void PickPlaceClient::doneMoveStockCb(const actionlib::SimpleClientGoalState& state,
                                    const action_msg::MotionResultConstPtr& result)
{
  if(state.toString() == "SUCCEEDED")
    ROS_INFO("Finished in state [%s]", state.toString().c_str());
  else
    ROS_WARN("Finished in state [%s]", state.toString().c_str());

  if (result->done)
    ROS_INFO_NAMED("SUCCESS", "MOVED to STOCK shelf");
  else 
  {
    ROS_ERROR_STREAM("Moving to STOCK SHELF FAILED, SHUTTING DOWN");
    ros::shutdown();
  }
}

void PickPlaceClient::doneMoveDisplayCb(const actionlib::SimpleClientGoalState& state,
                                    const action_msg::MotionResultConstPtr& result)
{
  if(state.toString() == "SUCCEEDED")
    ROS_INFO("Finished in state [%s]", state.toString().c_str());
  else
    ROS_WARN("Finished in state [%s]", state.toString().c_str());

  if (result->done)
    ROS_INFO_NAMED("SUCCESS", "MOVED to DISPLAY shelf");
  else 
  {
    ROS_ERROR_STREAM("Moving to DISPLAY SHELF FAILED, SHUTTING DOWN");
    ros::shutdown();
  }
}

void PickPlaceClient::donePlaceCb(const actionlib::SimpleClientGoalState& state,
                                    const action_msg::MotionResultConstPtr& result)
{
  if(state.toString() == "SUCCEEDED")
    ROS_INFO("Finished in state [%s]", state.toString().c_str());
  else
    ROS_WARN("Finished in state [%s]", state.toString().c_str());

  if (result->done)
    ROS_INFO_NAMED("SUCCESS", "PLACED the object");
  else 
  {
    ROS_ERROR_STREAM("Place Operation FAILED, SHUTTING DOWN");
    ros::shutdown();
  }
}

void PickPlaceClient::doneMarkerCb(const actionlib::SimpleClientGoalState& state,
                                    const action_msg::MarkerResultConstPtr& result)
{
  if(state.toString() == "SUCCEEDED")
    ROS_INFO("Finished in state [%s]", state.toString().c_str());
  else
    ROS_WARN("Finished in state [%s]", state.toString().c_str());

  if (result->done)
    ROS_INFO_NAMED("SUCCESS", "Marker Recognition DONE!");
  else 
  {
    ROS_ERROR_STREAM("Marker Recognition FAILED, SHUTTING DOWN");
    ros::shutdown();
  }
}

void PickPlaceClient::doneObjectCb(const actionlib::SimpleClientGoalState& state,
                                    const action_msg::ObjectResultConstPtr& result)
{
  if(state.toString() == "SUCCEEDED")
    ROS_INFO("Finished in state [%s]", state.toString().c_str());
  else
    ROS_WARN("Finished in state [%s]", state.toString().c_str());

  if (result->done)
    ROS_INFO_NAMED("SUCCESS", "Object Recognition DONE!");
  else 
  {
    ROS_ERROR_STREAM("Object Recognition FAILED, SHUTTING DOWN");
    ros::shutdown();
  }
}

void PickPlaceClient::donePlacementCb(const actionlib::SimpleClientGoalState& state,
                                    const action_msg::PlacementResultConstPtr& result)
{
  if(state.toString() == "SUCCEEDED")
    ROS_INFO("Finished in state [%s]", state.toString().c_str());
  else
    ROS_WARN("Finished in state [%s]", state.toString().c_str());

  if (result->done)
    ROS_INFO_NAMED("SUCCESS", "Placement Recognition DONE!");
  else 
  {
    ROS_ERROR_STREAM("Placement Recognition FAILED, SHUTTING DOWN");
    ros::shutdown();
  }
}

void PickPlaceClient::createGoal(action_msg::MotionGoal &goal, const std::string &str,
                                    const int &id, const bool &pick, const bool &place)
{
  goal.shelf = str;
  goal.move_id = id;
  goal.pick = pick;
  goal.place = place;
}


int main (int argc, char **argv)
{
  ros::init(argc, argv, "motion_execution_client");
  PickPlaceClient pickplaceClient;
  pickplaceClient.stateMachines();
  ros::spin();
  return 0;
}