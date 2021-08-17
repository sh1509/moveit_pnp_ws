#ifndef __PICK_PLACE_CLIENT_NODE_H__
#define __PICK_PLACE_CLIENT_NODE_H__

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <action_msg/MotionAction.h>
#include <action_msg/MarkerAction.h>
#include <action_msg/ObjectAction.h>
#include <action_msg/PlacementAction.h>


typedef actionlib::SimpleActionClient<action_msg::MotionAction> MotionClient;
typedef actionlib::SimpleActionClient<action_msg::MarkerAction> MarkerClient;
typedef actionlib::SimpleActionClient<action_msg::ObjectAction> ObjectClient;
typedef actionlib::SimpleActionClient<action_msg::PlacementAction> PlacementClient;

class PickPlaceClient
{
public:
  PickPlaceClient();
  ~PickPlaceClient();

  void stateMachines();

  void move_stock(const action_msg::MotionGoal &, const int &);

  void markerRecog();

  void objectRecog();

  void placementRecog();

  void doneMoveStockCb(const actionlib::SimpleClientGoalState&,
                        const action_msg::MotionResultConstPtr&);

  void doneMoveDisplayCb(const actionlib::SimpleClientGoalState&,
                        const action_msg::MotionResultConstPtr&);

  void donePickCb(const actionlib::SimpleClientGoalState&,
                        const action_msg::MotionResultConstPtr&);

  void donePlaceCb(const actionlib::SimpleClientGoalState&,
                        const action_msg::MotionResultConstPtr&);

  void doneMarkerCb(const actionlib::SimpleClientGoalState&,
                        const action_msg::MarkerResultConstPtr&);

  void doneObjectCb(const actionlib::SimpleClientGoalState&,
                        const action_msg::ObjectResultConstPtr&);

  void donePlacementCb(const actionlib::SimpleClientGoalState&,
                        const action_msg::PlacementResultConstPtr&);

  void createGoal(action_msg::MotionGoal &, const std::string &,
                        const int &, const bool &, const bool &);

private:
  MotionClient mc_;
  MarkerClient mac_;
  ObjectClient oc_;
  PlacementClient pc_;
};

#endif

