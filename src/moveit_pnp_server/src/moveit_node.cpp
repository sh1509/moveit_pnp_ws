#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>

#include <actionlib/server/simple_action_server.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <action_msg/MotionAction.h>
#include <geometry_msgs/Pose.h>
#include<moveit_msgs/MoveItErrorCodes.h>

#include <xmlrpcpp/XmlRpcValue.h>
#include <string>
#include "moveit_node.h"


MotionServer::MotionServer(const std::string &name):
  as_(nh_, name, boost::bind(&MotionServer::executeGoal, this, _1), false)
{
  as_.start();
  ROS_INFO("Action server for Motion Execution has started");
}

MotionServer::~MotionServer(){
}

void MotionServer::executeGoal(const action_msg::MotionGoalConstPtr &goal)
{
  ROS_INFO("Goal for Motion Execution received");
  bool success = true;
  static int count = 0;
  count++;

  // Retreive all the goal parameters
  std::string shelf = goal->shelf;
  int16_t move_id = goal->move_id;
  bool pick = goal->pick;
  bool place = goal->place;

  //Collect the pose from rosparam
  static geometry_msgs::Pose pick_pose;
  static geometry_msgs::Pose place_pose;

  // get the id of the shelf
  std::string id = "id_" + std::to_string(move_id);

  if (count == 1 and shelf == "display")
  {
    ROS_ERROR_STREAM("FIRST ACTION CAN ONLY BE MOVING TO STOCK SHELF");
    success = false;
    result_.done = success;
    as_.setAborted(result_);
  }
  else if (shelf != "display" && shelf != "stock")
  {
    ROS_ERROR_STREAM("Shelf input can only be either 'display' or 'stock'");
    success = false;
    result_.done = success;
    as_.setAborted(result_);
  }

  // MOVE ACTION
  else if (pick == false && place == false)
  {
    // check the maximum limit: current is 3
    if (move_id >= 3 || move_id < 0)
    {
      ROS_ERROR_STREAM("Shelf id doesn't exist");
      success = false;
    }

    else 
    {
      ROS_INFO("Moving to %s shelf", shelf.c_str());
      tf2::Quaternion orientation;
      orientation.setRPY(M_PI*0.444, -M_PI*0.6523 , M_PI*0.194);

      // get stock shelf parameters
      if (shelf == "stock")
      {
        XmlRpc::XmlRpcValue stock_shelf;
        nh_.getParam("/shelves/stock_shelf", stock_shelf);

        // size of the object to be used in pick operation
        std::vector<double> pos(3);
        stock_shelf_param(stock_shelf, pick_pose, id,  pos);

        // add the object to the scene
        moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
        addCollisionObjects(planning_scene_interface, pos, pick_pose);

        // change the orientation with respect to stock shelf
        pick_pose.orientation = tf2::toMsg(orientation);
      }

      // get display shelf parameters
      else 
      {
        XmlRpc::XmlRpcValue display_shelf;
        nh_.getParam("/shelves/display_shelf", display_shelf);
        display_shelf_param(display_shelf, place_pose, id);

        // change the orientation with respect to display shelf
        tf2::Quaternion orientation;
        orientation.setRPY(M_PI*0.444, -M_PI*0.6523 , M_PI*2.946);
        place_pose.orientation = tf2::toMsg(orientation);

        // hyperparameter to pre-place position
        place_pose.position.y -= 0.1;
      }

      // Execute the move action
      if (as_.isPreemptRequested() || !ros::ok())
      {
        ROS_INFO("Action Preempted");
        // set the action state to preempted
        as_.setPreempted();
        success = false;
      }
      else 
      {
        moveit::planning_interface::MoveItErrorCode plan_result = -1;
        moveit::planning_interface::MoveItErrorCode move_result = -1;
        if (shelf == "stock")
          MotionServer::move(pick_pose, plan_result, move_result);
        else
          MotionServer::move(place_pose, plan_result, move_result);
        // result can be either from plan or execution so take care of that.
        if (plan_result && move_result)
          success = true;
        else
        {
          success = false;
          result_.done = success;
          ROS_ERROR("Either Planning or Execution FAILED");
          ROS_ERROR_STREAM("Planning returned with exit code: " << plan_result);
          ROS_ERROR_STREAM("Execution returned with exit code: " << move_result);
          as_.setAborted(result_);
        }
      }
    }
  }
 
  // PICK Operation
  else if (move_id == -1 && pick == true && place == false && shelf == "stock")
  {
    // pick the object
    ROS_INFO("PICKING the Object");
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;;
    moveit::planning_interface::MoveGroupInterface group("panda_arm");
    group.setPlanningTime(20.0);

    if (as_.isPreemptRequested() || !ros::ok())
    {
        ROS_INFO("Action Preempted");
        // set the action state to preempted
        as_.setPreempted();
        success = false;
    }
    else
    {
      moveit::planning_interface::MoveItErrorCode pick_result = -1;
      pick_object(group, pick_pose, pick_result);
      if (pick_result)
          success = true;
      else
      {
        success = false;
        result_.done = success;
        ROS_ERROR("PICK OPERATION FAILED");
        ROS_ERROR_STREAM("PICK returned with exit code: " << pick_result);
        as_.setAborted(result_);
      }
    }
  }

  // PLACE Operation
  else if (move_id == -1 && pick == false && place == true && shelf == "display")
  {
    // place the object
    ROS_INFO("PLACING the Object");
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit::planning_interface::MoveGroupInterface group("panda_arm");
    group.setPlanningTime(20.0);

    if (as_.isPreemptRequested() || !ros::ok())
    {
        ROS_INFO("Action Preempted");
        // set the action state to preempted
        as_.setPreempted();
        success = false;
    }
    else
    {
      moveit::planning_interface::MoveItErrorCode place_result = -1;
      place_object(group, place_pose, place_result);
      if (place_result)
          success = true;
      else
      {
        success = false;
        result_.done = success;
        ROS_ERROR("PLACE OPERATION");
        ROS_ERROR_STREAM("PLACE returned with exit code: " << place_result);
        as_.setAborted(result_);
      }
    }
  }

  else 
  {
    success = false;
    result_.done = success;
    // It can be the case when the action is place but shelf option is stock
    ROS_ERROR_STREAM("Only one action: Move or Pick or Place || Provide correct logical action");
    as_.setAborted(result_);
  }

  // // Random feedback for the dummy server
  // geometry_msgs::Pose feedback_pose;
  // feedback_.current_pose = feedback_pose;
  // // publish the feedback
  // as_.publishFeedback(feedback_);

  if(success)
  {
    // result_.final_pose = feedback_.current_pose;
    result_.done = success;
    ROS_INFO("Action Succeeded");

    // set the action state to succeeded
    as_.setSucceeded(result_);
  }
}

void MotionServer::stock_shelf_param(const XmlRpc::XmlRpcValue &stock_shelf, geometry_msgs::Pose &Pose,
                                                         const std::string &id, std::vector<double> &pos)
{
  if(stock_shelf.getType() == XmlRpc::XmlRpcValue::Type::TypeStruct && stock_shelf.size() > 0)
  {
    Pose.orientation.x = stock_shelf[id]["orientation"]["x"];
    Pose.orientation.y = stock_shelf[id]["orientation"]["y"];
    Pose.orientation.z = stock_shelf[id]["orientation"]["z"];
    Pose.orientation.w = stock_shelf[id]["orientation"]["w"];

    Pose.position.x = stock_shelf[id]["position"]["x"];
    Pose.position.y = stock_shelf[id]["position"]["y"];
    Pose.position.z = stock_shelf[id]["position"]["z"];

    pos[0] = stock_shelf[id]["size"]["x"];
    pos[1] = stock_shelf[id]["size"]["y"];
    pos[2] = stock_shelf[id]["size"]["z"];
  }
  else 
  {
    ROS_ERROR_STREAM("XmlRpc type error");
  }
}

void MotionServer::display_shelf_param(const XmlRpc::XmlRpcValue &display_shelf, geometry_msgs::Pose &Pose,
                                                                                       const std::string &id)
{
  if(display_shelf.getType() == XmlRpc::XmlRpcValue::Type::TypeStruct && display_shelf.size() > 0)
  {
    Pose.orientation.x = display_shelf[id]["orientation"]["x"];
    Pose.orientation.y = display_shelf[id]["orientation"]["y"];
    Pose.orientation.z = display_shelf[id]["orientation"]["z"];
    Pose.orientation.w = display_shelf[id]["orientation"]["w"];

    Pose.position.x = display_shelf[id]["position"]["x"];
    Pose.position.y = display_shelf[id]["position"]["y"];
    Pose.position.z = display_shelf[id]["position"]["z"];
  }
  else 
  {
    ROS_ERROR_STREAM("XmlRpc type error");
  }
}

void MotionServer::move(const geometry_msgs::Pose & target_pose1, moveit::planning_interface::MoveItErrorCode &result,
                                                                    moveit::planning_interface::MoveItErrorCode &move_result)
{
  static const std::string PLANNING_GROUP = "panda_arm";
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  const robot_state::JointModelGroup* joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  move_group.setPoseTarget(target_pose1);
  move_group.setGoalTolerance(0.2);

  // Plan to the target pose
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  ROS_INFO("Planning the motion of the robot");
  result = move_group.plan(my_plan);
  ROS_INFO_COND(result == 1, "Plan SUCCEEDED");
  ROS_ERROR_COND(result != 1, "Plan FAILED");

  // move the robot
  ROS_INFO("Executing the motion of the robot");
  move_result = move_group.move();
  ROS_INFO_COND(move_result == 1, "Execute SUCCEEDED");
  ROS_ERROR_COND(move_result != 1, "Execute FAILED");
}

void MotionServer::addCollisionObjects(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface, 
                                          const std::vector<double> &pos, const geometry_msgs::Pose &Pose)
{
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.resize(1);

  collision_objects[0].header.frame_id = "panda_link0";
  collision_objects[0].id = "object";

  /* Define the primitive and its dimensions. */
  collision_objects[0].primitives.resize(1);
  collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
  collision_objects[0].primitives[0].dimensions.resize(3);
  collision_objects[0].primitives[0].dimensions[0] = pos[0];
  collision_objects[0].primitives[0].dimensions[1] = pos[1];
  collision_objects[0].primitives[0].dimensions[2] = pos[2] - 0.2;

  /* Define the pose of the object. */
  collision_objects[0].primitive_poses.resize(1);
  collision_objects[0].primitive_poses[0].position.x = Pose.position.x;
  collision_objects[0].primitive_poses[0].position.y = Pose.position.y - 0.1;
  collision_objects[0].primitive_poses[0].position.z = Pose.position.z + 0.12;

  tf2::Quaternion orientation;
  orientation.setRPY(-M_PI, M_PI/2, -M_PI /2);
  collision_objects[0].primitive_poses[0].orientation = tf2::toMsg(orientation);
  collision_objects[0].operation = collision_objects[0].ADD;
  planning_scene_interface.applyCollisionObjects(collision_objects);
}

void MotionServer::pick_object(moveit::planning_interface::MoveGroupInterface& move_group,
                                                    const geometry_msgs::Pose &target_pose,
                                                    moveit::planning_interface::MoveItErrorCode &result)
{
  std::vector<moveit_msgs::Grasp> grasps;
  grasps.resize(1);
  grasps[0].grasp_pose.header.frame_id = "panda_link0";

  // Calculated manually using transform concepts
  tf2::Quaternion orientation;
  orientation.setRPY(M_PI*0.444, -M_PI*0.6523 , M_PI*0.194);
  grasps[0].grasp_pose.pose.orientation = tf2::toMsg(orientation);

  grasps[0].grasp_pose.pose.position.x = target_pose.position.x;
  grasps[0].grasp_pose.pose.position.y = target_pose.position.y + 0.13;
  grasps[0].grasp_pose.pose.position.z = target_pose.position.z + 0.07;

  // Setting pre-grasp approach
  // ++++++++++++++++++++++++++
  /* Defined with respect to frame_id */
  grasps[0].pre_grasp_approach.direction.header.frame_id = "panda_link0";
  /* Direction is set as negative y axis */
  grasps[0].pre_grasp_approach.direction.vector.y = -1.0;
  grasps[0].pre_grasp_approach.min_distance = 0.05;
  grasps[0].pre_grasp_approach.desired_distance = 0.06;

  // Setting post-grasp retreat
  // ++++++++++++++++++++++++++
  /* Defined with respect to frame_id */
  grasps[0].post_grasp_retreat.direction.header.frame_id = "panda_link0";
   /* Direction is set as positive y axis */
  grasps[0].post_grasp_retreat.direction.vector.y = 1.0;
  grasps[0].post_grasp_retreat.min_distance = 0.2;
  grasps[0].post_grasp_retreat.desired_distance = 0.25;

  // Setting posture of eef before grasp
  openGripper(grasps[0].pre_grasp_posture);
  closedGripper(grasps[0].grasp_posture);
  move_group.setSupportSurfaceName("stock_shelf");
  result = move_group.pick("object", grasps);
}

void MotionServer::openGripper(trajectory_msgs::JointTrajectory& posture)
{
  /* Add both finger joints of panda robot. */
  posture.joint_names.resize(2);
  posture.joint_names[0] = "panda_finger_joint1";
  posture.joint_names[1] = "panda_finger_joint2";

  /* Set them as open, wide enough for the object to fit. */
  posture.points.resize(1);
  posture.points[0].positions.resize(2);
  posture.points[0].positions[0] = 0.04;
  posture.points[0].positions[1] = 0.04;
  posture.points[0].time_from_start = ros::Duration(0.5);
}

void MotionServer::closedGripper(trajectory_msgs::JointTrajectory& posture)
{
  /* Add both finger joints of panda robot. */
  posture.joint_names.resize(2);
  posture.joint_names[0] = "panda_finger_joint1";
  posture.joint_names[1] = "panda_finger_joint2";

  /* Set them as closed. */
  posture.points.resize(1);
  posture.points[0].positions.resize(2);
  posture.points[0].positions[0] = 0.00;
  posture.points[0].positions[1] = 0.00;
  posture.points[0].time_from_start = ros::Duration(0.5);
}

void MotionServer::place_object(moveit::planning_interface::MoveGroupInterface& group,
                                                  const geometry_msgs::Pose &target_pose,
                                                  moveit::planning_interface::MoveItErrorCode &result)
{
  std::vector<moveit_msgs::PlaceLocation> place_location;
  place_location.resize(1);

  // Setting place location pose
  // +++++++++++++++++++++++++++
  place_location[0].place_pose.header.frame_id = "panda_link0";
  tf2::Quaternion orientation;
  // Calculated manually using transform concepts
  orientation.setRPY(-M_PI*0.444, -M_PI*0.6523 , M_PI*2.946);
  place_location[0].place_pose.pose.orientation = tf2::toMsg(orientation);

  /* Making the place location slightly above and
  little inside the shelf as this pose collected from the config is the location of the id. */
  place_location[0].place_pose.pose.position.x = target_pose.position.x;
  place_location[0].place_pose.pose.position.y = target_pose.position.y + 0.2;
  place_location[0].place_pose.pose.position.z = target_pose.position.z + 0.15;

  // Setting pre-place approach
  // ++++++++++++++++++++++++++
  /* Defined with respect to frame_id */
  place_location[0].pre_place_approach.direction.header.frame_id = "panda_link0";
  /* Direction is set as positive y axis */
  place_location[0].pre_place_approach.direction.vector.y = 1.0;
  place_location[0].pre_place_approach.min_distance = 0.095;
  place_location[0].pre_place_approach.desired_distance = 0.115;

  // Setting post-grasp retreat
  // ++++++++++++++++++++++++++
  /* Defined with respect to frame_id */
  place_location[0].post_place_retreat.direction.header.frame_id = "panda_link0";
  /* Direction is set as negative y axis */
  place_location[0].post_place_retreat.direction.vector.y = -1.0;
  place_location[0].post_place_retreat.min_distance = 0.1;
  place_location[0].post_place_retreat.desired_distance = 0.2;

  // Setting posture of eef after placing object
  openGripper(place_location[0].post_place_posture);

  group.setSupportSurfaceName("display_shelf");
  result = group.place("object", place_location);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "moveit_pnp_server");
  MotionServer motionServer("move");
  ros::spin();
  return 0;
}