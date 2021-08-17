cmake_minimum_required(VERSION 3.0.2)
project(action_msg)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  geometry_msgs
  message_generation
  message_runtime
  roscpp
  std_msgs
)

add_action_files(
  FILES
  Motion.action
  Marker.action
  Object.action
  Placement.action
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  geometry_msgs
  std_msgs
)


catkin_package(

 CATKIN_DEPENDS actionlib actionlib_msgs geometry_msgs roscpp std_msgs roscpp
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

