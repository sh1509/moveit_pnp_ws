cmake_minimum_required(VERSION 3.0.2)
project(pick_place_client)
add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  action_msg
  actionlib
  actionlib_msgs
  message_generation
  message_runtime
  moveit_core
  moveit_ros_planning_interface
  moveit_visual_tools
  roscpp
  std_msgs
  geometry_msgs
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED system)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES pick_place_client
 CATKIN_DEPENDS actionlib actionlib_msgs roscpp std_msgs geometry_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/pick_place_client_node.cpp)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)
