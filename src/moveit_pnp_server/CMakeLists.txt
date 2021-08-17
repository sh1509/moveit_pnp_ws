cmake_minimum_required(VERSION 3.0.2)
project(moveit_pnp_server)
add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  action_msg
  actionlib
  actionlib_msgs
  roscpp
  std_msgs
  geometry_msgs
  message_generation
  interactive_markers
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_ros_perception
  rviz_visual_tools
  moveit_visual_tools
  pluginlib
  geometric_shapes
  pcl_ros
  pcl_conversions
  rosbag
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)


catkin_package(
  LIBRARIES
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    actionlib
    actionlib_msgs
    geometry_msgs
    roscpp std_msgs
    message_runtime
    moveit_core
    moveit_visual_tools
    moveit_ros_planning_interface
    interactive_markers
    tf2_geometry_msgs
  DEPENDS
    EIGEN3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIRS}
)

add_subdirectory(app)

add_executable(${PROJECT_NAME}_node src/moveit_node.cpp)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)
