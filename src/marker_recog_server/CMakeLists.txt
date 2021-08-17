cmake_minimum_required(VERSION 3.0.2)
project(marker_recog_server)
add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  action_msg
  actionlib
  actionlib_msgs
  message_generation
  roscpp
  std_msgs
)

find_package(Boost REQUIRED COMPONENTS system filesystem date_time thread)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES
 CATKIN_DEPENDS actionlib actionlib_msgs roscpp std_msgs
)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
)

add_executable(${PROJECT_NAME}_node src/marker_recog_server_node.cpp)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)
