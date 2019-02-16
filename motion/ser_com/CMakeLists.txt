cmake_minimum_required(VERSION 2.8.3)
project(ser_com_019)
set(CMAKE_CXX_FLAGS "-std=c++11")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  core_msgs_019
)
find_package(async_comm REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs core_msgs_019
  DEPENDS async_comm
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${async_comm_INCLUDE_DIRS}
)

add_executable(ser_com_manual src/ser_com_manual.cpp)
target_link_libraries(ser_com_manual ${catkin_LIBRARIES})
add_dependencies(ser_com_manual ser_com_019_generate_messages_cpp)

target_link_libraries(ser_com_manual ${async_comm_LIBRARIES})