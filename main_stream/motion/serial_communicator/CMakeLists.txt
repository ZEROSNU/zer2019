cmake_minimum_required(VERSION 2.8.3)
project(serial_communicator)
set(CMAKE_CXX_FLAGS "-std=c++11")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  core_msgs
)
find_package(async_comm REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs core_msgs
  DEPENDS async_comm
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${async_comm_INCLUDE_DIRS}
)

add_executable(serial_communicator_manual src/serial_communicator_manual.cpp)
target_link_libraries(serial_communicator_manual ${catkin_LIBRARIES})
add_dependencies(serial_communicator_manual serial_communicator_generate_messages_cpp)

target_link_libraries(serial_communicator_manual ${async_comm_LIBRARIES})