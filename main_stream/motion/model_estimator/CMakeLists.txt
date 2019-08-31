cmake_minimum_required(VERSION 2.8.3)
project(model_estimator)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  core_msgs
  message_generation
  tf2
  tf2_ros
)

catkin_package(CATKIN_DEPENDS
  INCLUDE_DIRS include
)
set (SOURCES
  ${CMAKE_CURRENT_SOURCE_DIR}/src/Model.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/ModelHandler.cpp
)

set (HEADERS
  ${CMAKE_CURRENT_SOURCE_DIR}/include/Model.h
  ${CMAKE_CURRENT_SOURCE_DIR}/include/ModelHandler.h
)


include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(include ${Eigen3_INCLUDE_DIRS})
include_directories(include include)

find_package(OMPL REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(model_estimator src/model_estimator.cpp ${HEADERS} ${SOURCES})
target_link_libraries(model_estimator ${catkin_LIBRARIES} ${Eigen3_LIBS})
