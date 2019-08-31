cmake_minimum_required(VERSION 2.8.3)
project(planner)
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
  ${CMAKE_CURRENT_SOURCE_DIR}/src/carsetupcomhandle.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/DORRT.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/magneticmodel.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/potentialmodel.cpp
)
set (SOURCES_ASTAR
  ${CMAKE_CURRENT_SOURCE_DIR}/src/carsetupcomhandle_astar.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/hybrid_astar.cpp
)

set (HEADERS
  ${CMAKE_CURRENT_SOURCE_DIR}/include/carsetupcomhandle.h
  ${CMAKE_CURRENT_SOURCE_DIR}/include/DORRT.h
  ${CMAKE_CURRENT_SOURCE_DIR}/include/magneticmodel.h
  ${CMAKE_CURRENT_SOURCE_DIR}/include/potentialmodel.h
  ${CMAKE_CURRENT_SOURCE_DIR}/include/hybrid_astar.h
)


include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(include ${OMPL_INCLUDE_DIRS})
include_directories(include ${Eigen3_INCLUDE_DIRS})
include_directories(include include)

find_package(OMPL REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(dorrt src/planner.cpp ${HEADERS} ${SOURCES})
add_executable(astar src/planner_astar.cpp ${HEADERS} ${SOURCES_ASTAR})
target_link_libraries(dorrt ${catkin_LIBRARIES} ${OMPL_LIBRARIES} ${Eigen3_LIBS})
target_link_libraries(astar ${catkin_LIBRARIES} ${OMPL_LIBRARIES} ${Eigen3_LIBS})