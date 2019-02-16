cmake_minimum_required(VERSION 2.8.3)
project(core_msgs)

find_package(catkin REQUIRED
  message_generation
  geometry_msgs
  sensor_msgs
  std_msgs
)

add_message_files( DIRECTORY msg
  FILES
  ROIPointArray.msg
  VehicleState.msg
  PathArray.msg
  CenPoint.msg
  Control.msg
  Estop.msg
  ParkPoints.msg
)

add_service_files(
  FILES
  ToggleMission.srv
)

generate_messages(DEPENDENCIES   geometry_msgs  sensor_msgs   std_msgs )

catkin_package(CATKIN_DEPENDS message_runtime geometry_msgs sensor_msgs std_msgs)
