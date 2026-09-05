cmake_minimum_required(VERSION 3.8)
project(fma_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DriveCommand.msg"
  "msg/Lane.msg"
  "msg/StopLine.msg"
  "msg/TrafficLight.msg"
  "msg/Obstacle.msg"
  "msg/ObstacleArray.msg"
  "msg/ParkingInfo.msg"
  "msg/LaneSignal.msg"
  "msg/MissionState.msg"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
