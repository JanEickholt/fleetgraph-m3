cmake_minimum_required(VERSION 3.8)
project(fleetgraph_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Meta.msg"
  "msg/Location.msg"
  "msg/TaskRequest.msg"
  "msg/TaskAssignment.msg"
  "msg/TaskCommand.msg"
  "msg/TaskUpdate.msg"
  "msg/ErrorInfo.msg"
  "msg/RobotState.msg"
  "msg/RobotConnection.msg"
  "msg/SceneEntity.msg"
  "msg/SceneRelation.msg"
  "msg/SceneObservation.msg"
  "msg/SceneGraphUpdate.msg"
  "srv/GetSceneGraph.srv"
  DEPENDENCIES builtin_interfaces geometry_msgs diagnostic_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
