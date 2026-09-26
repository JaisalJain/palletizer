cmake_minimum_required(VERSION 3.8)
project(palletizer_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(action_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/BoxSpec.msg
  msg/DetectedBox.msg
  msg/Placement.msg
  msg/CycleReport.msg
  msg/PalletReport.msg
  msg/DetectedPallet.msg
  msg/HeightMap.msg
  msg/PlacementCheck.msg
  msg/StackEvent.msg
  msg/PackingState.msg
  msg/PalletStatus.msg
  srv/DetectBox.srv
  srv/PlanPlacement.srv
  srv/CommitPlacement.srv
  srv/Weigh.srv
  srv/MeasurePallet.srv
  srv/SetPallet.srv
  srv/VerifyPlacement.srv
  srv/CheckPlacements.srv
  action/PickBox.action
  action/PlaceBox.action
  action/PalletExchange.action
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs action_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
