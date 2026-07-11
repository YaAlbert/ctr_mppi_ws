cmake_minimum_required(VERSION 3.8)
project(ctr_interfaces)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CtrBackbone.msg"
  "msg/CtrControllerMetrics.msg"
  "msg/CtrJointCommand.msg"
  "msg/CtrJointState.msg"
  "msg/CtrSafetyStatus.msg"
  "msg/CtrState.msg"
  "msg/CtrTactileState.msg"
  "srv/ClearFault.srv"
  "srv/ExecuteRetreat.srv"
  "srv/ResetController.srv"
  "srv/SetControllerWeights.srv"
  "srv/SetReference.srv"
  "srv/SetTaskMode.srv"
  "srv/StartExperiment.srv"
  "srv/StopExperiment.srv"
  DEPENDENCIES geometry_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
