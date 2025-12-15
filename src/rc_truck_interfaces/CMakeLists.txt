cmake_minimum_required(VERSION 3.8)
project(rc_truck_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

set(msg_files
  "msg/ThrottleCommand.msg"
  "msg/SteeringCommand.msg"
  "msg/VehicleState.msg"
  "msg/BehaviorState.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
