cmake_minimum_required(VERSION 3.5)
project(multi_drone_msg)

# ROS 2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)

# Custom message files
set(msg_files
  "msg/CoordinateDataMsg.msg"
  "msg/LocalAndGlobalCoordinatesMsg.msg"
  "msg/DroneParamsMsg.msg"
  "msg/DroneInformMsg.msg"
)

# Generate interfaces (messages)
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES geometry_msgs
)

# Export runtime dependencies
ament_export_dependencies(rosidl_default_runtime)

# Package declaration
ament_package()
