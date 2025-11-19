cmake_minimum_required(VERSION 3.8)
project(yolo_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_default_runtime REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Detection.msg"
  "msg/Detections.msg"
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
