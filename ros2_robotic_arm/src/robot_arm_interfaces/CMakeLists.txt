cmake_minimum_required(VERSION 3.8)
project(robot_arm_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(trajectory_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ExecuteTrajectory.srv"
  DEPENDENCIES trajectory_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()


