cmake_minimum_required(VERSION 3.20)
project(world_explorer)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_executable(world_explorer_node
  src/world_explorer_node.cpp
)

ament_target_dependencies(world_explorer_node
  rclcpp
  underground_world
)

ament_auto_add_executable(payload_action_node
  src/payload_action_node.cpp
)

ament_target_dependencies(payload_action_node
  rclcpp
  underground_world
)

ament_auto_package(
  USE_SCOPED_HEADER_INSTALL_DIR
  INSTALL_TO_SHARE
  launch
)