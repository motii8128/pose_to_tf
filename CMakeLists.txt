cmake_minimum_required(VERSION 3.5)
project(pose_to_tf)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(target_pose_to_tf SHARED
  DIRECTORY src
)

target_include_directories(target_pose_to_tf
  SYSTEM PUBLIC
)

rclcpp_components_register_node(target_pose_to_tf
  PLUGIN "pose_to_tf::PoseTF"
  EXECUTABLE pose_to_tf
)

ament_auto_package(
  INSTALL_TO_SHARE
)