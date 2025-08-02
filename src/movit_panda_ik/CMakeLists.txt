cmake_minimum_required(VERSION 3.22)
project(movit_panda_ik)

# Required build tools
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

# Add executable
add_executable(cartesian_to_joint_node src/cartesian_to_joint_node.cpp)

# Link dependencies
ament_target_dependencies(cartesian_to_joint_node
  rclcpp
  moveit_ros_planning_interface
)

# Install executable
install(TARGETS cartesian_to_joint_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch and config directories
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
    PATTERN "setup_assistant.launch" EXCLUDE)
endif()

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
#install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})

# Finalize package
ament_package()
