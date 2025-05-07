# cmake_minimum_required(VERSION 3.8)
# project(safety_shield_node)

# # --- C++ Standard ---
# set(CMAKE_CXX_STANDARD 17)
# set(CMAKE_CXX_STANDARD_REQUIRED ON)

# # --- Find Dependencies ---
# find_package(ament_cmake REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(sensor_msgs REQUIRED)
# find_package(geometry_msgs REQUIRED)
# find_package(Eigen3 REQUIRED NO_MODULE)
# find_package(Threads REQUIRED)
# find_package(fmt REQUIRED)                             # link fmt library for formatting support
# find_package(safety_shield CONFIG REQUIRED)

# # --- Compute install prefix and include dirs ---
# # safety_shield_DIR points to <prefix>/lib/cmake/safety_shield
# get_filename_component(_CMAKE_DIR "${safety_shield_DIR}" PATH)   # .../lib/cmake
# get_filename_component(_LIB_DIR   "${_CMAKE_DIR}"     PATH)   # .../lib
# get_filename_component(_PREFIX    "${_LIB_DIR}"       PATH)   # install prefix

# set(SHIELD_INCLUDE_DIR    "${_PREFIX}/include")
# set(LTP_INCLUDE_DIR       "${_PREFIX}/include/long_term_planner")
# set(SARA_INCLUDE_DIR      "${_PREFIX}/include/SaRA")

# # --- Executable ---
# add_executable(${PROJECT_NAME}
#   src/safety_shield_node.cpp
# )

# # --- Include directories ---
# target_include_directories(${PROJECT_NAME} PRIVATE
#   ${SHIELD_INCLUDE_DIR}
#   ${LTP_INCLUDE_DIR}
#   ${SARA_INCLUDE_DIR}
# )

# # --- Link Libraries and Dependencies ---
# ament_target_dependencies(${PROJECT_NAME}
#   rclcpp
#   sensor_msgs
#   geometry_msgs
# )

# target_link_libraries(${PROJECT_NAME}
#   safety_shield::safety_shield
#   Eigen3::Eigen
#   Threads::Threads
#   fmt::fmt                                         # link fmt explicitly to satisfy fmt symbols
# )

# # --- Install ---
# install(TARGETS ${PROJECT_NAME}
#   DESTINATION lib/${PROJECT_NAME}
# )

# ament_package()

cmake_minimum_required(VERSION 3.8)
project(safety_shield_node)

# --- C++ Standard ---
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# --- Find Dependencies ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(Threads REQUIRED)
find_package(fmt REQUIRED)                             # For formatting support
find_package(safety_shield CONFIG REQUIRED)
find_package(ament_index_cpp REQUIRED)

# --- Compute install prefix and include dirs ---
# safety_shield_DIR points to <prefix>/lib/cmake/safety_shield
get_filename_component(_CMAKE_DIR "${safety_shield_DIR}" PATH)
get_filename_component(_LIB_DIR   "${_CMAKE_DIR}"     PATH)
get_filename_component(_PREFIX    "${_LIB_DIR}"       PATH)

set(SHIELD_INCLUDE_DIR    "${_PREFIX}/include")
set(LTP_INCLUDE_DIR       "${_PREFIX}/include/long_term_planner")
set(SARA_INCLUDE_DIR      "${_PREFIX}/include/SaRA")

# --- Executable ---
add_executable(${PROJECT_NAME}
  src/safety_shield_node.cpp
)

# --- Include directories ---
target_include_directories(${PROJECT_NAME} PRIVATE
  ${SHIELD_INCLUDE_DIR}
  ${LTP_INCLUDE_DIR}
  ${SARA_INCLUDE_DIR}
)

# --- Link Libraries and Dependencies ---
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  sensor_msgs
  visualization_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  ament_index_cpp
)

target_link_libraries(${PROJECT_NAME}
  safety_shield::safety_shield
  Eigen3::Eigen
  Threads::Threads
  fmt::fmt
)

# --- Install executable ---
install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

# --- Install launch files ---
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# --- Install configuration files ---
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY rviz/
  DESTINATION share/${PROJECT_NAME}/rviz
)

ament_package()