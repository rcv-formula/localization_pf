cmake_minimum_required(VERSION 3.16)
project(localization_pf)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(vesc_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# range_libc is pulled in as a git submodule under third_party/.
# Run `git submodule update --init --recursive` after cloning.
set(RANGE_LIBC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party/range_libc")
if(NOT EXISTS "${RANGE_LIBC_DIR}/includes/RangeLib.h")
  message(FATAL_ERROR
    "range_libc submodule not found at ${RANGE_LIBC_DIR}.\n"
    "Run: git submodule update --init --recursive")
endif()

add_executable(localization_pf_node
  src/localization_pf.cpp
  src/scan_scoring.cpp
  src/particle_propagation.cpp
  src/particle_filter.cpp
  src/observability_map.cpp
  src/pose_estimation.cpp
  src/scan_deskew.cpp
  src/relocalization.cpp
  "${RANGE_LIBC_DIR}/vendor/lodepng/lodepng.cpp")

target_include_directories(localization_pf_node PRIVATE
  include
  ${EIGEN3_INCLUDE_DIRS}
  "${RANGE_LIBC_DIR}")

target_compile_definitions(localization_pf_node PRIVATE
  USE_CUDA=0
  USE_VULKAN=0
  ROS_WORLD_TO_GRID_CONVERSION=1)

ament_target_dependencies(localization_pf_node
  geometry_msgs
  nav_msgs
  rclcpp
  sensor_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  vesc_msgs
  visualization_msgs)

install(TARGETS
  localization_pf_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  config
  launch
  map
  DESTINATION share/${PROJECT_NAME})

ament_package()
