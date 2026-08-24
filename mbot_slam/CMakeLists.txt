cmake_minimum_required(VERSION 3.22)
project(mbot_slam)

# C++17 required (ROS2 Jazzy / Ubuntu 24)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(include)

# Shared algorithm sources (particle filter, sensor/action models, mapping, laser scan)
set(COMMON_SRCS
  src/common/action_model.cpp
  src/common/sensor_model.cpp
  src/common/particle_filter.cpp
  src/common/mapping.cpp
  src/common/moving_laser_scan.cpp
)

set(DEPS rclcpp sensor_msgs nav_msgs nav2_msgs geometry_msgs tf2 tf2_ros tf2_geometry_msgs)

# Three nodes: odometry mapping, localization-only, full SLAM
add_executable(mapping_node      ${COMMON_SRCS} src/mapping_node.cpp)
add_executable(localization_node ${COMMON_SRCS} src/localization_node.cpp)
add_executable(slam_node         ${COMMON_SRCS} src/slam_node.cpp)

ament_target_dependencies(mapping_node      ${DEPS})
ament_target_dependencies(localization_node ${DEPS})
ament_target_dependencies(slam_node         ${DEPS})

install(TARGETS mapping_node localization_node slam_node
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY rviz/   DESTINATION share/${PROJECT_NAME}/rviz)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY maps/   DESTINATION share/${PROJECT_NAME}/maps)

ament_package()
