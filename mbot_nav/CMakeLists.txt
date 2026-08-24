cmake_minimum_required(VERSION 3.22)
project(mbot_nav)

# C++17 required (ROS2 Jazzy / Ubuntu 24)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(mbot_interfaces REQUIRED)

include_directories(include)

set(DEPS rclcpp nav_msgs geometry_msgs tf2 tf2_ros tf2_geometry_msgs mbot_interfaces)

# navigation_node: path planning
add_executable(navigation_node
  src/obstacle_distance_grid.cpp
  src/planners/astar_planner.cpp
  src/planners/theta_star_planner.cpp
  src/planners/example_planner.cpp
  src/navigation_node.cpp
)
ament_target_dependencies(navigation_node ${DEPS})

# exploration_node: frontier detection, sends goals to navigation_node
add_executable(exploration_node
  src/obstacle_distance_grid.cpp
  src/frontier_explorer.cpp
  src/exploration_node.cpp
)
ament_target_dependencies(exploration_node ${DEPS})

# controller_node: add new controller here
add_executable(controller_node
  src/controllers/rtr_controller.cpp
  src/controllers/pure_pursuit_controller.cpp
  src/controllers/example_controller.cpp
  src/controller_node.cpp
)
ament_target_dependencies(controller_node ${DEPS})

# square_publisher: hardcoded waypoints for testing the controller
add_executable(square_publisher
  src/square_publisher.cpp
)
ament_target_dependencies(square_publisher rclcpp geometry_msgs mbot_interfaces)

install(TARGETS navigation_node exploration_node controller_node square_publisher
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY rviz/   DESTINATION share/${PROJECT_NAME}/rviz)
install(DIRECTORY maps/   DESTINATION share/${PROJECT_NAME}/maps)

ament_package()
