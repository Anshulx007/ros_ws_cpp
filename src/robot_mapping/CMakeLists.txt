cmake_minimum_required(VERSION 3.8)
project(robot_mapping)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++17)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)

# Add executable
add_executable(auto_explore src/auto_explore.cpp)
target_compile_features(auto_explore PRIVATE cxx_std_17)
ament_target_dependencies(auto_explore
  rclcpp
  rclcpp_action
  nav2_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
  tf2_ros
)

add_executable(wall_explorer src/wall_explorer.cpp)
target_compile_features(wall_explorer PRIVATE cxx_std_17)
ament_target_dependencies(wall_explorer
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
)

# Install executable
install(TARGETS
  auto_explore
  wall_explorer
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
