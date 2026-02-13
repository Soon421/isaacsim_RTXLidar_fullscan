cmake_minimum_required(VERSION 3.8)

project(fullscan)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Require C++17
set(CMAKE_CXX_STANDARD 17)

# Find ament and ROS2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(
  include
)

# Set the variable 'DEPENDENCIES'
set(DEPENDENCIES
  rclcpp
  sensor_msgs
  std_msgs
)

# Executable for fullscan
add_executable(fullscan_node src/fullscan_node.cpp)

ament_target_dependencies(fullscan_node
  ${DEPENDENCIES}
)

install(TARGETS 
  fullscan_node
  RUNTIME
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY 
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()