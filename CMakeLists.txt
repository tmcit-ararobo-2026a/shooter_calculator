cmake_minimum_required(VERSION 3.8)
project(shooter_calculator LANGUAGES C CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find ROS 2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AimTarget.msg" "msg/ShotSolution.msg"
  DEPENDENCIES geometry_msgs std_msgs builtin_interfaces)

option(BUILD_RUNTIME_NODES "Build shooter calculation runtime nodes" ON)
if(BUILD_RUNTIME_NODES)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(include)

# Shooter Calculator Library
add_library(shooter_calculator_lib src/shooter_calculator.cpp)
ament_target_dependencies(shooter_calculator_lib
  rclcpp
  geometry_msgs
)

# Shooter Calculator Node
add_executable(shooter_calculator_node src/shooter_calculator_node.cpp)
ament_target_dependencies(shooter_calculator_node
  rclcpp
  std_msgs
  geometry_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)
target_link_libraries(shooter_calculator_node shooter_calculator_lib)

# Install
install(TARGETS
  shooter_calculator_node
  DESTINATION lib/${PROJECT_NAME}
)

endif()

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
