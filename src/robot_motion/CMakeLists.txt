cmake_minimum_required(VERSION 3.8)
project(robot_motion)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(custom_interfaces REQUIRED)

add_executable(motion_controller_node src/motion_controller_node.cpp)
ament_target_dependencies(motion_controller_node rclcpp custom_interfaces)

install(TARGETS
motion_controller_node
DESTINATION lib/${PROJECT_NAME})



ament_package()
