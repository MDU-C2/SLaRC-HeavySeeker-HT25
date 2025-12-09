cmake_minimum_required(VERSION 3.8)
project(hs_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(hs_msgs REQUIRED)

include_directories(include)

set(CAN_PATH include/hs_robot/can_bus.cpp include/hs_robot/can_bus.hpp)

add_executable(health_check src/health_check.cpp)
add_executable(robot_node src/robot.cpp)
add_executable(test_node src/test_node.cpp ${CAN_PATH})

ament_target_dependencies(health_check rclcpp std_msgs hs_msgs)
ament_target_dependencies(robot_node rclcpp std_msgs geometry_msgs hs_msgs)
ament_target_dependencies(test_node rclcpp)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS 
  health_check
  robot_node
  test_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
