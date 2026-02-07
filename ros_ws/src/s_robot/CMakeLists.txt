cmake_minimum_required(VERSION 3.8)
project(s_robot LANGUAGES CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ros2_control_cmake REQUIRED)
set_compiler_options()
export_windows_symbols()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

find_package(backward_ros REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(pluginlib REQUIRED)
find_package(hardware_interface REQUIRED)

find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(s_msgs REQUIRED)

# Specify the required version of ros2_control
find_package(controller_manager 4.0.0)
# Handle the case where the required version is not found
if(NOT controller_manager_FOUND)
  message(FATAL_ERROR "ros2_control version 4.0.0 or higher is required. "
  "Are you using the correct branch of the ros2_control_demos repository?")
endif()

## COMPILE
add_library(
  motor_interface
  SHARED
  src/motor_interface.cpp
  src/can_bus.cpp
)
target_compile_features(motor_interface PUBLIC cxx_std_17)
target_include_directories(motor_interface PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>
)
target_link_libraries(motor_interface PUBLIC
  hardware_interface::hardware_interface
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
)

include_directories(include)

set(CAN_PATH include/s_robot/can_bus.hpp)
set(STATUS_INFO include/s_robot/status_info.hpp)

add_executable(health_check src/health_check.cpp ${STATUS_INFO})
#add_executable(robot_node src/robot.cpp)
add_executable(test_node src/test_node.cpp ${CAN_PATH} src/can_bus.cpp)

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface motor_interface.xml)

# INSTALL
install(
  DIRECTORY include/s_robot
  DESTINATION include
)
install(TARGETS motor_interface
  EXPORT export_motor_interface
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY
  config
  launch
  description
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS 
  health_check
  #robot_node
  test_node
  DESTINATION lib/${PROJECT_NAME}
)




ament_target_dependencies(health_check rclcpp std_msgs s_msgs sensor_msgs)
#ament_target_dependencies(robot_node rclcpp std_msgs geometry_msgs s_msgs)
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




ament_export_targets(export_motor_interface HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
