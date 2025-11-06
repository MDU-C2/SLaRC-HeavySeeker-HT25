cmake_minimum_required(VERSION 3.8)
project(hs_cameras)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# --------------------------------------------------------------------
# Generate StartCamera.action
# --------------------------------------------------------------------
rosidl_generate_interfaces(${PROJECT_NAME}
  "fpv_server/action/StartCamera.action"
  DEPENDENCIES builtin_interfaces
)

# --------------------------------------------------------------------
# Install Python subpackages
# --------------------------------------------------------------------
ament_python_install_package(cameras)
ament_python_install_package(fpv_server)

# --------------------------------------------------------------------
# Install executables (ROS 2 nodes)
# --------------------------------------------------------------------
install(PROGRAMS
  fpv_server/fpv_server.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME fpv_server
)

install(PROGRAMS
  fpv_server/fpv_client.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME fpv_client
)

# --------------------------------------------------------------------
# Install launch + config
# --------------------------------------------------------------------
install(
  DIRECTORY launch cameras/config
  DESTINATION share/${PROJECT_NAME}
)

# --------------------------------------------------------------------
# Export dependencies and finalize
# --------------------------------------------------------------------
ament_export_dependencies(rosidl_default_runtime)
ament_package()
