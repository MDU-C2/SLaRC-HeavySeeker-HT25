cmake_minimum_required(VERSION 3.8)
project(hs_cameras)

# --------------------------------------------------------------------
# Dependencies
# --------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# --------------------------------------------------------------------
# Generate Actions and Services
# --------------------------------------------------------------------
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/StartCamera.action"
  "action/StopCamera.action"
  "srv/GetCameras.srv"
  DEPENDENCIES builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)

# --------------------------------------------------------------------
# Install Python packages
# --------------------------------------------------------------------
ament_python_install_package(fpv_server)
ament_python_install_package(cameras)

# --------------------------------------------------------------------
# Install Executables (ROS 2 Nodes)
# --------------------------------------------------------------------
install(PROGRAMS
  fpv_server/fpv_server.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME fpv_server
)

install(PROGRAMS
  fpv_client/fpv_client.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME fpv_client
)

install(PROGRAMS
  fpv_client/decoder.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME decoder
)

# --------------------------------------------------------------------
#  Install Launch and Config Directories
# --------------------------------------------------------------------
install(
  DIRECTORY launch cameras/config
  DESTINATION share/${PROJECT_NAME}
)

# --------------------------------------------------------------------
# Make sure messages are generated before Python code uses them
# --------------------------------------------------------------------
ament_export_dependencies(rosidl_default_runtime)
ament_package()
