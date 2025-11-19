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

ament_python_install_package(camera_server)
ament_python_install_package(fpv_client)
ament_python_install_package(init_cameras)
ament_python_install_package(utils)

# --------------------------------------------------------------------
# Install Executables (entry points)
# --------------------------------------------------------------------

install(PROGRAMS
  camera_server/server_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME server_node
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


install(
  FILES fpv_client/decoder_cfg.dat
  DESTINATION lib/${PROJECT_NAME}
)

# --------------------------------------------------------------------
# Install Launch & Config
# --------------------------------------------------------------------
install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# --------------------------------------------------------------------
ament_package()
