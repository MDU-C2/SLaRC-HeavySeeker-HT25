cmake_minimum_required(VERSION 3.8)
project(s_cameras)

# --------------------------------------------------------------------
# Dependencies
# --------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# --------------------------------------------------------------------
# Install Python packages
# --------------------------------------------------------------------

ament_python_install_package(camera_server)
ament_python_install_package(fpv_client)
ament_python_install_package(init_cameras)

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

# --------------------------------------------------------------------
# Install Launch & Config
# --------------------------------------------------------------------
install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# --------------------------------------------------------------------
ament_package()
