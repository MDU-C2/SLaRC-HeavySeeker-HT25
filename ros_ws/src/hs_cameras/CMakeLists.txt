cmake_minimum_required(VERSION 3.8)
project(hs_cameras)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Make Python code importable as "hs_cameras"
ament_python_install_package(src)

# Install launch and config directories
install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
