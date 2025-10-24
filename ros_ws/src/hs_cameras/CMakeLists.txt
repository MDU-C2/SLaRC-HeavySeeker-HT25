cmake_minimum_required(VERSION 3.8)
project(hs_cameras)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)


ament_python_install_package(src)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
