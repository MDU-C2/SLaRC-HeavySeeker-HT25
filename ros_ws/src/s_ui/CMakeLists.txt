cmake_policy(SET CMP0072 NEW)
set(OpenGL_GL_PREFERENCE GLVND)
cmake_minimum_required(VERSION 3.10)
project(s_ui)

find_package(ament_cmake REQUIRED)
find_package(rviz2 REQUIRED)
find_package(foxglove_bridge REQUIRED)
find_package(mapviz REQUIRED)


# Install launch files
install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
