cmake_minimum_required(VERSION 3.8)
project(gcs_bringup)

find_package(ament_cmake REQUIRED)
find_package(arianna_description REQUIRED)
find_package(dottorcane_description REQUIRED)

# Install Launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

# Install RViz files
install(DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME})

ament_package()
