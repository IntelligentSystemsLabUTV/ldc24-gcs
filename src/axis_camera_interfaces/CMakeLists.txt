cmake_minimum_required(VERSION 3.8)
project(axis_camera_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(action_msgs REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(vision_msgs REQUIRED)

set(ACTION_DIR "${CMAKE_CURRENT_SOURCE_DIR}/action")
file(GLOB ACTION_FILES RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" "${ACTION_DIR}/*.action")

set(SRV_DIR "${CMAKE_CURRENT_SOURCE_DIR}/srv")
file(GLOB SRV_FILES RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" "${SRV_DIR}/*.srv")

set(MSG_DIR "${CMAKE_CURRENT_SOURCE_DIR}/msg")
file(GLOB MSG_FILES RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" "${MSG_DIR}/*.msg")

rosidl_generate_interfaces(${PROJECT_NAME}
  ${ACTION_FILES}
  ${SRV_FILES}
  ${MSG_FILES}
  DEPENDENCIES geometry_msgs sensor_msgs std_msgs vision_msgs)

ament_export_dependencies(
  action_msgs
  geometry_msgs
  sensor_msgs
  std_msgs
  vision_msgs)

ament_package()
