cmake_minimum_required(VERSION 3.8)
project(rv2_server_compositions)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Check ROS_DISTRO
set(MY_ROS2_DISTRO $ENV{ROS_DISTRO})
if(${MY_ROS2_DISTRO} STREQUAL "foxy")
  add_definitions(-DFOUND_FOXY)
elseif(${MY_ROS2_DISTRO} STREQUAL "humble")
  add_definitions(-DFOUND_HUMBLE)
else()
  message(FATAL_ERROR "ROS_DISTRO is not found or not supported.")
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
find_package(rclcpp REQUIRED)
find_package(rv2_interfaces REQUIRED)
find_package(rv2_server_components REQUIRED)


# Build

## Executable
add_executable(devmanageserver_server src/devmanageserver_server.cpp)
ament_target_dependencies(devmanageserver_server rclcpp rv2_interfaces rv2_server_components)

add_executable(devmanageserver_control src/devmanageserver_control.cpp)
ament_target_dependencies(devmanageserver_control rclcpp rv2_interfaces)

add_executable(qosserver_server src/qosserver_server.cpp)
ament_target_dependencies(qosserver_server rclcpp rv2_interfaces rv2_server_components)

add_executable(qosserver_control src/qosserver_control.cpp)
ament_target_dependencies(qosserver_control rclcpp rv2_interfaces)

# Install

## Executable
install(TARGETS
  devmanageserver_control
  devmanageserver_server
  qosserver_control
  qosserver_server
  DESTINATION lib/${PROJECT_NAME}
)

## Files
install(DIRECTORY
  launch params scripts
  DESTINATION share/${PROJECT_NAME}
)

## Requirements
install(FILES
  requirements_apt.txt
  requirements_pip.txt
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
