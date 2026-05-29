cmake_minimum_required(VERSION 3.8)
project(nav2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)
find_package(std_msgs REQUIRED)

ament_python_install_package(
  route_core
  PACKAGE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../../route_core
)

install(DIRECTORY
  config
  launch
  world
  maps
  web
  rviz
  models
  src
  DESTINATION share/${PROJECT_NAME}/
)

install(PROGRAMS
  src/test_action.py
  src/goal_pose_bridge.py
  src/lm_route_manager.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  PROGRAMS
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
