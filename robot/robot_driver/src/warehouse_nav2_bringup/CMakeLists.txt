cmake_minimum_required(VERSION 3.8)
project(warehouse_nav2_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  config
  launch
  rviz
  DESTINATION share/${PROJECT_NAME}/
  PATTERN "__pycache__" EXCLUDE
)

ament_package()
