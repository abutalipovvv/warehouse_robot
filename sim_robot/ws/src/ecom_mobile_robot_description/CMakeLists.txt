cmake_minimum_required(VERSION 3.8)
project(ecom_mobile_robot_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch meshes config
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

install(
  FILES urdf/ecom_stage.urdf.xacro
  DESTINATION share/${PROJECT_NAME}/urdf
)

ament_package()
