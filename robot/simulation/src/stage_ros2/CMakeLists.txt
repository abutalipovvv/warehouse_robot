cmake_minimum_required(VERSION 3.8)
project(stage_ros2)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(stage REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)

include_directories(
  ./include
  ${STAGE_INCLUDE_DIRS}
)

ament_python_install_package(${PROJECT_NAME})

add_library(stage_node SHARED
            src/stage_node.cpp src/robot_domain_map.cpp src/laser_scan_merger.cpp src/vehicle.cpp
            src/ranger.cpp src/camera.cpp)
target_compile_definitions(stage_node PRIVATE "STAGE_ROS2_PACKAGE_DLL")
ament_target_dependencies(stage_node rclcpp rclcpp_components
sensor_msgs geometry_msgs nav_msgs std_msgs tf2 tf2_ros std_srvs tf2_geometry_msgs)


# This package installs libraries without exporting them.
# Export the library path to ensure that the installed libraries are available.
if(NOT WIN32)
  ament_environment_hooks(
    "${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}")
endif()

add_executable(stage_ros2 src/main.cpp)
target_include_directories(stage_ros2 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(stage_ros2 PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_link_libraries(stage_ros2 stage_node ${STAGE_LIBRARIES})
ament_target_dependencies(stage_ros2 rclcpp)

install(
  DIRECTORY
  launch
  world
  config
  res
  DESTINATION
    share/${PROJECT_NAME}/)

install(
  DIRECTORY include/
  DESTINATION include)

install(TARGETS
  stage_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS
  stage_ros2
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_add_gtest(
    test_robot_domain_map
    test/test_robot_domain_map.cpp
    src/robot_domain_map.cpp)
  target_include_directories(test_robot_domain_map PRIVATE include)
  ament_add_gtest(
    test_laser_scan_merger
    test/test_laser_scan_merger.cpp
    src/laser_scan_merger.cpp)
  target_include_directories(test_laser_scan_merger PRIVATE include)
  ament_add_pytest_test(
    test_fleet_world
    test/test_fleet_world.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR})
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
