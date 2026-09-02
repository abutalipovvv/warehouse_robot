#ifndef STAGE_ROS2__ROBOT_DOMAIN_MAP_HPP_
#define STAGE_ROS2__ROBOT_DOMAIN_MAP_HPP_

#include <cstddef>
#include <map>
#include <string>

#include "stage_ros2/visibility.h"

namespace stage_ros2
{

STAGE_ROS2_PACKAGE_PUBLIC
std::map<std::string, size_t> parse_robot_domain_map(const std::string & config);

}  // namespace stage_ros2

#endif  // STAGE_ROS2__ROBOT_DOMAIN_MAP_HPP_
