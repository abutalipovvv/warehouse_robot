// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from robot_msgs:msg/RobotStatus.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "robot_msgs/msg/robot_status.hpp"


#ifndef ROBOT_MSGS__MSG__DETAIL__ROBOT_STATUS__TRAITS_HPP_
#define ROBOT_MSGS__MSG__DETAIL__ROBOT_STATUS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "robot_msgs/msg/detail/robot_status__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__traits.hpp"

namespace robot_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const RobotStatus & msg,
  std::ostream & out)
{
  out << "{";
  // member: stamp
  {
    out << "stamp: ";
    to_flow_style_yaml(msg.stamp, out);
    out << ", ";
  }

  // member: robot_id
  {
    out << "robot_id: ";
    rosidl_generator_traits::value_to_yaml(msg.robot_id, out);
    out << ", ";
  }

  // member: map_id
  {
    out << "map_id: ";
    rosidl_generator_traits::value_to_yaml(msg.map_id, out);
    out << ", ";
  }

  // member: connected
  {
    out << "connected: ";
    rosidl_generator_traits::value_to_yaml(msg.connected, out);
    out << ", ";
  }

  // member: localization_ok
  {
    out << "localization_ok: ";
    rosidl_generator_traits::value_to_yaml(msg.localization_ok, out);
    out << ", ";
  }

  // member: localization_age_sec
  {
    out << "localization_age_sec: ";
    rosidl_generator_traits::value_to_yaml(msg.localization_age_sec, out);
    out << ", ";
  }

  // member: state
  {
    out << "state: ";
    rosidl_generator_traits::value_to_yaml(msg.state, out);
    out << ", ";
  }

  // member: message
  {
    out << "message: ";
    rosidl_generator_traits::value_to_yaml(msg.message, out);
    out << ", ";
  }

  // member: target_lm
  {
    out << "target_lm: ";
    rosidl_generator_traits::value_to_yaml(msg.target_lm, out);
    out << ", ";
  }

  // member: nearest_lm
  {
    out << "nearest_lm: ";
    rosidl_generator_traits::value_to_yaml(msg.nearest_lm, out);
    out << ", ";
  }

  // member: current_edge_id
  {
    out << "current_edge_id: ";
    rosidl_generator_traits::value_to_yaml(msg.current_edge_id, out);
    out << ", ";
  }

  // member: route_id
  {
    out << "route_id: ";
    rosidl_generator_traits::value_to_yaml(msg.route_id, out);
    out << ", ";
  }

  // member: route_progress
  {
    out << "route_progress: ";
    rosidl_generator_traits::value_to_yaml(msg.route_progress, out);
    out << ", ";
  }

  // member: pose_x
  {
    out << "pose_x: ";
    rosidl_generator_traits::value_to_yaml(msg.pose_x, out);
    out << ", ";
  }

  // member: pose_y
  {
    out << "pose_y: ";
    rosidl_generator_traits::value_to_yaml(msg.pose_y, out);
    out << ", ";
  }

  // member: pose_yaw
  {
    out << "pose_yaw: ";
    rosidl_generator_traits::value_to_yaml(msg.pose_yaw, out);
    out << ", ";
  }

  // member: linear_velocity
  {
    out << "linear_velocity: ";
    rosidl_generator_traits::value_to_yaml(msg.linear_velocity, out);
    out << ", ";
  }

  // member: angular_velocity
  {
    out << "angular_velocity: ";
    rosidl_generator_traits::value_to_yaml(msg.angular_velocity, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const RobotStatus & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: stamp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "stamp:\n";
    to_block_style_yaml(msg.stamp, out, indentation + 2);
  }

  // member: robot_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "robot_id: ";
    rosidl_generator_traits::value_to_yaml(msg.robot_id, out);
    out << "\n";
  }

  // member: map_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "map_id: ";
    rosidl_generator_traits::value_to_yaml(msg.map_id, out);
    out << "\n";
  }

  // member: connected
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "connected: ";
    rosidl_generator_traits::value_to_yaml(msg.connected, out);
    out << "\n";
  }

  // member: localization_ok
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "localization_ok: ";
    rosidl_generator_traits::value_to_yaml(msg.localization_ok, out);
    out << "\n";
  }

  // member: localization_age_sec
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "localization_age_sec: ";
    rosidl_generator_traits::value_to_yaml(msg.localization_age_sec, out);
    out << "\n";
  }

  // member: state
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "state: ";
    rosidl_generator_traits::value_to_yaml(msg.state, out);
    out << "\n";
  }

  // member: message
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "message: ";
    rosidl_generator_traits::value_to_yaml(msg.message, out);
    out << "\n";
  }

  // member: target_lm
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "target_lm: ";
    rosidl_generator_traits::value_to_yaml(msg.target_lm, out);
    out << "\n";
  }

  // member: nearest_lm
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "nearest_lm: ";
    rosidl_generator_traits::value_to_yaml(msg.nearest_lm, out);
    out << "\n";
  }

  // member: current_edge_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "current_edge_id: ";
    rosidl_generator_traits::value_to_yaml(msg.current_edge_id, out);
    out << "\n";
  }

  // member: route_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "route_id: ";
    rosidl_generator_traits::value_to_yaml(msg.route_id, out);
    out << "\n";
  }

  // member: route_progress
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "route_progress: ";
    rosidl_generator_traits::value_to_yaml(msg.route_progress, out);
    out << "\n";
  }

  // member: pose_x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pose_x: ";
    rosidl_generator_traits::value_to_yaml(msg.pose_x, out);
    out << "\n";
  }

  // member: pose_y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pose_y: ";
    rosidl_generator_traits::value_to_yaml(msg.pose_y, out);
    out << "\n";
  }

  // member: pose_yaw
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pose_yaw: ";
    rosidl_generator_traits::value_to_yaml(msg.pose_yaw, out);
    out << "\n";
  }

  // member: linear_velocity
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "linear_velocity: ";
    rosidl_generator_traits::value_to_yaml(msg.linear_velocity, out);
    out << "\n";
  }

  // member: angular_velocity
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "angular_velocity: ";
    rosidl_generator_traits::value_to_yaml(msg.angular_velocity, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const RobotStatus & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace robot_msgs

namespace rosidl_generator_traits
{

[[deprecated("use robot_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const robot_msgs::msg::RobotStatus & msg,
  std::ostream & out, size_t indentation = 0)
{
  robot_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use robot_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const robot_msgs::msg::RobotStatus & msg)
{
  return robot_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<robot_msgs::msg::RobotStatus>()
{
  return "robot_msgs::msg::RobotStatus";
}

template<>
inline const char * name<robot_msgs::msg::RobotStatus>()
{
  return "robot_msgs/msg/RobotStatus";
}

template<>
struct has_fixed_size<robot_msgs::msg::RobotStatus>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<robot_msgs::msg::RobotStatus>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<robot_msgs::msg::RobotStatus>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ROBOT_MSGS__MSG__DETAIL__ROBOT_STATUS__TRAITS_HPP_
