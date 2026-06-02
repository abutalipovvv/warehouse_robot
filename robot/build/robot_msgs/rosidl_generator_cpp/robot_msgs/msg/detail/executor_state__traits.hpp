// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from robot_msgs:msg/ExecutorState.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "robot_msgs/msg/executor_state.hpp"


#ifndef ROBOT_MSGS__MSG__DETAIL__EXECUTOR_STATE__TRAITS_HPP_
#define ROBOT_MSGS__MSG__DETAIL__EXECUTOR_STATE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "robot_msgs/msg/detail/executor_state__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__traits.hpp"

namespace robot_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const ExecutorState & msg,
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

  // member: route_active
  {
    out << "route_active: ";
    rosidl_generator_traits::value_to_yaml(msg.route_active, out);
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
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ExecutorState & msg,
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

  // member: route_active
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "route_active: ";
    rosidl_generator_traits::value_to_yaml(msg.route_active, out);
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
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ExecutorState & msg, bool use_flow_style = false)
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
  const robot_msgs::msg::ExecutorState & msg,
  std::ostream & out, size_t indentation = 0)
{
  robot_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use robot_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const robot_msgs::msg::ExecutorState & msg)
{
  return robot_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<robot_msgs::msg::ExecutorState>()
{
  return "robot_msgs::msg::ExecutorState";
}

template<>
inline const char * name<robot_msgs::msg::ExecutorState>()
{
  return "robot_msgs/msg/ExecutorState";
}

template<>
struct has_fixed_size<robot_msgs::msg::ExecutorState>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<robot_msgs::msg::ExecutorState>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<robot_msgs::msg::ExecutorState>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ROBOT_MSGS__MSG__DETAIL__EXECUTOR_STATE__TRAITS_HPP_
