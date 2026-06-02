// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from robot_msgs:srv/PlanRoute.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "robot_msgs/srv/plan_route.hpp"


#ifndef ROBOT_MSGS__SRV__DETAIL__PLAN_ROUTE__TRAITS_HPP_
#define ROBOT_MSGS__SRV__DETAIL__PLAN_ROUTE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "robot_msgs/srv/detail/plan_route__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace robot_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const PlanRoute_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: goal_lm
  {
    out << "goal_lm: ";
    rosidl_generator_traits::value_to_yaml(msg.goal_lm, out);
    out << ", ";
  }

  // member: start_lm
  {
    out << "start_lm: ";
    rosidl_generator_traits::value_to_yaml(msg.start_lm, out);
    out << ", ";
  }

  // member: use_start_pose
  {
    out << "use_start_pose: ";
    rosidl_generator_traits::value_to_yaml(msg.use_start_pose, out);
    out << ", ";
  }

  // member: start_x
  {
    out << "start_x: ";
    rosidl_generator_traits::value_to_yaml(msg.start_x, out);
    out << ", ";
  }

  // member: start_y
  {
    out << "start_y: ";
    rosidl_generator_traits::value_to_yaml(msg.start_y, out);
    out << ", ";
  }

  // member: start_yaw
  {
    out << "start_yaw: ";
    rosidl_generator_traits::value_to_yaml(msg.start_yaw, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const PlanRoute_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: goal_lm
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal_lm: ";
    rosidl_generator_traits::value_to_yaml(msg.goal_lm, out);
    out << "\n";
  }

  // member: start_lm
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "start_lm: ";
    rosidl_generator_traits::value_to_yaml(msg.start_lm, out);
    out << "\n";
  }

  // member: use_start_pose
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "use_start_pose: ";
    rosidl_generator_traits::value_to_yaml(msg.use_start_pose, out);
    out << "\n";
  }

  // member: start_x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "start_x: ";
    rosidl_generator_traits::value_to_yaml(msg.start_x, out);
    out << "\n";
  }

  // member: start_y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "start_y: ";
    rosidl_generator_traits::value_to_yaml(msg.start_y, out);
    out << "\n";
  }

  // member: start_yaw
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "start_yaw: ";
    rosidl_generator_traits::value_to_yaml(msg.start_yaw, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const PlanRoute_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace robot_msgs

namespace rosidl_generator_traits
{

[[deprecated("use robot_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const robot_msgs::srv::PlanRoute_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  robot_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use robot_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const robot_msgs::srv::PlanRoute_Request & msg)
{
  return robot_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<robot_msgs::srv::PlanRoute_Request>()
{
  return "robot_msgs::srv::PlanRoute_Request";
}

template<>
inline const char * name<robot_msgs::srv::PlanRoute_Request>()
{
  return "robot_msgs/srv/PlanRoute_Request";
}

template<>
struct has_fixed_size<robot_msgs::srv::PlanRoute_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<robot_msgs::srv::PlanRoute_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<robot_msgs::srv::PlanRoute_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace robot_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const PlanRoute_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: ok
  {
    out << "ok: ";
    rosidl_generator_traits::value_to_yaml(msg.ok, out);
    out << ", ";
  }

  // member: error
  {
    out << "error: ";
    rosidl_generator_traits::value_to_yaml(msg.error, out);
    out << ", ";
  }

  // member: route_json
  {
    out << "route_json: ";
    rosidl_generator_traits::value_to_yaml(msg.route_json, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const PlanRoute_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: ok
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ok: ";
    rosidl_generator_traits::value_to_yaml(msg.ok, out);
    out << "\n";
  }

  // member: error
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "error: ";
    rosidl_generator_traits::value_to_yaml(msg.error, out);
    out << "\n";
  }

  // member: route_json
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "route_json: ";
    rosidl_generator_traits::value_to_yaml(msg.route_json, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const PlanRoute_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace robot_msgs

namespace rosidl_generator_traits
{

[[deprecated("use robot_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const robot_msgs::srv::PlanRoute_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  robot_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use robot_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const robot_msgs::srv::PlanRoute_Response & msg)
{
  return robot_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<robot_msgs::srv::PlanRoute_Response>()
{
  return "robot_msgs::srv::PlanRoute_Response";
}

template<>
inline const char * name<robot_msgs::srv::PlanRoute_Response>()
{
  return "robot_msgs/srv/PlanRoute_Response";
}

template<>
struct has_fixed_size<robot_msgs::srv::PlanRoute_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<robot_msgs::srv::PlanRoute_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<robot_msgs::srv::PlanRoute_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__traits.hpp"

namespace robot_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const PlanRoute_Event & msg,
  std::ostream & out)
{
  out << "{";
  // member: info
  {
    out << "info: ";
    to_flow_style_yaml(msg.info, out);
    out << ", ";
  }

  // member: request
  {
    if (msg.request.size() == 0) {
      out << "request: []";
    } else {
      out << "request: [";
      size_t pending_items = msg.request.size();
      for (auto item : msg.request) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: response
  {
    if (msg.response.size() == 0) {
      out << "response: []";
    } else {
      out << "response: [";
      size_t pending_items = msg.response.size();
      for (auto item : msg.response) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const PlanRoute_Event & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: info
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "info:\n";
    to_block_style_yaml(msg.info, out, indentation + 2);
  }

  // member: request
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.request.size() == 0) {
      out << "request: []\n";
    } else {
      out << "request:\n";
      for (auto item : msg.request) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: response
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.response.size() == 0) {
      out << "response: []\n";
    } else {
      out << "response:\n";
      for (auto item : msg.response) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const PlanRoute_Event & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace robot_msgs

namespace rosidl_generator_traits
{

[[deprecated("use robot_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const robot_msgs::srv::PlanRoute_Event & msg,
  std::ostream & out, size_t indentation = 0)
{
  robot_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use robot_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const robot_msgs::srv::PlanRoute_Event & msg)
{
  return robot_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<robot_msgs::srv::PlanRoute_Event>()
{
  return "robot_msgs::srv::PlanRoute_Event";
}

template<>
inline const char * name<robot_msgs::srv::PlanRoute_Event>()
{
  return "robot_msgs/srv/PlanRoute_Event";
}

template<>
struct has_fixed_size<robot_msgs::srv::PlanRoute_Event>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<robot_msgs::srv::PlanRoute_Event>
  : std::integral_constant<bool, has_bounded_size<robot_msgs::srv::PlanRoute_Request>::value && has_bounded_size<robot_msgs::srv::PlanRoute_Response>::value && has_bounded_size<service_msgs::msg::ServiceEventInfo>::value> {};

template<>
struct is_message<robot_msgs::srv::PlanRoute_Event>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<robot_msgs::srv::PlanRoute>()
{
  return "robot_msgs::srv::PlanRoute";
}

template<>
inline const char * name<robot_msgs::srv::PlanRoute>()
{
  return "robot_msgs/srv/PlanRoute";
}

template<>
struct has_fixed_size<robot_msgs::srv::PlanRoute>
  : std::integral_constant<
    bool,
    has_fixed_size<robot_msgs::srv::PlanRoute_Request>::value &&
    has_fixed_size<robot_msgs::srv::PlanRoute_Response>::value
  >
{
};

template<>
struct has_bounded_size<robot_msgs::srv::PlanRoute>
  : std::integral_constant<
    bool,
    has_bounded_size<robot_msgs::srv::PlanRoute_Request>::value &&
    has_bounded_size<robot_msgs::srv::PlanRoute_Response>::value
  >
{
};

template<>
struct is_service<robot_msgs::srv::PlanRoute>
  : std::true_type
{
};

template<>
struct is_service_request<robot_msgs::srv::PlanRoute_Request>
  : std::true_type
{
};

template<>
struct is_service_response<robot_msgs::srv::PlanRoute_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // ROBOT_MSGS__SRV__DETAIL__PLAN_ROUTE__TRAITS_HPP_
