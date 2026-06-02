// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from robot_msgs:srv/SetTeleop.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "robot_msgs/srv/set_teleop.hpp"


#ifndef ROBOT_MSGS__SRV__DETAIL__SET_TELEOP__BUILDER_HPP_
#define ROBOT_MSGS__SRV__DETAIL__SET_TELEOP__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "robot_msgs/srv/detail/set_teleop__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace robot_msgs
{

namespace srv
{

namespace builder
{

class Init_SetTeleop_Request_timeout_ms
{
public:
  explicit Init_SetTeleop_Request_timeout_ms(::robot_msgs::srv::SetTeleop_Request & msg)
  : msg_(msg)
  {}
  ::robot_msgs::srv::SetTeleop_Request timeout_ms(::robot_msgs::srv::SetTeleop_Request::_timeout_ms_type arg)
  {
    msg_.timeout_ms = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robot_msgs::srv::SetTeleop_Request msg_;
};

class Init_SetTeleop_Request_angular
{
public:
  explicit Init_SetTeleop_Request_angular(::robot_msgs::srv::SetTeleop_Request & msg)
  : msg_(msg)
  {}
  Init_SetTeleop_Request_timeout_ms angular(::robot_msgs::srv::SetTeleop_Request::_angular_type arg)
  {
    msg_.angular = std::move(arg);
    return Init_SetTeleop_Request_timeout_ms(msg_);
  }

private:
  ::robot_msgs::srv::SetTeleop_Request msg_;
};

class Init_SetTeleop_Request_linear
{
public:
  Init_SetTeleop_Request_linear()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SetTeleop_Request_angular linear(::robot_msgs::srv::SetTeleop_Request::_linear_type arg)
  {
    msg_.linear = std::move(arg);
    return Init_SetTeleop_Request_angular(msg_);
  }

private:
  ::robot_msgs::srv::SetTeleop_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::robot_msgs::srv::SetTeleop_Request>()
{
  return robot_msgs::srv::builder::Init_SetTeleop_Request_linear();
}

}  // namespace robot_msgs


namespace robot_msgs
{

namespace srv
{

namespace builder
{

class Init_SetTeleop_Response_error
{
public:
  explicit Init_SetTeleop_Response_error(::robot_msgs::srv::SetTeleop_Response & msg)
  : msg_(msg)
  {}
  ::robot_msgs::srv::SetTeleop_Response error(::robot_msgs::srv::SetTeleop_Response::_error_type arg)
  {
    msg_.error = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robot_msgs::srv::SetTeleop_Response msg_;
};

class Init_SetTeleop_Response_ok
{
public:
  Init_SetTeleop_Response_ok()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SetTeleop_Response_error ok(::robot_msgs::srv::SetTeleop_Response::_ok_type arg)
  {
    msg_.ok = std::move(arg);
    return Init_SetTeleop_Response_error(msg_);
  }

private:
  ::robot_msgs::srv::SetTeleop_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::robot_msgs::srv::SetTeleop_Response>()
{
  return robot_msgs::srv::builder::Init_SetTeleop_Response_ok();
}

}  // namespace robot_msgs


namespace robot_msgs
{

namespace srv
{

namespace builder
{

class Init_SetTeleop_Event_response
{
public:
  explicit Init_SetTeleop_Event_response(::robot_msgs::srv::SetTeleop_Event & msg)
  : msg_(msg)
  {}
  ::robot_msgs::srv::SetTeleop_Event response(::robot_msgs::srv::SetTeleop_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robot_msgs::srv::SetTeleop_Event msg_;
};

class Init_SetTeleop_Event_request
{
public:
  explicit Init_SetTeleop_Event_request(::robot_msgs::srv::SetTeleop_Event & msg)
  : msg_(msg)
  {}
  Init_SetTeleop_Event_response request(::robot_msgs::srv::SetTeleop_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_SetTeleop_Event_response(msg_);
  }

private:
  ::robot_msgs::srv::SetTeleop_Event msg_;
};

class Init_SetTeleop_Event_info
{
public:
  Init_SetTeleop_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SetTeleop_Event_request info(::robot_msgs::srv::SetTeleop_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_SetTeleop_Event_request(msg_);
  }

private:
  ::robot_msgs::srv::SetTeleop_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::robot_msgs::srv::SetTeleop_Event>()
{
  return robot_msgs::srv::builder::Init_SetTeleop_Event_info();
}

}  // namespace robot_msgs

#endif  // ROBOT_MSGS__SRV__DETAIL__SET_TELEOP__BUILDER_HPP_
