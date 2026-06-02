// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from robot_msgs:srv/StopRobot.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "robot_msgs/srv/stop_robot.hpp"


#ifndef ROBOT_MSGS__SRV__DETAIL__STOP_ROBOT__BUILDER_HPP_
#define ROBOT_MSGS__SRV__DETAIL__STOP_ROBOT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "robot_msgs/srv/detail/stop_robot__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace robot_msgs
{

namespace srv
{

namespace builder
{

class Init_StopRobot_Request_message
{
public:
  Init_StopRobot_Request_message()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::robot_msgs::srv::StopRobot_Request message(::robot_msgs::srv::StopRobot_Request::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robot_msgs::srv::StopRobot_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::robot_msgs::srv::StopRobot_Request>()
{
  return robot_msgs::srv::builder::Init_StopRobot_Request_message();
}

}  // namespace robot_msgs


namespace robot_msgs
{

namespace srv
{

namespace builder
{

class Init_StopRobot_Response_error
{
public:
  explicit Init_StopRobot_Response_error(::robot_msgs::srv::StopRobot_Response & msg)
  : msg_(msg)
  {}
  ::robot_msgs::srv::StopRobot_Response error(::robot_msgs::srv::StopRobot_Response::_error_type arg)
  {
    msg_.error = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robot_msgs::srv::StopRobot_Response msg_;
};

class Init_StopRobot_Response_ok
{
public:
  Init_StopRobot_Response_ok()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_StopRobot_Response_error ok(::robot_msgs::srv::StopRobot_Response::_ok_type arg)
  {
    msg_.ok = std::move(arg);
    return Init_StopRobot_Response_error(msg_);
  }

private:
  ::robot_msgs::srv::StopRobot_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::robot_msgs::srv::StopRobot_Response>()
{
  return robot_msgs::srv::builder::Init_StopRobot_Response_ok();
}

}  // namespace robot_msgs


namespace robot_msgs
{

namespace srv
{

namespace builder
{

class Init_StopRobot_Event_response
{
public:
  explicit Init_StopRobot_Event_response(::robot_msgs::srv::StopRobot_Event & msg)
  : msg_(msg)
  {}
  ::robot_msgs::srv::StopRobot_Event response(::robot_msgs::srv::StopRobot_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robot_msgs::srv::StopRobot_Event msg_;
};

class Init_StopRobot_Event_request
{
public:
  explicit Init_StopRobot_Event_request(::robot_msgs::srv::StopRobot_Event & msg)
  : msg_(msg)
  {}
  Init_StopRobot_Event_response request(::robot_msgs::srv::StopRobot_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_StopRobot_Event_response(msg_);
  }

private:
  ::robot_msgs::srv::StopRobot_Event msg_;
};

class Init_StopRobot_Event_info
{
public:
  Init_StopRobot_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_StopRobot_Event_request info(::robot_msgs::srv::StopRobot_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_StopRobot_Event_request(msg_);
  }

private:
  ::robot_msgs::srv::StopRobot_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::robot_msgs::srv::StopRobot_Event>()
{
  return robot_msgs::srv::builder::Init_StopRobot_Event_info();
}

}  // namespace robot_msgs

#endif  // ROBOT_MSGS__SRV__DETAIL__STOP_ROBOT__BUILDER_HPP_
