// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from robot_msgs:srv/ExecuteRoute.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "robot_msgs/srv/execute_route.hpp"


#ifndef ROBOT_MSGS__SRV__DETAIL__EXECUTE_ROUTE__BUILDER_HPP_
#define ROBOT_MSGS__SRV__DETAIL__EXECUTE_ROUTE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "robot_msgs/srv/detail/execute_route__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace robot_msgs
{

namespace srv
{

namespace builder
{

class Init_ExecuteRoute_Request_route_json
{
public:
  Init_ExecuteRoute_Request_route_json()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::robot_msgs::srv::ExecuteRoute_Request route_json(::robot_msgs::srv::ExecuteRoute_Request::_route_json_type arg)
  {
    msg_.route_json = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robot_msgs::srv::ExecuteRoute_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::robot_msgs::srv::ExecuteRoute_Request>()
{
  return robot_msgs::srv::builder::Init_ExecuteRoute_Request_route_json();
}

}  // namespace robot_msgs


namespace robot_msgs
{

namespace srv
{

namespace builder
{

class Init_ExecuteRoute_Response_error
{
public:
  explicit Init_ExecuteRoute_Response_error(::robot_msgs::srv::ExecuteRoute_Response & msg)
  : msg_(msg)
  {}
  ::robot_msgs::srv::ExecuteRoute_Response error(::robot_msgs::srv::ExecuteRoute_Response::_error_type arg)
  {
    msg_.error = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robot_msgs::srv::ExecuteRoute_Response msg_;
};

class Init_ExecuteRoute_Response_ok
{
public:
  Init_ExecuteRoute_Response_ok()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ExecuteRoute_Response_error ok(::robot_msgs::srv::ExecuteRoute_Response::_ok_type arg)
  {
    msg_.ok = std::move(arg);
    return Init_ExecuteRoute_Response_error(msg_);
  }

private:
  ::robot_msgs::srv::ExecuteRoute_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::robot_msgs::srv::ExecuteRoute_Response>()
{
  return robot_msgs::srv::builder::Init_ExecuteRoute_Response_ok();
}

}  // namespace robot_msgs


namespace robot_msgs
{

namespace srv
{

namespace builder
{

class Init_ExecuteRoute_Event_response
{
public:
  explicit Init_ExecuteRoute_Event_response(::robot_msgs::srv::ExecuteRoute_Event & msg)
  : msg_(msg)
  {}
  ::robot_msgs::srv::ExecuteRoute_Event response(::robot_msgs::srv::ExecuteRoute_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robot_msgs::srv::ExecuteRoute_Event msg_;
};

class Init_ExecuteRoute_Event_request
{
public:
  explicit Init_ExecuteRoute_Event_request(::robot_msgs::srv::ExecuteRoute_Event & msg)
  : msg_(msg)
  {}
  Init_ExecuteRoute_Event_response request(::robot_msgs::srv::ExecuteRoute_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_ExecuteRoute_Event_response(msg_);
  }

private:
  ::robot_msgs::srv::ExecuteRoute_Event msg_;
};

class Init_ExecuteRoute_Event_info
{
public:
  Init_ExecuteRoute_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ExecuteRoute_Event_request info(::robot_msgs::srv::ExecuteRoute_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_ExecuteRoute_Event_request(msg_);
  }

private:
  ::robot_msgs::srv::ExecuteRoute_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::robot_msgs::srv::ExecuteRoute_Event>()
{
  return robot_msgs::srv::builder::Init_ExecuteRoute_Event_info();
}

}  // namespace robot_msgs

#endif  // ROBOT_MSGS__SRV__DETAIL__EXECUTE_ROUTE__BUILDER_HPP_
