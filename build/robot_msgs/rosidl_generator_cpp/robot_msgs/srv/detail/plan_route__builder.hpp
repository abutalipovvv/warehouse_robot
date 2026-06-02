// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from robot_msgs:srv/PlanRoute.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "robot_msgs/srv/plan_route.hpp"


#ifndef ROBOT_MSGS__SRV__DETAIL__PLAN_ROUTE__BUILDER_HPP_
#define ROBOT_MSGS__SRV__DETAIL__PLAN_ROUTE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "robot_msgs/srv/detail/plan_route__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace robot_msgs
{

namespace srv
{

namespace builder
{

class Init_PlanRoute_Request_start_yaw
{
public:
  explicit Init_PlanRoute_Request_start_yaw(::robot_msgs::srv::PlanRoute_Request & msg)
  : msg_(msg)
  {}
  ::robot_msgs::srv::PlanRoute_Request start_yaw(::robot_msgs::srv::PlanRoute_Request::_start_yaw_type arg)
  {
    msg_.start_yaw = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robot_msgs::srv::PlanRoute_Request msg_;
};

class Init_PlanRoute_Request_start_y
{
public:
  explicit Init_PlanRoute_Request_start_y(::robot_msgs::srv::PlanRoute_Request & msg)
  : msg_(msg)
  {}
  Init_PlanRoute_Request_start_yaw start_y(::robot_msgs::srv::PlanRoute_Request::_start_y_type arg)
  {
    msg_.start_y = std::move(arg);
    return Init_PlanRoute_Request_start_yaw(msg_);
  }

private:
  ::robot_msgs::srv::PlanRoute_Request msg_;
};

class Init_PlanRoute_Request_start_x
{
public:
  explicit Init_PlanRoute_Request_start_x(::robot_msgs::srv::PlanRoute_Request & msg)
  : msg_(msg)
  {}
  Init_PlanRoute_Request_start_y start_x(::robot_msgs::srv::PlanRoute_Request::_start_x_type arg)
  {
    msg_.start_x = std::move(arg);
    return Init_PlanRoute_Request_start_y(msg_);
  }

private:
  ::robot_msgs::srv::PlanRoute_Request msg_;
};

class Init_PlanRoute_Request_use_start_pose
{
public:
  explicit Init_PlanRoute_Request_use_start_pose(::robot_msgs::srv::PlanRoute_Request & msg)
  : msg_(msg)
  {}
  Init_PlanRoute_Request_start_x use_start_pose(::robot_msgs::srv::PlanRoute_Request::_use_start_pose_type arg)
  {
    msg_.use_start_pose = std::move(arg);
    return Init_PlanRoute_Request_start_x(msg_);
  }

private:
  ::robot_msgs::srv::PlanRoute_Request msg_;
};

class Init_PlanRoute_Request_start_lm
{
public:
  explicit Init_PlanRoute_Request_start_lm(::robot_msgs::srv::PlanRoute_Request & msg)
  : msg_(msg)
  {}
  Init_PlanRoute_Request_use_start_pose start_lm(::robot_msgs::srv::PlanRoute_Request::_start_lm_type arg)
  {
    msg_.start_lm = std::move(arg);
    return Init_PlanRoute_Request_use_start_pose(msg_);
  }

private:
  ::robot_msgs::srv::PlanRoute_Request msg_;
};

class Init_PlanRoute_Request_goal_lm
{
public:
  Init_PlanRoute_Request_goal_lm()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PlanRoute_Request_start_lm goal_lm(::robot_msgs::srv::PlanRoute_Request::_goal_lm_type arg)
  {
    msg_.goal_lm = std::move(arg);
    return Init_PlanRoute_Request_start_lm(msg_);
  }

private:
  ::robot_msgs::srv::PlanRoute_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::robot_msgs::srv::PlanRoute_Request>()
{
  return robot_msgs::srv::builder::Init_PlanRoute_Request_goal_lm();
}

}  // namespace robot_msgs


namespace robot_msgs
{

namespace srv
{

namespace builder
{

class Init_PlanRoute_Response_route_json
{
public:
  explicit Init_PlanRoute_Response_route_json(::robot_msgs::srv::PlanRoute_Response & msg)
  : msg_(msg)
  {}
  ::robot_msgs::srv::PlanRoute_Response route_json(::robot_msgs::srv::PlanRoute_Response::_route_json_type arg)
  {
    msg_.route_json = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robot_msgs::srv::PlanRoute_Response msg_;
};

class Init_PlanRoute_Response_error
{
public:
  explicit Init_PlanRoute_Response_error(::robot_msgs::srv::PlanRoute_Response & msg)
  : msg_(msg)
  {}
  Init_PlanRoute_Response_route_json error(::robot_msgs::srv::PlanRoute_Response::_error_type arg)
  {
    msg_.error = std::move(arg);
    return Init_PlanRoute_Response_route_json(msg_);
  }

private:
  ::robot_msgs::srv::PlanRoute_Response msg_;
};

class Init_PlanRoute_Response_ok
{
public:
  Init_PlanRoute_Response_ok()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PlanRoute_Response_error ok(::robot_msgs::srv::PlanRoute_Response::_ok_type arg)
  {
    msg_.ok = std::move(arg);
    return Init_PlanRoute_Response_error(msg_);
  }

private:
  ::robot_msgs::srv::PlanRoute_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::robot_msgs::srv::PlanRoute_Response>()
{
  return robot_msgs::srv::builder::Init_PlanRoute_Response_ok();
}

}  // namespace robot_msgs


namespace robot_msgs
{

namespace srv
{

namespace builder
{

class Init_PlanRoute_Event_response
{
public:
  explicit Init_PlanRoute_Event_response(::robot_msgs::srv::PlanRoute_Event & msg)
  : msg_(msg)
  {}
  ::robot_msgs::srv::PlanRoute_Event response(::robot_msgs::srv::PlanRoute_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robot_msgs::srv::PlanRoute_Event msg_;
};

class Init_PlanRoute_Event_request
{
public:
  explicit Init_PlanRoute_Event_request(::robot_msgs::srv::PlanRoute_Event & msg)
  : msg_(msg)
  {}
  Init_PlanRoute_Event_response request(::robot_msgs::srv::PlanRoute_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_PlanRoute_Event_response(msg_);
  }

private:
  ::robot_msgs::srv::PlanRoute_Event msg_;
};

class Init_PlanRoute_Event_info
{
public:
  Init_PlanRoute_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PlanRoute_Event_request info(::robot_msgs::srv::PlanRoute_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_PlanRoute_Event_request(msg_);
  }

private:
  ::robot_msgs::srv::PlanRoute_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::robot_msgs::srv::PlanRoute_Event>()
{
  return robot_msgs::srv::builder::Init_PlanRoute_Event_info();
}

}  // namespace robot_msgs

#endif  // ROBOT_MSGS__SRV__DETAIL__PLAN_ROUTE__BUILDER_HPP_
