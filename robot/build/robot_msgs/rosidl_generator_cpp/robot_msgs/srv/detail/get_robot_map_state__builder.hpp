// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from robot_msgs:srv/GetRobotMapState.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "robot_msgs/srv/get_robot_map_state.hpp"


#ifndef ROBOT_MSGS__SRV__DETAIL__GET_ROBOT_MAP_STATE__BUILDER_HPP_
#define ROBOT_MSGS__SRV__DETAIL__GET_ROBOT_MAP_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "robot_msgs/srv/detail/get_robot_map_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace robot_msgs
{

namespace srv
{


}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::robot_msgs::srv::GetRobotMapState_Request>()
{
  return ::robot_msgs::srv::GetRobotMapState_Request(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace robot_msgs


namespace robot_msgs
{

namespace srv
{

namespace builder
{

class Init_GetRobotMapState_Response_map_id
{
public:
  explicit Init_GetRobotMapState_Response_map_id(::robot_msgs::srv::GetRobotMapState_Response & msg)
  : msg_(msg)
  {}
  ::robot_msgs::srv::GetRobotMapState_Response map_id(::robot_msgs::srv::GetRobotMapState_Response::_map_id_type arg)
  {
    msg_.map_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robot_msgs::srv::GetRobotMapState_Response msg_;
};

class Init_GetRobotMapState_Response_map_dir
{
public:
  explicit Init_GetRobotMapState_Response_map_dir(::robot_msgs::srv::GetRobotMapState_Response & msg)
  : msg_(msg)
  {}
  Init_GetRobotMapState_Response_map_id map_dir(::robot_msgs::srv::GetRobotMapState_Response::_map_dir_type arg)
  {
    msg_.map_dir = std::move(arg);
    return Init_GetRobotMapState_Response_map_id(msg_);
  }

private:
  ::robot_msgs::srv::GetRobotMapState_Response msg_;
};

class Init_GetRobotMapState_Response_map_name
{
public:
  explicit Init_GetRobotMapState_Response_map_name(::robot_msgs::srv::GetRobotMapState_Response & msg)
  : msg_(msg)
  {}
  Init_GetRobotMapState_Response_map_dir map_name(::robot_msgs::srv::GetRobotMapState_Response::_map_name_type arg)
  {
    msg_.map_name = std::move(arg);
    return Init_GetRobotMapState_Response_map_dir(msg_);
  }

private:
  ::robot_msgs::srv::GetRobotMapState_Response msg_;
};

class Init_GetRobotMapState_Response_error
{
public:
  explicit Init_GetRobotMapState_Response_error(::robot_msgs::srv::GetRobotMapState_Response & msg)
  : msg_(msg)
  {}
  Init_GetRobotMapState_Response_map_name error(::robot_msgs::srv::GetRobotMapState_Response::_error_type arg)
  {
    msg_.error = std::move(arg);
    return Init_GetRobotMapState_Response_map_name(msg_);
  }

private:
  ::robot_msgs::srv::GetRobotMapState_Response msg_;
};

class Init_GetRobotMapState_Response_ok
{
public:
  Init_GetRobotMapState_Response_ok()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GetRobotMapState_Response_error ok(::robot_msgs::srv::GetRobotMapState_Response::_ok_type arg)
  {
    msg_.ok = std::move(arg);
    return Init_GetRobotMapState_Response_error(msg_);
  }

private:
  ::robot_msgs::srv::GetRobotMapState_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::robot_msgs::srv::GetRobotMapState_Response>()
{
  return robot_msgs::srv::builder::Init_GetRobotMapState_Response_ok();
}

}  // namespace robot_msgs


namespace robot_msgs
{

namespace srv
{

namespace builder
{

class Init_GetRobotMapState_Event_response
{
public:
  explicit Init_GetRobotMapState_Event_response(::robot_msgs::srv::GetRobotMapState_Event & msg)
  : msg_(msg)
  {}
  ::robot_msgs::srv::GetRobotMapState_Event response(::robot_msgs::srv::GetRobotMapState_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robot_msgs::srv::GetRobotMapState_Event msg_;
};

class Init_GetRobotMapState_Event_request
{
public:
  explicit Init_GetRobotMapState_Event_request(::robot_msgs::srv::GetRobotMapState_Event & msg)
  : msg_(msg)
  {}
  Init_GetRobotMapState_Event_response request(::robot_msgs::srv::GetRobotMapState_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_GetRobotMapState_Event_response(msg_);
  }

private:
  ::robot_msgs::srv::GetRobotMapState_Event msg_;
};

class Init_GetRobotMapState_Event_info
{
public:
  Init_GetRobotMapState_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GetRobotMapState_Event_request info(::robot_msgs::srv::GetRobotMapState_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_GetRobotMapState_Event_request(msg_);
  }

private:
  ::robot_msgs::srv::GetRobotMapState_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::robot_msgs::srv::GetRobotMapState_Event>()
{
  return robot_msgs::srv::builder::Init_GetRobotMapState_Event_info();
}

}  // namespace robot_msgs

#endif  // ROBOT_MSGS__SRV__DETAIL__GET_ROBOT_MAP_STATE__BUILDER_HPP_
