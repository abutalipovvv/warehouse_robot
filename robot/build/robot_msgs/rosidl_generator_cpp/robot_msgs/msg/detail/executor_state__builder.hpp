// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from robot_msgs:msg/ExecutorState.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "robot_msgs/msg/executor_state.hpp"


#ifndef ROBOT_MSGS__MSG__DETAIL__EXECUTOR_STATE__BUILDER_HPP_
#define ROBOT_MSGS__MSG__DETAIL__EXECUTOR_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "robot_msgs/msg/detail/executor_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace robot_msgs
{

namespace msg
{

namespace builder
{

class Init_ExecutorState_route_progress
{
public:
  explicit Init_ExecutorState_route_progress(::robot_msgs::msg::ExecutorState & msg)
  : msg_(msg)
  {}
  ::robot_msgs::msg::ExecutorState route_progress(::robot_msgs::msg::ExecutorState::_route_progress_type arg)
  {
    msg_.route_progress = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robot_msgs::msg::ExecutorState msg_;
};

class Init_ExecutorState_route_id
{
public:
  explicit Init_ExecutorState_route_id(::robot_msgs::msg::ExecutorState & msg)
  : msg_(msg)
  {}
  Init_ExecutorState_route_progress route_id(::robot_msgs::msg::ExecutorState::_route_id_type arg)
  {
    msg_.route_id = std::move(arg);
    return Init_ExecutorState_route_progress(msg_);
  }

private:
  ::robot_msgs::msg::ExecutorState msg_;
};

class Init_ExecutorState_current_edge_id
{
public:
  explicit Init_ExecutorState_current_edge_id(::robot_msgs::msg::ExecutorState & msg)
  : msg_(msg)
  {}
  Init_ExecutorState_route_id current_edge_id(::robot_msgs::msg::ExecutorState::_current_edge_id_type arg)
  {
    msg_.current_edge_id = std::move(arg);
    return Init_ExecutorState_route_id(msg_);
  }

private:
  ::robot_msgs::msg::ExecutorState msg_;
};

class Init_ExecutorState_target_lm
{
public:
  explicit Init_ExecutorState_target_lm(::robot_msgs::msg::ExecutorState & msg)
  : msg_(msg)
  {}
  Init_ExecutorState_current_edge_id target_lm(::robot_msgs::msg::ExecutorState::_target_lm_type arg)
  {
    msg_.target_lm = std::move(arg);
    return Init_ExecutorState_current_edge_id(msg_);
  }

private:
  ::robot_msgs::msg::ExecutorState msg_;
};

class Init_ExecutorState_message
{
public:
  explicit Init_ExecutorState_message(::robot_msgs::msg::ExecutorState & msg)
  : msg_(msg)
  {}
  Init_ExecutorState_target_lm message(::robot_msgs::msg::ExecutorState::_message_type arg)
  {
    msg_.message = std::move(arg);
    return Init_ExecutorState_target_lm(msg_);
  }

private:
  ::robot_msgs::msg::ExecutorState msg_;
};

class Init_ExecutorState_state
{
public:
  explicit Init_ExecutorState_state(::robot_msgs::msg::ExecutorState & msg)
  : msg_(msg)
  {}
  Init_ExecutorState_message state(::robot_msgs::msg::ExecutorState::_state_type arg)
  {
    msg_.state = std::move(arg);
    return Init_ExecutorState_message(msg_);
  }

private:
  ::robot_msgs::msg::ExecutorState msg_;
};

class Init_ExecutorState_route_active
{
public:
  explicit Init_ExecutorState_route_active(::robot_msgs::msg::ExecutorState & msg)
  : msg_(msg)
  {}
  Init_ExecutorState_state route_active(::robot_msgs::msg::ExecutorState::_route_active_type arg)
  {
    msg_.route_active = std::move(arg);
    return Init_ExecutorState_state(msg_);
  }

private:
  ::robot_msgs::msg::ExecutorState msg_;
};

class Init_ExecutorState_map_id
{
public:
  explicit Init_ExecutorState_map_id(::robot_msgs::msg::ExecutorState & msg)
  : msg_(msg)
  {}
  Init_ExecutorState_route_active map_id(::robot_msgs::msg::ExecutorState::_map_id_type arg)
  {
    msg_.map_id = std::move(arg);
    return Init_ExecutorState_route_active(msg_);
  }

private:
  ::robot_msgs::msg::ExecutorState msg_;
};

class Init_ExecutorState_robot_id
{
public:
  explicit Init_ExecutorState_robot_id(::robot_msgs::msg::ExecutorState & msg)
  : msg_(msg)
  {}
  Init_ExecutorState_map_id robot_id(::robot_msgs::msg::ExecutorState::_robot_id_type arg)
  {
    msg_.robot_id = std::move(arg);
    return Init_ExecutorState_map_id(msg_);
  }

private:
  ::robot_msgs::msg::ExecutorState msg_;
};

class Init_ExecutorState_stamp
{
public:
  Init_ExecutorState_stamp()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ExecutorState_robot_id stamp(::robot_msgs::msg::ExecutorState::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return Init_ExecutorState_robot_id(msg_);
  }

private:
  ::robot_msgs::msg::ExecutorState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::robot_msgs::msg::ExecutorState>()
{
  return robot_msgs::msg::builder::Init_ExecutorState_stamp();
}

}  // namespace robot_msgs

#endif  // ROBOT_MSGS__MSG__DETAIL__EXECUTOR_STATE__BUILDER_HPP_
