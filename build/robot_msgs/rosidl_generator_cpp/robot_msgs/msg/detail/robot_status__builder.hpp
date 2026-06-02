// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from robot_msgs:msg/RobotStatus.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "robot_msgs/msg/robot_status.hpp"


#ifndef ROBOT_MSGS__MSG__DETAIL__ROBOT_STATUS__BUILDER_HPP_
#define ROBOT_MSGS__MSG__DETAIL__ROBOT_STATUS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "robot_msgs/msg/detail/robot_status__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace robot_msgs
{

namespace msg
{

namespace builder
{

class Init_RobotStatus_angular_velocity
{
public:
  explicit Init_RobotStatus_angular_velocity(::robot_msgs::msg::RobotStatus & msg)
  : msg_(msg)
  {}
  ::robot_msgs::msg::RobotStatus angular_velocity(::robot_msgs::msg::RobotStatus::_angular_velocity_type arg)
  {
    msg_.angular_velocity = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robot_msgs::msg::RobotStatus msg_;
};

class Init_RobotStatus_linear_velocity
{
public:
  explicit Init_RobotStatus_linear_velocity(::robot_msgs::msg::RobotStatus & msg)
  : msg_(msg)
  {}
  Init_RobotStatus_angular_velocity linear_velocity(::robot_msgs::msg::RobotStatus::_linear_velocity_type arg)
  {
    msg_.linear_velocity = std::move(arg);
    return Init_RobotStatus_angular_velocity(msg_);
  }

private:
  ::robot_msgs::msg::RobotStatus msg_;
};

class Init_RobotStatus_pose_yaw
{
public:
  explicit Init_RobotStatus_pose_yaw(::robot_msgs::msg::RobotStatus & msg)
  : msg_(msg)
  {}
  Init_RobotStatus_linear_velocity pose_yaw(::robot_msgs::msg::RobotStatus::_pose_yaw_type arg)
  {
    msg_.pose_yaw = std::move(arg);
    return Init_RobotStatus_linear_velocity(msg_);
  }

private:
  ::robot_msgs::msg::RobotStatus msg_;
};

class Init_RobotStatus_pose_y
{
public:
  explicit Init_RobotStatus_pose_y(::robot_msgs::msg::RobotStatus & msg)
  : msg_(msg)
  {}
  Init_RobotStatus_pose_yaw pose_y(::robot_msgs::msg::RobotStatus::_pose_y_type arg)
  {
    msg_.pose_y = std::move(arg);
    return Init_RobotStatus_pose_yaw(msg_);
  }

private:
  ::robot_msgs::msg::RobotStatus msg_;
};

class Init_RobotStatus_pose_x
{
public:
  explicit Init_RobotStatus_pose_x(::robot_msgs::msg::RobotStatus & msg)
  : msg_(msg)
  {}
  Init_RobotStatus_pose_y pose_x(::robot_msgs::msg::RobotStatus::_pose_x_type arg)
  {
    msg_.pose_x = std::move(arg);
    return Init_RobotStatus_pose_y(msg_);
  }

private:
  ::robot_msgs::msg::RobotStatus msg_;
};

class Init_RobotStatus_route_progress
{
public:
  explicit Init_RobotStatus_route_progress(::robot_msgs::msg::RobotStatus & msg)
  : msg_(msg)
  {}
  Init_RobotStatus_pose_x route_progress(::robot_msgs::msg::RobotStatus::_route_progress_type arg)
  {
    msg_.route_progress = std::move(arg);
    return Init_RobotStatus_pose_x(msg_);
  }

private:
  ::robot_msgs::msg::RobotStatus msg_;
};

class Init_RobotStatus_route_id
{
public:
  explicit Init_RobotStatus_route_id(::robot_msgs::msg::RobotStatus & msg)
  : msg_(msg)
  {}
  Init_RobotStatus_route_progress route_id(::robot_msgs::msg::RobotStatus::_route_id_type arg)
  {
    msg_.route_id = std::move(arg);
    return Init_RobotStatus_route_progress(msg_);
  }

private:
  ::robot_msgs::msg::RobotStatus msg_;
};

class Init_RobotStatus_current_edge_id
{
public:
  explicit Init_RobotStatus_current_edge_id(::robot_msgs::msg::RobotStatus & msg)
  : msg_(msg)
  {}
  Init_RobotStatus_route_id current_edge_id(::robot_msgs::msg::RobotStatus::_current_edge_id_type arg)
  {
    msg_.current_edge_id = std::move(arg);
    return Init_RobotStatus_route_id(msg_);
  }

private:
  ::robot_msgs::msg::RobotStatus msg_;
};

class Init_RobotStatus_nearest_lm
{
public:
  explicit Init_RobotStatus_nearest_lm(::robot_msgs::msg::RobotStatus & msg)
  : msg_(msg)
  {}
  Init_RobotStatus_current_edge_id nearest_lm(::robot_msgs::msg::RobotStatus::_nearest_lm_type arg)
  {
    msg_.nearest_lm = std::move(arg);
    return Init_RobotStatus_current_edge_id(msg_);
  }

private:
  ::robot_msgs::msg::RobotStatus msg_;
};

class Init_RobotStatus_target_lm
{
public:
  explicit Init_RobotStatus_target_lm(::robot_msgs::msg::RobotStatus & msg)
  : msg_(msg)
  {}
  Init_RobotStatus_nearest_lm target_lm(::robot_msgs::msg::RobotStatus::_target_lm_type arg)
  {
    msg_.target_lm = std::move(arg);
    return Init_RobotStatus_nearest_lm(msg_);
  }

private:
  ::robot_msgs::msg::RobotStatus msg_;
};

class Init_RobotStatus_message
{
public:
  explicit Init_RobotStatus_message(::robot_msgs::msg::RobotStatus & msg)
  : msg_(msg)
  {}
  Init_RobotStatus_target_lm message(::robot_msgs::msg::RobotStatus::_message_type arg)
  {
    msg_.message = std::move(arg);
    return Init_RobotStatus_target_lm(msg_);
  }

private:
  ::robot_msgs::msg::RobotStatus msg_;
};

class Init_RobotStatus_state
{
public:
  explicit Init_RobotStatus_state(::robot_msgs::msg::RobotStatus & msg)
  : msg_(msg)
  {}
  Init_RobotStatus_message state(::robot_msgs::msg::RobotStatus::_state_type arg)
  {
    msg_.state = std::move(arg);
    return Init_RobotStatus_message(msg_);
  }

private:
  ::robot_msgs::msg::RobotStatus msg_;
};

class Init_RobotStatus_localization_age_sec
{
public:
  explicit Init_RobotStatus_localization_age_sec(::robot_msgs::msg::RobotStatus & msg)
  : msg_(msg)
  {}
  Init_RobotStatus_state localization_age_sec(::robot_msgs::msg::RobotStatus::_localization_age_sec_type arg)
  {
    msg_.localization_age_sec = std::move(arg);
    return Init_RobotStatus_state(msg_);
  }

private:
  ::robot_msgs::msg::RobotStatus msg_;
};

class Init_RobotStatus_localization_ok
{
public:
  explicit Init_RobotStatus_localization_ok(::robot_msgs::msg::RobotStatus & msg)
  : msg_(msg)
  {}
  Init_RobotStatus_localization_age_sec localization_ok(::robot_msgs::msg::RobotStatus::_localization_ok_type arg)
  {
    msg_.localization_ok = std::move(arg);
    return Init_RobotStatus_localization_age_sec(msg_);
  }

private:
  ::robot_msgs::msg::RobotStatus msg_;
};

class Init_RobotStatus_connected
{
public:
  explicit Init_RobotStatus_connected(::robot_msgs::msg::RobotStatus & msg)
  : msg_(msg)
  {}
  Init_RobotStatus_localization_ok connected(::robot_msgs::msg::RobotStatus::_connected_type arg)
  {
    msg_.connected = std::move(arg);
    return Init_RobotStatus_localization_ok(msg_);
  }

private:
  ::robot_msgs::msg::RobotStatus msg_;
};

class Init_RobotStatus_map_id
{
public:
  explicit Init_RobotStatus_map_id(::robot_msgs::msg::RobotStatus & msg)
  : msg_(msg)
  {}
  Init_RobotStatus_connected map_id(::robot_msgs::msg::RobotStatus::_map_id_type arg)
  {
    msg_.map_id = std::move(arg);
    return Init_RobotStatus_connected(msg_);
  }

private:
  ::robot_msgs::msg::RobotStatus msg_;
};

class Init_RobotStatus_robot_id
{
public:
  explicit Init_RobotStatus_robot_id(::robot_msgs::msg::RobotStatus & msg)
  : msg_(msg)
  {}
  Init_RobotStatus_map_id robot_id(::robot_msgs::msg::RobotStatus::_robot_id_type arg)
  {
    msg_.robot_id = std::move(arg);
    return Init_RobotStatus_map_id(msg_);
  }

private:
  ::robot_msgs::msg::RobotStatus msg_;
};

class Init_RobotStatus_stamp
{
public:
  Init_RobotStatus_stamp()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RobotStatus_robot_id stamp(::robot_msgs::msg::RobotStatus::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return Init_RobotStatus_robot_id(msg_);
  }

private:
  ::robot_msgs::msg::RobotStatus msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::robot_msgs::msg::RobotStatus>()
{
  return robot_msgs::msg::builder::Init_RobotStatus_stamp();
}

}  // namespace robot_msgs

#endif  // ROBOT_MSGS__MSG__DETAIL__ROBOT_STATUS__BUILDER_HPP_
