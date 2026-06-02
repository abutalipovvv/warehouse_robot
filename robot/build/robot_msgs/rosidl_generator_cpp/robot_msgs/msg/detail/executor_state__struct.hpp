// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from robot_msgs:msg/ExecutorState.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "robot_msgs/msg/executor_state.hpp"


#ifndef ROBOT_MSGS__MSG__DETAIL__EXECUTOR_STATE__STRUCT_HPP_
#define ROBOT_MSGS__MSG__DETAIL__EXECUTOR_STATE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__robot_msgs__msg__ExecutorState __attribute__((deprecated))
#else
# define DEPRECATED__robot_msgs__msg__ExecutorState __declspec(deprecated)
#endif

namespace robot_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ExecutorState_
{
  using Type = ExecutorState_<ContainerAllocator>;

  explicit ExecutorState_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->robot_id = "";
      this->map_id = "";
      this->route_active = false;
      this->state = "";
      this->message = "";
      this->target_lm = "";
      this->current_edge_id = "";
      this->route_id = "";
      this->route_progress = 0.0f;
    }
  }

  explicit ExecutorState_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_alloc, _init),
    robot_id(_alloc),
    map_id(_alloc),
    state(_alloc),
    message(_alloc),
    target_lm(_alloc),
    current_edge_id(_alloc),
    route_id(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->robot_id = "";
      this->map_id = "";
      this->route_active = false;
      this->state = "";
      this->message = "";
      this->target_lm = "";
      this->current_edge_id = "";
      this->route_id = "";
      this->route_progress = 0.0f;
    }
  }

  // field types and members
  using _stamp_type =
    builtin_interfaces::msg::Time_<ContainerAllocator>;
  _stamp_type stamp;
  using _robot_id_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _robot_id_type robot_id;
  using _map_id_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _map_id_type map_id;
  using _route_active_type =
    bool;
  _route_active_type route_active;
  using _state_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _state_type state;
  using _message_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _message_type message;
  using _target_lm_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _target_lm_type target_lm;
  using _current_edge_id_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _current_edge_id_type current_edge_id;
  using _route_id_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _route_id_type route_id;
  using _route_progress_type =
    float;
  _route_progress_type route_progress;

  // setters for named parameter idiom
  Type & set__stamp(
    const builtin_interfaces::msg::Time_<ContainerAllocator> & _arg)
  {
    this->stamp = _arg;
    return *this;
  }
  Type & set__robot_id(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->robot_id = _arg;
    return *this;
  }
  Type & set__map_id(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->map_id = _arg;
    return *this;
  }
  Type & set__route_active(
    const bool & _arg)
  {
    this->route_active = _arg;
    return *this;
  }
  Type & set__state(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->state = _arg;
    return *this;
  }
  Type & set__message(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->message = _arg;
    return *this;
  }
  Type & set__target_lm(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->target_lm = _arg;
    return *this;
  }
  Type & set__current_edge_id(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->current_edge_id = _arg;
    return *this;
  }
  Type & set__route_id(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->route_id = _arg;
    return *this;
  }
  Type & set__route_progress(
    const float & _arg)
  {
    this->route_progress = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    robot_msgs::msg::ExecutorState_<ContainerAllocator> *;
  using ConstRawPtr =
    const robot_msgs::msg::ExecutorState_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<robot_msgs::msg::ExecutorState_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<robot_msgs::msg::ExecutorState_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      robot_msgs::msg::ExecutorState_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<robot_msgs::msg::ExecutorState_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      robot_msgs::msg::ExecutorState_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<robot_msgs::msg::ExecutorState_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<robot_msgs::msg::ExecutorState_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<robot_msgs::msg::ExecutorState_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__robot_msgs__msg__ExecutorState
    std::shared_ptr<robot_msgs::msg::ExecutorState_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__robot_msgs__msg__ExecutorState
    std::shared_ptr<robot_msgs::msg::ExecutorState_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ExecutorState_ & other) const
  {
    if (this->stamp != other.stamp) {
      return false;
    }
    if (this->robot_id != other.robot_id) {
      return false;
    }
    if (this->map_id != other.map_id) {
      return false;
    }
    if (this->route_active != other.route_active) {
      return false;
    }
    if (this->state != other.state) {
      return false;
    }
    if (this->message != other.message) {
      return false;
    }
    if (this->target_lm != other.target_lm) {
      return false;
    }
    if (this->current_edge_id != other.current_edge_id) {
      return false;
    }
    if (this->route_id != other.route_id) {
      return false;
    }
    if (this->route_progress != other.route_progress) {
      return false;
    }
    return true;
  }
  bool operator!=(const ExecutorState_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ExecutorState_

// alias to use template instance with default allocator
using ExecutorState =
  robot_msgs::msg::ExecutorState_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace robot_msgs

#endif  // ROBOT_MSGS__MSG__DETAIL__EXECUTOR_STATE__STRUCT_HPP_
