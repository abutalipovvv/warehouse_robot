// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from robot_msgs:srv/PlanRoute.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "robot_msgs/srv/plan_route.hpp"


#ifndef ROBOT_MSGS__SRV__DETAIL__PLAN_ROUTE__STRUCT_HPP_
#define ROBOT_MSGS__SRV__DETAIL__PLAN_ROUTE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__robot_msgs__srv__PlanRoute_Request __attribute__((deprecated))
#else
# define DEPRECATED__robot_msgs__srv__PlanRoute_Request __declspec(deprecated)
#endif

namespace robot_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct PlanRoute_Request_
{
  using Type = PlanRoute_Request_<ContainerAllocator>;

  explicit PlanRoute_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->goal_lm = "";
      this->start_lm = "";
      this->use_start_pose = false;
      this->start_x = 0.0;
      this->start_y = 0.0;
      this->start_yaw = 0.0;
    }
  }

  explicit PlanRoute_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_lm(_alloc),
    start_lm(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->goal_lm = "";
      this->start_lm = "";
      this->use_start_pose = false;
      this->start_x = 0.0;
      this->start_y = 0.0;
      this->start_yaw = 0.0;
    }
  }

  // field types and members
  using _goal_lm_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _goal_lm_type goal_lm;
  using _start_lm_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _start_lm_type start_lm;
  using _use_start_pose_type =
    bool;
  _use_start_pose_type use_start_pose;
  using _start_x_type =
    double;
  _start_x_type start_x;
  using _start_y_type =
    double;
  _start_y_type start_y;
  using _start_yaw_type =
    double;
  _start_yaw_type start_yaw;

  // setters for named parameter idiom
  Type & set__goal_lm(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->goal_lm = _arg;
    return *this;
  }
  Type & set__start_lm(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->start_lm = _arg;
    return *this;
  }
  Type & set__use_start_pose(
    const bool & _arg)
  {
    this->use_start_pose = _arg;
    return *this;
  }
  Type & set__start_x(
    const double & _arg)
  {
    this->start_x = _arg;
    return *this;
  }
  Type & set__start_y(
    const double & _arg)
  {
    this->start_y = _arg;
    return *this;
  }
  Type & set__start_yaw(
    const double & _arg)
  {
    this->start_yaw = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    robot_msgs::srv::PlanRoute_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const robot_msgs::srv::PlanRoute_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<robot_msgs::srv::PlanRoute_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<robot_msgs::srv::PlanRoute_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      robot_msgs::srv::PlanRoute_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<robot_msgs::srv::PlanRoute_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      robot_msgs::srv::PlanRoute_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<robot_msgs::srv::PlanRoute_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<robot_msgs::srv::PlanRoute_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<robot_msgs::srv::PlanRoute_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__robot_msgs__srv__PlanRoute_Request
    std::shared_ptr<robot_msgs::srv::PlanRoute_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__robot_msgs__srv__PlanRoute_Request
    std::shared_ptr<robot_msgs::srv::PlanRoute_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PlanRoute_Request_ & other) const
  {
    if (this->goal_lm != other.goal_lm) {
      return false;
    }
    if (this->start_lm != other.start_lm) {
      return false;
    }
    if (this->use_start_pose != other.use_start_pose) {
      return false;
    }
    if (this->start_x != other.start_x) {
      return false;
    }
    if (this->start_y != other.start_y) {
      return false;
    }
    if (this->start_yaw != other.start_yaw) {
      return false;
    }
    return true;
  }
  bool operator!=(const PlanRoute_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PlanRoute_Request_

// alias to use template instance with default allocator
using PlanRoute_Request =
  robot_msgs::srv::PlanRoute_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace robot_msgs


#ifndef _WIN32
# define DEPRECATED__robot_msgs__srv__PlanRoute_Response __attribute__((deprecated))
#else
# define DEPRECATED__robot_msgs__srv__PlanRoute_Response __declspec(deprecated)
#endif

namespace robot_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct PlanRoute_Response_
{
  using Type = PlanRoute_Response_<ContainerAllocator>;

  explicit PlanRoute_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->ok = false;
      this->error = "";
      this->route_json = "";
    }
  }

  explicit PlanRoute_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : error(_alloc),
    route_json(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->ok = false;
      this->error = "";
      this->route_json = "";
    }
  }

  // field types and members
  using _ok_type =
    bool;
  _ok_type ok;
  using _error_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _error_type error;
  using _route_json_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _route_json_type route_json;

  // setters for named parameter idiom
  Type & set__ok(
    const bool & _arg)
  {
    this->ok = _arg;
    return *this;
  }
  Type & set__error(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->error = _arg;
    return *this;
  }
  Type & set__route_json(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->route_json = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    robot_msgs::srv::PlanRoute_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const robot_msgs::srv::PlanRoute_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<robot_msgs::srv::PlanRoute_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<robot_msgs::srv::PlanRoute_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      robot_msgs::srv::PlanRoute_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<robot_msgs::srv::PlanRoute_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      robot_msgs::srv::PlanRoute_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<robot_msgs::srv::PlanRoute_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<robot_msgs::srv::PlanRoute_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<robot_msgs::srv::PlanRoute_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__robot_msgs__srv__PlanRoute_Response
    std::shared_ptr<robot_msgs::srv::PlanRoute_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__robot_msgs__srv__PlanRoute_Response
    std::shared_ptr<robot_msgs::srv::PlanRoute_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PlanRoute_Response_ & other) const
  {
    if (this->ok != other.ok) {
      return false;
    }
    if (this->error != other.error) {
      return false;
    }
    if (this->route_json != other.route_json) {
      return false;
    }
    return true;
  }
  bool operator!=(const PlanRoute_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PlanRoute_Response_

// alias to use template instance with default allocator
using PlanRoute_Response =
  robot_msgs::srv::PlanRoute_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace robot_msgs


// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__robot_msgs__srv__PlanRoute_Event __attribute__((deprecated))
#else
# define DEPRECATED__robot_msgs__srv__PlanRoute_Event __declspec(deprecated)
#endif

namespace robot_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct PlanRoute_Event_
{
  using Type = PlanRoute_Event_<ContainerAllocator>;

  explicit PlanRoute_Event_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_init)
  {
    (void)_init;
  }

  explicit PlanRoute_Event_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _info_type =
    service_msgs::msg::ServiceEventInfo_<ContainerAllocator>;
  _info_type info;
  using _request_type =
    rosidl_runtime_cpp::BoundedVector<robot_msgs::srv::PlanRoute_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<robot_msgs::srv::PlanRoute_Request_<ContainerAllocator>>>;
  _request_type request;
  using _response_type =
    rosidl_runtime_cpp::BoundedVector<robot_msgs::srv::PlanRoute_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<robot_msgs::srv::PlanRoute_Response_<ContainerAllocator>>>;
  _response_type response;

  // setters for named parameter idiom
  Type & set__info(
    const service_msgs::msg::ServiceEventInfo_<ContainerAllocator> & _arg)
  {
    this->info = _arg;
    return *this;
  }
  Type & set__request(
    const rosidl_runtime_cpp::BoundedVector<robot_msgs::srv::PlanRoute_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<robot_msgs::srv::PlanRoute_Request_<ContainerAllocator>>> & _arg)
  {
    this->request = _arg;
    return *this;
  }
  Type & set__response(
    const rosidl_runtime_cpp::BoundedVector<robot_msgs::srv::PlanRoute_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<robot_msgs::srv::PlanRoute_Response_<ContainerAllocator>>> & _arg)
  {
    this->response = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    robot_msgs::srv::PlanRoute_Event_<ContainerAllocator> *;
  using ConstRawPtr =
    const robot_msgs::srv::PlanRoute_Event_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<robot_msgs::srv::PlanRoute_Event_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<robot_msgs::srv::PlanRoute_Event_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      robot_msgs::srv::PlanRoute_Event_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<robot_msgs::srv::PlanRoute_Event_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      robot_msgs::srv::PlanRoute_Event_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<robot_msgs::srv::PlanRoute_Event_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<robot_msgs::srv::PlanRoute_Event_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<robot_msgs::srv::PlanRoute_Event_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__robot_msgs__srv__PlanRoute_Event
    std::shared_ptr<robot_msgs::srv::PlanRoute_Event_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__robot_msgs__srv__PlanRoute_Event
    std::shared_ptr<robot_msgs::srv::PlanRoute_Event_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PlanRoute_Event_ & other) const
  {
    if (this->info != other.info) {
      return false;
    }
    if (this->request != other.request) {
      return false;
    }
    if (this->response != other.response) {
      return false;
    }
    return true;
  }
  bool operator!=(const PlanRoute_Event_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PlanRoute_Event_

// alias to use template instance with default allocator
using PlanRoute_Event =
  robot_msgs::srv::PlanRoute_Event_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace robot_msgs

namespace robot_msgs
{

namespace srv
{

struct PlanRoute
{
  using Request = robot_msgs::srv::PlanRoute_Request;
  using Response = robot_msgs::srv::PlanRoute_Response;
  using Event = robot_msgs::srv::PlanRoute_Event;
};

}  // namespace srv

}  // namespace robot_msgs

#endif  // ROBOT_MSGS__SRV__DETAIL__PLAN_ROUTE__STRUCT_HPP_
