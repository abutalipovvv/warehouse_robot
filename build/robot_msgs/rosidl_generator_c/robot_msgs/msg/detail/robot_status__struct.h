// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from robot_msgs:msg/RobotStatus.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "robot_msgs/msg/robot_status.h"


#ifndef ROBOT_MSGS__MSG__DETAIL__ROBOT_STATUS__STRUCT_H_
#define ROBOT_MSGS__MSG__DETAIL__ROBOT_STATUS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"
// Member 'robot_id'
// Member 'map_id'
// Member 'state'
// Member 'message'
// Member 'target_lm'
// Member 'nearest_lm'
// Member 'current_edge_id'
// Member 'route_id'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/RobotStatus in the package robot_msgs.
/**
  * Robot state values are published as strings:
  * DISCONNECTED, LOCALIZING, IDLE, MANUAL, EXECUTING_ROUTE, ARRIVED, ERROR
 */
typedef struct robot_msgs__msg__RobotStatus
{
  builtin_interfaces__msg__Time stamp;
  rosidl_runtime_c__String robot_id;
  rosidl_runtime_c__String map_id;
  bool connected;
  bool localization_ok;
  float localization_age_sec;
  rosidl_runtime_c__String state;
  rosidl_runtime_c__String message;
  rosidl_runtime_c__String target_lm;
  rosidl_runtime_c__String nearest_lm;
  rosidl_runtime_c__String current_edge_id;
  rosidl_runtime_c__String route_id;
  float route_progress;
  float pose_x;
  float pose_y;
  float pose_yaw;
  float linear_velocity;
  float angular_velocity;
} robot_msgs__msg__RobotStatus;

// Struct for a sequence of robot_msgs__msg__RobotStatus.
typedef struct robot_msgs__msg__RobotStatus__Sequence
{
  robot_msgs__msg__RobotStatus * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robot_msgs__msg__RobotStatus__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROBOT_MSGS__MSG__DETAIL__ROBOT_STATUS__STRUCT_H_
