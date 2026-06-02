// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from robot_msgs:msg/ExecutorState.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "robot_msgs/msg/executor_state.h"


#ifndef ROBOT_MSGS__MSG__DETAIL__EXECUTOR_STATE__STRUCT_H_
#define ROBOT_MSGS__MSG__DETAIL__EXECUTOR_STATE__STRUCT_H_

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
// Member 'current_edge_id'
// Member 'route_id'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/ExecutorState in the package robot_msgs.
typedef struct robot_msgs__msg__ExecutorState
{
  builtin_interfaces__msg__Time stamp;
  rosidl_runtime_c__String robot_id;
  rosidl_runtime_c__String map_id;
  bool route_active;
  rosidl_runtime_c__String state;
  rosidl_runtime_c__String message;
  rosidl_runtime_c__String target_lm;
  rosidl_runtime_c__String current_edge_id;
  rosidl_runtime_c__String route_id;
  float route_progress;
} robot_msgs__msg__ExecutorState;

// Struct for a sequence of robot_msgs__msg__ExecutorState.
typedef struct robot_msgs__msg__ExecutorState__Sequence
{
  robot_msgs__msg__ExecutorState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robot_msgs__msg__ExecutorState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROBOT_MSGS__MSG__DETAIL__EXECUTOR_STATE__STRUCT_H_
