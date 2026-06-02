// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from robot_msgs:srv/PlanRoute.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "robot_msgs/srv/plan_route.h"


#ifndef ROBOT_MSGS__SRV__DETAIL__PLAN_ROUTE__STRUCT_H_
#define ROBOT_MSGS__SRV__DETAIL__PLAN_ROUTE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'goal_lm'
// Member 'start_lm'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/PlanRoute in the package robot_msgs.
typedef struct robot_msgs__srv__PlanRoute_Request
{
  rosidl_runtime_c__String goal_lm;
  rosidl_runtime_c__String start_lm;
  bool use_start_pose;
  double start_x;
  double start_y;
  double start_yaw;
} robot_msgs__srv__PlanRoute_Request;

// Struct for a sequence of robot_msgs__srv__PlanRoute_Request.
typedef struct robot_msgs__srv__PlanRoute_Request__Sequence
{
  robot_msgs__srv__PlanRoute_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robot_msgs__srv__PlanRoute_Request__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'error'
// Member 'route_json'
// already included above
// #include "rosidl_runtime_c/string.h"

/// Struct defined in srv/PlanRoute in the package robot_msgs.
typedef struct robot_msgs__srv__PlanRoute_Response
{
  bool ok;
  rosidl_runtime_c__String error;
  rosidl_runtime_c__String route_json;
} robot_msgs__srv__PlanRoute_Response;

// Struct for a sequence of robot_msgs__srv__PlanRoute_Response.
typedef struct robot_msgs__srv__PlanRoute_Response__Sequence
{
  robot_msgs__srv__PlanRoute_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robot_msgs__srv__PlanRoute_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  robot_msgs__srv__PlanRoute_Event__request__MAX_SIZE = 1
};
// response
enum
{
  robot_msgs__srv__PlanRoute_Event__response__MAX_SIZE = 1
};

/// Struct defined in srv/PlanRoute in the package robot_msgs.
typedef struct robot_msgs__srv__PlanRoute_Event
{
  service_msgs__msg__ServiceEventInfo info;
  robot_msgs__srv__PlanRoute_Request__Sequence request;
  robot_msgs__srv__PlanRoute_Response__Sequence response;
} robot_msgs__srv__PlanRoute_Event;

// Struct for a sequence of robot_msgs__srv__PlanRoute_Event.
typedef struct robot_msgs__srv__PlanRoute_Event__Sequence
{
  robot_msgs__srv__PlanRoute_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robot_msgs__srv__PlanRoute_Event__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROBOT_MSGS__SRV__DETAIL__PLAN_ROUTE__STRUCT_H_
