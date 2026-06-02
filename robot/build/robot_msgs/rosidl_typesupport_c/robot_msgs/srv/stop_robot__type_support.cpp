// generated from rosidl_typesupport_c/resource/idl__type_support.cpp.em
// with input from robot_msgs:srv/StopRobot.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "robot_msgs/srv/detail/stop_robot__struct.h"
#include "robot_msgs/srv/detail/stop_robot__type_support.h"
#include "robot_msgs/srv/detail/stop_robot__functions.h"
#include "rosidl_typesupport_c/identifier.h"
#include "rosidl_typesupport_c/message_type_support_dispatch.h"
#include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_c/visibility_control.h"
#include "rosidl_typesupport_interface/macros.h"

namespace robot_msgs
{

namespace srv
{

namespace rosidl_typesupport_c
{

typedef struct _StopRobot_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _StopRobot_Request_type_support_ids_t;

static const _StopRobot_Request_type_support_ids_t _StopRobot_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _StopRobot_Request_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _StopRobot_Request_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _StopRobot_Request_type_support_symbol_names_t _StopRobot_Request_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, robot_msgs, srv, StopRobot_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robot_msgs, srv, StopRobot_Request)),
  }
};

typedef struct _StopRobot_Request_type_support_data_t
{
  void * data[2];
} _StopRobot_Request_type_support_data_t;

static _StopRobot_Request_type_support_data_t _StopRobot_Request_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _StopRobot_Request_message_typesupport_map = {
  2,
  "robot_msgs",
  &_StopRobot_Request_message_typesupport_ids.typesupport_identifier[0],
  &_StopRobot_Request_message_typesupport_symbol_names.symbol_name[0],
  &_StopRobot_Request_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t StopRobot_Request_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_StopRobot_Request_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
  &robot_msgs__srv__StopRobot_Request__get_type_hash,
  &robot_msgs__srv__StopRobot_Request__get_type_description,
  &robot_msgs__srv__StopRobot_Request__get_type_description_sources,
};

}  // namespace rosidl_typesupport_c

}  // namespace srv

}  // namespace robot_msgs

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, robot_msgs, srv, StopRobot_Request)() {
  return &::robot_msgs::srv::rosidl_typesupport_c::StopRobot_Request_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "robot_msgs/srv/detail/stop_robot__struct.h"
// already included above
// #include "robot_msgs/srv/detail/stop_robot__type_support.h"
// already included above
// #include "robot_msgs/srv/detail/stop_robot__functions.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/message_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_c/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace robot_msgs
{

namespace srv
{

namespace rosidl_typesupport_c
{

typedef struct _StopRobot_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _StopRobot_Response_type_support_ids_t;

static const _StopRobot_Response_type_support_ids_t _StopRobot_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _StopRobot_Response_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _StopRobot_Response_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _StopRobot_Response_type_support_symbol_names_t _StopRobot_Response_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, robot_msgs, srv, StopRobot_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robot_msgs, srv, StopRobot_Response)),
  }
};

typedef struct _StopRobot_Response_type_support_data_t
{
  void * data[2];
} _StopRobot_Response_type_support_data_t;

static _StopRobot_Response_type_support_data_t _StopRobot_Response_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _StopRobot_Response_message_typesupport_map = {
  2,
  "robot_msgs",
  &_StopRobot_Response_message_typesupport_ids.typesupport_identifier[0],
  &_StopRobot_Response_message_typesupport_symbol_names.symbol_name[0],
  &_StopRobot_Response_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t StopRobot_Response_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_StopRobot_Response_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
  &robot_msgs__srv__StopRobot_Response__get_type_hash,
  &robot_msgs__srv__StopRobot_Response__get_type_description,
  &robot_msgs__srv__StopRobot_Response__get_type_description_sources,
};

}  // namespace rosidl_typesupport_c

}  // namespace srv

}  // namespace robot_msgs

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, robot_msgs, srv, StopRobot_Response)() {
  return &::robot_msgs::srv::rosidl_typesupport_c::StopRobot_Response_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "robot_msgs/srv/detail/stop_robot__struct.h"
// already included above
// #include "robot_msgs/srv/detail/stop_robot__type_support.h"
// already included above
// #include "robot_msgs/srv/detail/stop_robot__functions.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/message_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_c/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace robot_msgs
{

namespace srv
{

namespace rosidl_typesupport_c
{

typedef struct _StopRobot_Event_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _StopRobot_Event_type_support_ids_t;

static const _StopRobot_Event_type_support_ids_t _StopRobot_Event_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _StopRobot_Event_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _StopRobot_Event_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _StopRobot_Event_type_support_symbol_names_t _StopRobot_Event_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, robot_msgs, srv, StopRobot_Event)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robot_msgs, srv, StopRobot_Event)),
  }
};

typedef struct _StopRobot_Event_type_support_data_t
{
  void * data[2];
} _StopRobot_Event_type_support_data_t;

static _StopRobot_Event_type_support_data_t _StopRobot_Event_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _StopRobot_Event_message_typesupport_map = {
  2,
  "robot_msgs",
  &_StopRobot_Event_message_typesupport_ids.typesupport_identifier[0],
  &_StopRobot_Event_message_typesupport_symbol_names.symbol_name[0],
  &_StopRobot_Event_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t StopRobot_Event_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_StopRobot_Event_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
  &robot_msgs__srv__StopRobot_Event__get_type_hash,
  &robot_msgs__srv__StopRobot_Event__get_type_description,
  &robot_msgs__srv__StopRobot_Event__get_type_description_sources,
};

}  // namespace rosidl_typesupport_c

}  // namespace srv

}  // namespace robot_msgs

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, robot_msgs, srv, StopRobot_Event)() {
  return &::robot_msgs::srv::rosidl_typesupport_c::StopRobot_Event_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "robot_msgs/srv/detail/stop_robot__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
#include "rosidl_typesupport_c/service_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
#include "service_msgs/msg/service_event_info.h"
#include "builtin_interfaces/msg/time.h"

namespace robot_msgs
{

namespace srv
{

namespace rosidl_typesupport_c
{
typedef struct _StopRobot_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _StopRobot_type_support_ids_t;

static const _StopRobot_type_support_ids_t _StopRobot_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _StopRobot_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _StopRobot_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _StopRobot_type_support_symbol_names_t _StopRobot_service_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, robot_msgs, srv, StopRobot)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robot_msgs, srv, StopRobot)),
  }
};

typedef struct _StopRobot_type_support_data_t
{
  void * data[2];
} _StopRobot_type_support_data_t;

static _StopRobot_type_support_data_t _StopRobot_service_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _StopRobot_service_typesupport_map = {
  2,
  "robot_msgs",
  &_StopRobot_service_typesupport_ids.typesupport_identifier[0],
  &_StopRobot_service_typesupport_symbol_names.symbol_name[0],
  &_StopRobot_service_typesupport_data.data[0],
};

static const rosidl_service_type_support_t StopRobot_service_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_StopRobot_service_typesupport_map),
  rosidl_typesupport_c__get_service_typesupport_handle_function,
  &StopRobot_Request_message_type_support_handle,
  &StopRobot_Response_message_type_support_handle,
  &StopRobot_Event_message_type_support_handle,
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_CREATE_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    robot_msgs,
    srv,
    StopRobot
  ),
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_DESTROY_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    robot_msgs,
    srv,
    StopRobot
  ),
  &robot_msgs__srv__StopRobot__get_type_hash,
  &robot_msgs__srv__StopRobot__get_type_description,
  &robot_msgs__srv__StopRobot__get_type_description_sources,
};

}  // namespace rosidl_typesupport_c

}  // namespace srv

}  // namespace robot_msgs

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_c, robot_msgs, srv, StopRobot)() {
  return &::robot_msgs::srv::rosidl_typesupport_c::StopRobot_service_type_support_handle;
}

#ifdef __cplusplus
}
#endif
