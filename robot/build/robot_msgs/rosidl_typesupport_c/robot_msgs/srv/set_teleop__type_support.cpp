// generated from rosidl_typesupport_c/resource/idl__type_support.cpp.em
// with input from robot_msgs:srv/SetTeleop.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "robot_msgs/srv/detail/set_teleop__struct.h"
#include "robot_msgs/srv/detail/set_teleop__type_support.h"
#include "robot_msgs/srv/detail/set_teleop__functions.h"
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

typedef struct _SetTeleop_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _SetTeleop_Request_type_support_ids_t;

static const _SetTeleop_Request_type_support_ids_t _SetTeleop_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _SetTeleop_Request_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _SetTeleop_Request_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _SetTeleop_Request_type_support_symbol_names_t _SetTeleop_Request_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, robot_msgs, srv, SetTeleop_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robot_msgs, srv, SetTeleop_Request)),
  }
};

typedef struct _SetTeleop_Request_type_support_data_t
{
  void * data[2];
} _SetTeleop_Request_type_support_data_t;

static _SetTeleop_Request_type_support_data_t _SetTeleop_Request_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _SetTeleop_Request_message_typesupport_map = {
  2,
  "robot_msgs",
  &_SetTeleop_Request_message_typesupport_ids.typesupport_identifier[0],
  &_SetTeleop_Request_message_typesupport_symbol_names.symbol_name[0],
  &_SetTeleop_Request_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t SetTeleop_Request_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_SetTeleop_Request_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
  &robot_msgs__srv__SetTeleop_Request__get_type_hash,
  &robot_msgs__srv__SetTeleop_Request__get_type_description,
  &robot_msgs__srv__SetTeleop_Request__get_type_description_sources,
};

}  // namespace rosidl_typesupport_c

}  // namespace srv

}  // namespace robot_msgs

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, robot_msgs, srv, SetTeleop_Request)() {
  return &::robot_msgs::srv::rosidl_typesupport_c::SetTeleop_Request_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "robot_msgs/srv/detail/set_teleop__struct.h"
// already included above
// #include "robot_msgs/srv/detail/set_teleop__type_support.h"
// already included above
// #include "robot_msgs/srv/detail/set_teleop__functions.h"
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

typedef struct _SetTeleop_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _SetTeleop_Response_type_support_ids_t;

static const _SetTeleop_Response_type_support_ids_t _SetTeleop_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _SetTeleop_Response_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _SetTeleop_Response_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _SetTeleop_Response_type_support_symbol_names_t _SetTeleop_Response_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, robot_msgs, srv, SetTeleop_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robot_msgs, srv, SetTeleop_Response)),
  }
};

typedef struct _SetTeleop_Response_type_support_data_t
{
  void * data[2];
} _SetTeleop_Response_type_support_data_t;

static _SetTeleop_Response_type_support_data_t _SetTeleop_Response_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _SetTeleop_Response_message_typesupport_map = {
  2,
  "robot_msgs",
  &_SetTeleop_Response_message_typesupport_ids.typesupport_identifier[0],
  &_SetTeleop_Response_message_typesupport_symbol_names.symbol_name[0],
  &_SetTeleop_Response_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t SetTeleop_Response_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_SetTeleop_Response_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
  &robot_msgs__srv__SetTeleop_Response__get_type_hash,
  &robot_msgs__srv__SetTeleop_Response__get_type_description,
  &robot_msgs__srv__SetTeleop_Response__get_type_description_sources,
};

}  // namespace rosidl_typesupport_c

}  // namespace srv

}  // namespace robot_msgs

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, robot_msgs, srv, SetTeleop_Response)() {
  return &::robot_msgs::srv::rosidl_typesupport_c::SetTeleop_Response_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "robot_msgs/srv/detail/set_teleop__struct.h"
// already included above
// #include "robot_msgs/srv/detail/set_teleop__type_support.h"
// already included above
// #include "robot_msgs/srv/detail/set_teleop__functions.h"
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

typedef struct _SetTeleop_Event_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _SetTeleop_Event_type_support_ids_t;

static const _SetTeleop_Event_type_support_ids_t _SetTeleop_Event_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _SetTeleop_Event_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _SetTeleop_Event_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _SetTeleop_Event_type_support_symbol_names_t _SetTeleop_Event_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, robot_msgs, srv, SetTeleop_Event)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robot_msgs, srv, SetTeleop_Event)),
  }
};

typedef struct _SetTeleop_Event_type_support_data_t
{
  void * data[2];
} _SetTeleop_Event_type_support_data_t;

static _SetTeleop_Event_type_support_data_t _SetTeleop_Event_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _SetTeleop_Event_message_typesupport_map = {
  2,
  "robot_msgs",
  &_SetTeleop_Event_message_typesupport_ids.typesupport_identifier[0],
  &_SetTeleop_Event_message_typesupport_symbol_names.symbol_name[0],
  &_SetTeleop_Event_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t SetTeleop_Event_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_SetTeleop_Event_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
  &robot_msgs__srv__SetTeleop_Event__get_type_hash,
  &robot_msgs__srv__SetTeleop_Event__get_type_description,
  &robot_msgs__srv__SetTeleop_Event__get_type_description_sources,
};

}  // namespace rosidl_typesupport_c

}  // namespace srv

}  // namespace robot_msgs

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, robot_msgs, srv, SetTeleop_Event)() {
  return &::robot_msgs::srv::rosidl_typesupport_c::SetTeleop_Event_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "robot_msgs/srv/detail/set_teleop__type_support.h"
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
typedef struct _SetTeleop_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _SetTeleop_type_support_ids_t;

static const _SetTeleop_type_support_ids_t _SetTeleop_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _SetTeleop_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _SetTeleop_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _SetTeleop_type_support_symbol_names_t _SetTeleop_service_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, robot_msgs, srv, SetTeleop)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robot_msgs, srv, SetTeleop)),
  }
};

typedef struct _SetTeleop_type_support_data_t
{
  void * data[2];
} _SetTeleop_type_support_data_t;

static _SetTeleop_type_support_data_t _SetTeleop_service_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _SetTeleop_service_typesupport_map = {
  2,
  "robot_msgs",
  &_SetTeleop_service_typesupport_ids.typesupport_identifier[0],
  &_SetTeleop_service_typesupport_symbol_names.symbol_name[0],
  &_SetTeleop_service_typesupport_data.data[0],
};

static const rosidl_service_type_support_t SetTeleop_service_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_SetTeleop_service_typesupport_map),
  rosidl_typesupport_c__get_service_typesupport_handle_function,
  &SetTeleop_Request_message_type_support_handle,
  &SetTeleop_Response_message_type_support_handle,
  &SetTeleop_Event_message_type_support_handle,
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_CREATE_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    robot_msgs,
    srv,
    SetTeleop
  ),
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_DESTROY_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    robot_msgs,
    srv,
    SetTeleop
  ),
  &robot_msgs__srv__SetTeleop__get_type_hash,
  &robot_msgs__srv__SetTeleop__get_type_description,
  &robot_msgs__srv__SetTeleop__get_type_description_sources,
};

}  // namespace rosidl_typesupport_c

}  // namespace srv

}  // namespace robot_msgs

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_c, robot_msgs, srv, SetTeleop)() {
  return &::robot_msgs::srv::rosidl_typesupport_c::SetTeleop_service_type_support_handle;
}

#ifdef __cplusplus
}
#endif
