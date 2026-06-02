// generated from rosidl_typesupport_c/resource/idl__type_support.cpp.em
// with input from robot_msgs:srv/CancelRoute.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "robot_msgs/srv/detail/cancel_route__struct.h"
#include "robot_msgs/srv/detail/cancel_route__type_support.h"
#include "robot_msgs/srv/detail/cancel_route__functions.h"
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

typedef struct _CancelRoute_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _CancelRoute_Request_type_support_ids_t;

static const _CancelRoute_Request_type_support_ids_t _CancelRoute_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _CancelRoute_Request_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _CancelRoute_Request_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _CancelRoute_Request_type_support_symbol_names_t _CancelRoute_Request_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, robot_msgs, srv, CancelRoute_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robot_msgs, srv, CancelRoute_Request)),
  }
};

typedef struct _CancelRoute_Request_type_support_data_t
{
  void * data[2];
} _CancelRoute_Request_type_support_data_t;

static _CancelRoute_Request_type_support_data_t _CancelRoute_Request_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _CancelRoute_Request_message_typesupport_map = {
  2,
  "robot_msgs",
  &_CancelRoute_Request_message_typesupport_ids.typesupport_identifier[0],
  &_CancelRoute_Request_message_typesupport_symbol_names.symbol_name[0],
  &_CancelRoute_Request_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t CancelRoute_Request_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_CancelRoute_Request_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
  &robot_msgs__srv__CancelRoute_Request__get_type_hash,
  &robot_msgs__srv__CancelRoute_Request__get_type_description,
  &robot_msgs__srv__CancelRoute_Request__get_type_description_sources,
};

}  // namespace rosidl_typesupport_c

}  // namespace srv

}  // namespace robot_msgs

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, robot_msgs, srv, CancelRoute_Request)() {
  return &::robot_msgs::srv::rosidl_typesupport_c::CancelRoute_Request_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "robot_msgs/srv/detail/cancel_route__struct.h"
// already included above
// #include "robot_msgs/srv/detail/cancel_route__type_support.h"
// already included above
// #include "robot_msgs/srv/detail/cancel_route__functions.h"
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

typedef struct _CancelRoute_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _CancelRoute_Response_type_support_ids_t;

static const _CancelRoute_Response_type_support_ids_t _CancelRoute_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _CancelRoute_Response_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _CancelRoute_Response_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _CancelRoute_Response_type_support_symbol_names_t _CancelRoute_Response_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, robot_msgs, srv, CancelRoute_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robot_msgs, srv, CancelRoute_Response)),
  }
};

typedef struct _CancelRoute_Response_type_support_data_t
{
  void * data[2];
} _CancelRoute_Response_type_support_data_t;

static _CancelRoute_Response_type_support_data_t _CancelRoute_Response_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _CancelRoute_Response_message_typesupport_map = {
  2,
  "robot_msgs",
  &_CancelRoute_Response_message_typesupport_ids.typesupport_identifier[0],
  &_CancelRoute_Response_message_typesupport_symbol_names.symbol_name[0],
  &_CancelRoute_Response_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t CancelRoute_Response_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_CancelRoute_Response_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
  &robot_msgs__srv__CancelRoute_Response__get_type_hash,
  &robot_msgs__srv__CancelRoute_Response__get_type_description,
  &robot_msgs__srv__CancelRoute_Response__get_type_description_sources,
};

}  // namespace rosidl_typesupport_c

}  // namespace srv

}  // namespace robot_msgs

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, robot_msgs, srv, CancelRoute_Response)() {
  return &::robot_msgs::srv::rosidl_typesupport_c::CancelRoute_Response_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "robot_msgs/srv/detail/cancel_route__struct.h"
// already included above
// #include "robot_msgs/srv/detail/cancel_route__type_support.h"
// already included above
// #include "robot_msgs/srv/detail/cancel_route__functions.h"
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

typedef struct _CancelRoute_Event_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _CancelRoute_Event_type_support_ids_t;

static const _CancelRoute_Event_type_support_ids_t _CancelRoute_Event_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _CancelRoute_Event_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _CancelRoute_Event_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _CancelRoute_Event_type_support_symbol_names_t _CancelRoute_Event_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, robot_msgs, srv, CancelRoute_Event)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robot_msgs, srv, CancelRoute_Event)),
  }
};

typedef struct _CancelRoute_Event_type_support_data_t
{
  void * data[2];
} _CancelRoute_Event_type_support_data_t;

static _CancelRoute_Event_type_support_data_t _CancelRoute_Event_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _CancelRoute_Event_message_typesupport_map = {
  2,
  "robot_msgs",
  &_CancelRoute_Event_message_typesupport_ids.typesupport_identifier[0],
  &_CancelRoute_Event_message_typesupport_symbol_names.symbol_name[0],
  &_CancelRoute_Event_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t CancelRoute_Event_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_CancelRoute_Event_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
  &robot_msgs__srv__CancelRoute_Event__get_type_hash,
  &robot_msgs__srv__CancelRoute_Event__get_type_description,
  &robot_msgs__srv__CancelRoute_Event__get_type_description_sources,
};

}  // namespace rosidl_typesupport_c

}  // namespace srv

}  // namespace robot_msgs

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, robot_msgs, srv, CancelRoute_Event)() {
  return &::robot_msgs::srv::rosidl_typesupport_c::CancelRoute_Event_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "robot_msgs/srv/detail/cancel_route__type_support.h"
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
typedef struct _CancelRoute_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _CancelRoute_type_support_ids_t;

static const _CancelRoute_type_support_ids_t _CancelRoute_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _CancelRoute_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _CancelRoute_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _CancelRoute_type_support_symbol_names_t _CancelRoute_service_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, robot_msgs, srv, CancelRoute)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robot_msgs, srv, CancelRoute)),
  }
};

typedef struct _CancelRoute_type_support_data_t
{
  void * data[2];
} _CancelRoute_type_support_data_t;

static _CancelRoute_type_support_data_t _CancelRoute_service_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _CancelRoute_service_typesupport_map = {
  2,
  "robot_msgs",
  &_CancelRoute_service_typesupport_ids.typesupport_identifier[0],
  &_CancelRoute_service_typesupport_symbol_names.symbol_name[0],
  &_CancelRoute_service_typesupport_data.data[0],
};

static const rosidl_service_type_support_t CancelRoute_service_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_CancelRoute_service_typesupport_map),
  rosidl_typesupport_c__get_service_typesupport_handle_function,
  &CancelRoute_Request_message_type_support_handle,
  &CancelRoute_Response_message_type_support_handle,
  &CancelRoute_Event_message_type_support_handle,
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_CREATE_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    robot_msgs,
    srv,
    CancelRoute
  ),
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_DESTROY_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    robot_msgs,
    srv,
    CancelRoute
  ),
  &robot_msgs__srv__CancelRoute__get_type_hash,
  &robot_msgs__srv__CancelRoute__get_type_description,
  &robot_msgs__srv__CancelRoute__get_type_description_sources,
};

}  // namespace rosidl_typesupport_c

}  // namespace srv

}  // namespace robot_msgs

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_c, robot_msgs, srv, CancelRoute)() {
  return &::robot_msgs::srv::rosidl_typesupport_c::CancelRoute_service_type_support_handle;
}

#ifdef __cplusplus
}
#endif
