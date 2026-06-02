// generated from rosidl_typesupport_cpp/resource/idl__type_support.cpp.em
// with input from robot_msgs:srv/PlanRoute.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "robot_msgs/srv/detail/plan_route__functions.h"
#include "robot_msgs/srv/detail/plan_route__struct.hpp"
#include "rosidl_typesupport_cpp/identifier.hpp"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
#include "rosidl_typesupport_cpp/visibility_control.h"
#include "rosidl_typesupport_interface/macros.h"

namespace robot_msgs
{

namespace srv
{

namespace rosidl_typesupport_cpp
{

typedef struct _PlanRoute_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _PlanRoute_Request_type_support_ids_t;

static const _PlanRoute_Request_type_support_ids_t _PlanRoute_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _PlanRoute_Request_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _PlanRoute_Request_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _PlanRoute_Request_type_support_symbol_names_t _PlanRoute_Request_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, robot_msgs, srv, PlanRoute_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, robot_msgs, srv, PlanRoute_Request)),
  }
};

typedef struct _PlanRoute_Request_type_support_data_t
{
  void * data[2];
} _PlanRoute_Request_type_support_data_t;

static _PlanRoute_Request_type_support_data_t _PlanRoute_Request_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _PlanRoute_Request_message_typesupport_map = {
  2,
  "robot_msgs",
  &_PlanRoute_Request_message_typesupport_ids.typesupport_identifier[0],
  &_PlanRoute_Request_message_typesupport_symbol_names.symbol_name[0],
  &_PlanRoute_Request_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t PlanRoute_Request_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_PlanRoute_Request_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &robot_msgs__srv__PlanRoute_Request__get_type_hash,
  &robot_msgs__srv__PlanRoute_Request__get_type_description,
  &robot_msgs__srv__PlanRoute_Request__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace robot_msgs

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<robot_msgs::srv::PlanRoute_Request>()
{
  return &::robot_msgs::srv::rosidl_typesupport_cpp::PlanRoute_Request_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, robot_msgs, srv, PlanRoute_Request)() {
  return get_message_type_support_handle<robot_msgs::srv::PlanRoute_Request>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "robot_msgs/srv/detail/plan_route__functions.h"
// already included above
// #include "robot_msgs/srv/detail/plan_route__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace robot_msgs
{

namespace srv
{

namespace rosidl_typesupport_cpp
{

typedef struct _PlanRoute_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _PlanRoute_Response_type_support_ids_t;

static const _PlanRoute_Response_type_support_ids_t _PlanRoute_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _PlanRoute_Response_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _PlanRoute_Response_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _PlanRoute_Response_type_support_symbol_names_t _PlanRoute_Response_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, robot_msgs, srv, PlanRoute_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, robot_msgs, srv, PlanRoute_Response)),
  }
};

typedef struct _PlanRoute_Response_type_support_data_t
{
  void * data[2];
} _PlanRoute_Response_type_support_data_t;

static _PlanRoute_Response_type_support_data_t _PlanRoute_Response_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _PlanRoute_Response_message_typesupport_map = {
  2,
  "robot_msgs",
  &_PlanRoute_Response_message_typesupport_ids.typesupport_identifier[0],
  &_PlanRoute_Response_message_typesupport_symbol_names.symbol_name[0],
  &_PlanRoute_Response_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t PlanRoute_Response_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_PlanRoute_Response_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &robot_msgs__srv__PlanRoute_Response__get_type_hash,
  &robot_msgs__srv__PlanRoute_Response__get_type_description,
  &robot_msgs__srv__PlanRoute_Response__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace robot_msgs

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<robot_msgs::srv::PlanRoute_Response>()
{
  return &::robot_msgs::srv::rosidl_typesupport_cpp::PlanRoute_Response_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, robot_msgs, srv, PlanRoute_Response)() {
  return get_message_type_support_handle<robot_msgs::srv::PlanRoute_Response>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "robot_msgs/srv/detail/plan_route__functions.h"
// already included above
// #include "robot_msgs/srv/detail/plan_route__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace robot_msgs
{

namespace srv
{

namespace rosidl_typesupport_cpp
{

typedef struct _PlanRoute_Event_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _PlanRoute_Event_type_support_ids_t;

static const _PlanRoute_Event_type_support_ids_t _PlanRoute_Event_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _PlanRoute_Event_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _PlanRoute_Event_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _PlanRoute_Event_type_support_symbol_names_t _PlanRoute_Event_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, robot_msgs, srv, PlanRoute_Event)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, robot_msgs, srv, PlanRoute_Event)),
  }
};

typedef struct _PlanRoute_Event_type_support_data_t
{
  void * data[2];
} _PlanRoute_Event_type_support_data_t;

static _PlanRoute_Event_type_support_data_t _PlanRoute_Event_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _PlanRoute_Event_message_typesupport_map = {
  2,
  "robot_msgs",
  &_PlanRoute_Event_message_typesupport_ids.typesupport_identifier[0],
  &_PlanRoute_Event_message_typesupport_symbol_names.symbol_name[0],
  &_PlanRoute_Event_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t PlanRoute_Event_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_PlanRoute_Event_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &robot_msgs__srv__PlanRoute_Event__get_type_hash,
  &robot_msgs__srv__PlanRoute_Event__get_type_description,
  &robot_msgs__srv__PlanRoute_Event__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace robot_msgs

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<robot_msgs::srv::PlanRoute_Event>()
{
  return &::robot_msgs::srv::rosidl_typesupport_cpp::PlanRoute_Event_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, robot_msgs, srv, PlanRoute_Event)() {
  return get_message_type_support_handle<robot_msgs::srv::PlanRoute_Event>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
#include "rosidl_runtime_c/service_type_support_struct.h"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "robot_msgs/srv/detail/plan_route__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_cpp/service_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace robot_msgs
{

namespace srv
{

namespace rosidl_typesupport_cpp
{

typedef struct _PlanRoute_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _PlanRoute_type_support_ids_t;

static const _PlanRoute_type_support_ids_t _PlanRoute_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _PlanRoute_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _PlanRoute_type_support_symbol_names_t;
#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _PlanRoute_type_support_symbol_names_t _PlanRoute_service_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, robot_msgs, srv, PlanRoute)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, robot_msgs, srv, PlanRoute)),
  }
};

typedef struct _PlanRoute_type_support_data_t
{
  void * data[2];
} _PlanRoute_type_support_data_t;

static _PlanRoute_type_support_data_t _PlanRoute_service_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _PlanRoute_service_typesupport_map = {
  2,
  "robot_msgs",
  &_PlanRoute_service_typesupport_ids.typesupport_identifier[0],
  &_PlanRoute_service_typesupport_symbol_names.symbol_name[0],
  &_PlanRoute_service_typesupport_data.data[0],
};

static const rosidl_service_type_support_t PlanRoute_service_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_PlanRoute_service_typesupport_map),
  ::rosidl_typesupport_cpp::get_service_typesupport_handle_function,
  ::rosidl_typesupport_cpp::get_message_type_support_handle<robot_msgs::srv::PlanRoute_Request>(),
  ::rosidl_typesupport_cpp::get_message_type_support_handle<robot_msgs::srv::PlanRoute_Response>(),
  ::rosidl_typesupport_cpp::get_message_type_support_handle<robot_msgs::srv::PlanRoute_Event>(),
  &::rosidl_typesupport_cpp::service_create_event_message<robot_msgs::srv::PlanRoute>,
  &::rosidl_typesupport_cpp::service_destroy_event_message<robot_msgs::srv::PlanRoute>,
  &robot_msgs__srv__PlanRoute__get_type_hash,
  &robot_msgs__srv__PlanRoute__get_type_description,
  &robot_msgs__srv__PlanRoute__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace robot_msgs

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
get_service_type_support_handle<robot_msgs::srv::PlanRoute>()
{
  return &::robot_msgs::srv::rosidl_typesupport_cpp::PlanRoute_service_type_support_handle;
}

}  // namespace rosidl_typesupport_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_cpp, robot_msgs, srv, PlanRoute)() {
  return ::rosidl_typesupport_cpp::get_service_type_support_handle<robot_msgs::srv::PlanRoute>();
}

#ifdef __cplusplus
}
#endif
