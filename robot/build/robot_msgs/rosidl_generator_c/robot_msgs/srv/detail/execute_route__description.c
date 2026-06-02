// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from robot_msgs:srv/ExecuteRoute.idl
// generated code does not contain a copyright notice

#include "robot_msgs/srv/detail/execute_route__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_robot_msgs
const rosidl_type_hash_t *
robot_msgs__srv__ExecuteRoute__get_type_hash(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x40, 0xa7, 0x28, 0x88, 0x59, 0xd2, 0xd6, 0x08,
      0x6d, 0x3f, 0xa3, 0x10, 0x4a, 0x9e, 0x28, 0x15,
      0x47, 0x10, 0x23, 0xa3, 0x33, 0xc1, 0x49, 0x12,
      0x83, 0xb0, 0xd7, 0xa2, 0xbf, 0x15, 0x19, 0xe6,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_robot_msgs
const rosidl_type_hash_t *
robot_msgs__srv__ExecuteRoute_Request__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xa8, 0xa3, 0x51, 0x81, 0xe5, 0xde, 0x89, 0x7b,
      0x75, 0x6c, 0x68, 0x1a, 0xa9, 0x89, 0x09, 0x09,
      0xa2, 0x34, 0x36, 0x7f, 0x73, 0x3a, 0xc6, 0x27,
      0x0c, 0x32, 0x16, 0xd1, 0xca, 0x74, 0xdf, 0xa4,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_robot_msgs
const rosidl_type_hash_t *
robot_msgs__srv__ExecuteRoute_Response__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x74, 0x78, 0x7d, 0x30, 0x52, 0x37, 0xae, 0x55,
      0xc8, 0xca, 0x9b, 0x3c, 0xd1, 0xaa, 0x33, 0xcf,
      0xad, 0x0a, 0x59, 0x70, 0x65, 0x68, 0xa9, 0xcc,
      0x74, 0xaa, 0xa1, 0xd5, 0x05, 0x70, 0x9c, 0x7e,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_robot_msgs
const rosidl_type_hash_t *
robot_msgs__srv__ExecuteRoute_Event__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xee, 0x39, 0x1e, 0xb2, 0x55, 0xc7, 0x84, 0xbb,
      0xaa, 0xaa, 0x2b, 0x8d, 0x1e, 0x39, 0x59, 0x57,
      0x20, 0x7d, 0xc7, 0xca, 0xce, 0x15, 0xf7, 0x2b,
      0xd4, 0x59, 0xa8, 0x96, 0xda, 0xde, 0xf3, 0xe1,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "builtin_interfaces/msg/detail/time__functions.h"
#include "service_msgs/msg/detail/service_event_info__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t builtin_interfaces__msg__Time__EXPECTED_HASH = {1, {
    0xb1, 0x06, 0x23, 0x5e, 0x25, 0xa4, 0xc5, 0xed,
    0x35, 0x09, 0x8a, 0xa0, 0xa6, 0x1a, 0x3e, 0xe9,
    0xc9, 0xb1, 0x8d, 0x19, 0x7f, 0x39, 0x8b, 0x0e,
    0x42, 0x06, 0xce, 0xa9, 0xac, 0xf9, 0xc1, 0x97,
  }};
static const rosidl_type_hash_t service_msgs__msg__ServiceEventInfo__EXPECTED_HASH = {1, {
    0x41, 0xbc, 0xbb, 0xe0, 0x7a, 0x75, 0xc9, 0xb5,
    0x2b, 0xc9, 0x6b, 0xfd, 0x5c, 0x24, 0xd7, 0xf0,
    0xfc, 0x0a, 0x08, 0xc0, 0xcb, 0x79, 0x21, 0xb3,
    0x37, 0x3c, 0x57, 0x32, 0x34, 0x5a, 0x6f, 0x45,
  }};
#endif

static char robot_msgs__srv__ExecuteRoute__TYPE_NAME[] = "robot_msgs/srv/ExecuteRoute";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char robot_msgs__srv__ExecuteRoute_Event__TYPE_NAME[] = "robot_msgs/srv/ExecuteRoute_Event";
static char robot_msgs__srv__ExecuteRoute_Request__TYPE_NAME[] = "robot_msgs/srv/ExecuteRoute_Request";
static char robot_msgs__srv__ExecuteRoute_Response__TYPE_NAME[] = "robot_msgs/srv/ExecuteRoute_Response";
static char service_msgs__msg__ServiceEventInfo__TYPE_NAME[] = "service_msgs/msg/ServiceEventInfo";

// Define type names, field names, and default values
static char robot_msgs__srv__ExecuteRoute__FIELD_NAME__request_message[] = "request_message";
static char robot_msgs__srv__ExecuteRoute__FIELD_NAME__response_message[] = "response_message";
static char robot_msgs__srv__ExecuteRoute__FIELD_NAME__event_message[] = "event_message";

static rosidl_runtime_c__type_description__Field robot_msgs__srv__ExecuteRoute__FIELDS[] = {
  {
    {robot_msgs__srv__ExecuteRoute__FIELD_NAME__request_message, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {robot_msgs__srv__ExecuteRoute_Request__TYPE_NAME, 35, 35},
    },
    {NULL, 0, 0},
  },
  {
    {robot_msgs__srv__ExecuteRoute__FIELD_NAME__response_message, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {robot_msgs__srv__ExecuteRoute_Response__TYPE_NAME, 36, 36},
    },
    {NULL, 0, 0},
  },
  {
    {robot_msgs__srv__ExecuteRoute__FIELD_NAME__event_message, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {robot_msgs__srv__ExecuteRoute_Event__TYPE_NAME, 33, 33},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription robot_msgs__srv__ExecuteRoute__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {robot_msgs__srv__ExecuteRoute_Event__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
  {
    {robot_msgs__srv__ExecuteRoute_Request__TYPE_NAME, 35, 35},
    {NULL, 0, 0},
  },
  {
    {robot_msgs__srv__ExecuteRoute_Response__TYPE_NAME, 36, 36},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
robot_msgs__srv__ExecuteRoute__get_type_description(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {robot_msgs__srv__ExecuteRoute__TYPE_NAME, 27, 27},
      {robot_msgs__srv__ExecuteRoute__FIELDS, 3, 3},
    },
    {robot_msgs__srv__ExecuteRoute__REFERENCED_TYPE_DESCRIPTIONS, 5, 5},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[1].fields = robot_msgs__srv__ExecuteRoute_Event__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = robot_msgs__srv__ExecuteRoute_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[3].fields = robot_msgs__srv__ExecuteRoute_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[4].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char robot_msgs__srv__ExecuteRoute_Request__FIELD_NAME__route_json[] = "route_json";

static rosidl_runtime_c__type_description__Field robot_msgs__srv__ExecuteRoute_Request__FIELDS[] = {
  {
    {robot_msgs__srv__ExecuteRoute_Request__FIELD_NAME__route_json, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
robot_msgs__srv__ExecuteRoute_Request__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {robot_msgs__srv__ExecuteRoute_Request__TYPE_NAME, 35, 35},
      {robot_msgs__srv__ExecuteRoute_Request__FIELDS, 1, 1},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char robot_msgs__srv__ExecuteRoute_Response__FIELD_NAME__ok[] = "ok";
static char robot_msgs__srv__ExecuteRoute_Response__FIELD_NAME__error[] = "error";

static rosidl_runtime_c__type_description__Field robot_msgs__srv__ExecuteRoute_Response__FIELDS[] = {
  {
    {robot_msgs__srv__ExecuteRoute_Response__FIELD_NAME__ok, 2, 2},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {robot_msgs__srv__ExecuteRoute_Response__FIELD_NAME__error, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
robot_msgs__srv__ExecuteRoute_Response__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {robot_msgs__srv__ExecuteRoute_Response__TYPE_NAME, 36, 36},
      {robot_msgs__srv__ExecuteRoute_Response__FIELDS, 2, 2},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char robot_msgs__srv__ExecuteRoute_Event__FIELD_NAME__info[] = "info";
static char robot_msgs__srv__ExecuteRoute_Event__FIELD_NAME__request[] = "request";
static char robot_msgs__srv__ExecuteRoute_Event__FIELD_NAME__response[] = "response";

static rosidl_runtime_c__type_description__Field robot_msgs__srv__ExecuteRoute_Event__FIELDS[] = {
  {
    {robot_msgs__srv__ExecuteRoute_Event__FIELD_NAME__info, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    },
    {NULL, 0, 0},
  },
  {
    {robot_msgs__srv__ExecuteRoute_Event__FIELD_NAME__request, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {robot_msgs__srv__ExecuteRoute_Request__TYPE_NAME, 35, 35},
    },
    {NULL, 0, 0},
  },
  {
    {robot_msgs__srv__ExecuteRoute_Event__FIELD_NAME__response, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {robot_msgs__srv__ExecuteRoute_Response__TYPE_NAME, 36, 36},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription robot_msgs__srv__ExecuteRoute_Event__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {robot_msgs__srv__ExecuteRoute_Request__TYPE_NAME, 35, 35},
    {NULL, 0, 0},
  },
  {
    {robot_msgs__srv__ExecuteRoute_Response__TYPE_NAME, 36, 36},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
robot_msgs__srv__ExecuteRoute_Event__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {robot_msgs__srv__ExecuteRoute_Event__TYPE_NAME, 33, 33},
      {robot_msgs__srv__ExecuteRoute_Event__FIELDS, 3, 3},
    },
    {robot_msgs__srv__ExecuteRoute_Event__REFERENCED_TYPE_DESCRIPTIONS, 4, 4},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[1].fields = robot_msgs__srv__ExecuteRoute_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = robot_msgs__srv__ExecuteRoute_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[3].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "string route_json\n"
  "---\n"
  "bool ok\n"
  "string error";

static char srv_encoding[] = "srv";
static char implicit_encoding[] = "implicit";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
robot_msgs__srv__ExecuteRoute__get_individual_type_description_source(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {robot_msgs__srv__ExecuteRoute__TYPE_NAME, 27, 27},
    {srv_encoding, 3, 3},
    {toplevel_type_raw_source, 43, 43},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
robot_msgs__srv__ExecuteRoute_Request__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {robot_msgs__srv__ExecuteRoute_Request__TYPE_NAME, 35, 35},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
robot_msgs__srv__ExecuteRoute_Response__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {robot_msgs__srv__ExecuteRoute_Response__TYPE_NAME, 36, 36},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
robot_msgs__srv__ExecuteRoute_Event__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {robot_msgs__srv__ExecuteRoute_Event__TYPE_NAME, 33, 33},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
robot_msgs__srv__ExecuteRoute__get_type_description_sources(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[6];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 6, 6};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *robot_msgs__srv__ExecuteRoute__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *robot_msgs__srv__ExecuteRoute_Event__get_individual_type_description_source(NULL);
    sources[3] = *robot_msgs__srv__ExecuteRoute_Request__get_individual_type_description_source(NULL);
    sources[4] = *robot_msgs__srv__ExecuteRoute_Response__get_individual_type_description_source(NULL);
    sources[5] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
robot_msgs__srv__ExecuteRoute_Request__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *robot_msgs__srv__ExecuteRoute_Request__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
robot_msgs__srv__ExecuteRoute_Response__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *robot_msgs__srv__ExecuteRoute_Response__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
robot_msgs__srv__ExecuteRoute_Event__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[5];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 5, 5};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *robot_msgs__srv__ExecuteRoute_Event__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *robot_msgs__srv__ExecuteRoute_Request__get_individual_type_description_source(NULL);
    sources[3] = *robot_msgs__srv__ExecuteRoute_Response__get_individual_type_description_source(NULL);
    sources[4] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
