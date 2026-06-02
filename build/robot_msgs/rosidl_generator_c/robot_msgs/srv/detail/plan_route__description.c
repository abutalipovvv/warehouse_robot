// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from robot_msgs:srv/PlanRoute.idl
// generated code does not contain a copyright notice

#include "robot_msgs/srv/detail/plan_route__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_robot_msgs
const rosidl_type_hash_t *
robot_msgs__srv__PlanRoute__get_type_hash(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x80, 0x77, 0x95, 0x1e, 0x2b, 0x4b, 0x0a, 0x84,
      0xad, 0xa3, 0x21, 0xef, 0x90, 0x26, 0x46, 0x98,
      0x43, 0xca, 0xb2, 0x22, 0x19, 0x69, 0x31, 0xe7,
      0xd4, 0xe3, 0xd3, 0x80, 0x0d, 0x59, 0xd9, 0x22,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_robot_msgs
const rosidl_type_hash_t *
robot_msgs__srv__PlanRoute_Request__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xe2, 0x18, 0xb9, 0x96, 0xe0, 0x65, 0x65, 0x8f,
      0x0f, 0xdc, 0xfa, 0x23, 0xe6, 0x62, 0x25, 0x45,
      0xb3, 0xaa, 0xdc, 0xf2, 0xcf, 0x0d, 0x7d, 0x2a,
      0x99, 0xea, 0xa3, 0x19, 0x25, 0x57, 0x90, 0x98,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_robot_msgs
const rosidl_type_hash_t *
robot_msgs__srv__PlanRoute_Response__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xa9, 0xf1, 0x72, 0x31, 0x0e, 0xc7, 0xbe, 0x34,
      0xd1, 0xe9, 0x1f, 0xb6, 0xd0, 0x02, 0x66, 0x0b,
      0x7a, 0x0c, 0xd0, 0x99, 0xdd, 0x3d, 0x15, 0x62,
      0x01, 0x2a, 0x29, 0x6d, 0x4f, 0xb0, 0xd1, 0xbc,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_robot_msgs
const rosidl_type_hash_t *
robot_msgs__srv__PlanRoute_Event__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xb3, 0x22, 0x5c, 0x3c, 0xd6, 0xc6, 0xd1, 0x9a,
      0xd9, 0xae, 0xeb, 0x98, 0x08, 0x3e, 0xa2, 0x04,
      0x30, 0x5d, 0x9b, 0x31, 0x20, 0x8a, 0xad, 0xe6,
      0x50, 0xf5, 0x7c, 0x2e, 0x93, 0xcf, 0x02, 0xa1,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "service_msgs/msg/detail/service_event_info__functions.h"
#include "builtin_interfaces/msg/detail/time__functions.h"

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

static char robot_msgs__srv__PlanRoute__TYPE_NAME[] = "robot_msgs/srv/PlanRoute";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char robot_msgs__srv__PlanRoute_Event__TYPE_NAME[] = "robot_msgs/srv/PlanRoute_Event";
static char robot_msgs__srv__PlanRoute_Request__TYPE_NAME[] = "robot_msgs/srv/PlanRoute_Request";
static char robot_msgs__srv__PlanRoute_Response__TYPE_NAME[] = "robot_msgs/srv/PlanRoute_Response";
static char service_msgs__msg__ServiceEventInfo__TYPE_NAME[] = "service_msgs/msg/ServiceEventInfo";

// Define type names, field names, and default values
static char robot_msgs__srv__PlanRoute__FIELD_NAME__request_message[] = "request_message";
static char robot_msgs__srv__PlanRoute__FIELD_NAME__response_message[] = "response_message";
static char robot_msgs__srv__PlanRoute__FIELD_NAME__event_message[] = "event_message";

static rosidl_runtime_c__type_description__Field robot_msgs__srv__PlanRoute__FIELDS[] = {
  {
    {robot_msgs__srv__PlanRoute__FIELD_NAME__request_message, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {robot_msgs__srv__PlanRoute_Request__TYPE_NAME, 32, 32},
    },
    {NULL, 0, 0},
  },
  {
    {robot_msgs__srv__PlanRoute__FIELD_NAME__response_message, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {robot_msgs__srv__PlanRoute_Response__TYPE_NAME, 33, 33},
    },
    {NULL, 0, 0},
  },
  {
    {robot_msgs__srv__PlanRoute__FIELD_NAME__event_message, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {robot_msgs__srv__PlanRoute_Event__TYPE_NAME, 30, 30},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription robot_msgs__srv__PlanRoute__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {robot_msgs__srv__PlanRoute_Event__TYPE_NAME, 30, 30},
    {NULL, 0, 0},
  },
  {
    {robot_msgs__srv__PlanRoute_Request__TYPE_NAME, 32, 32},
    {NULL, 0, 0},
  },
  {
    {robot_msgs__srv__PlanRoute_Response__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
robot_msgs__srv__PlanRoute__get_type_description(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {robot_msgs__srv__PlanRoute__TYPE_NAME, 24, 24},
      {robot_msgs__srv__PlanRoute__FIELDS, 3, 3},
    },
    {robot_msgs__srv__PlanRoute__REFERENCED_TYPE_DESCRIPTIONS, 5, 5},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[1].fields = robot_msgs__srv__PlanRoute_Event__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = robot_msgs__srv__PlanRoute_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[3].fields = robot_msgs__srv__PlanRoute_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[4].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char robot_msgs__srv__PlanRoute_Request__FIELD_NAME__goal_lm[] = "goal_lm";
static char robot_msgs__srv__PlanRoute_Request__FIELD_NAME__start_lm[] = "start_lm";
static char robot_msgs__srv__PlanRoute_Request__FIELD_NAME__use_start_pose[] = "use_start_pose";
static char robot_msgs__srv__PlanRoute_Request__FIELD_NAME__start_x[] = "start_x";
static char robot_msgs__srv__PlanRoute_Request__FIELD_NAME__start_y[] = "start_y";
static char robot_msgs__srv__PlanRoute_Request__FIELD_NAME__start_yaw[] = "start_yaw";

static rosidl_runtime_c__type_description__Field robot_msgs__srv__PlanRoute_Request__FIELDS[] = {
  {
    {robot_msgs__srv__PlanRoute_Request__FIELD_NAME__goal_lm, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {robot_msgs__srv__PlanRoute_Request__FIELD_NAME__start_lm, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {robot_msgs__srv__PlanRoute_Request__FIELD_NAME__use_start_pose, 14, 14},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {robot_msgs__srv__PlanRoute_Request__FIELD_NAME__start_x, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {robot_msgs__srv__PlanRoute_Request__FIELD_NAME__start_y, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {robot_msgs__srv__PlanRoute_Request__FIELD_NAME__start_yaw, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
robot_msgs__srv__PlanRoute_Request__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {robot_msgs__srv__PlanRoute_Request__TYPE_NAME, 32, 32},
      {robot_msgs__srv__PlanRoute_Request__FIELDS, 6, 6},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char robot_msgs__srv__PlanRoute_Response__FIELD_NAME__ok[] = "ok";
static char robot_msgs__srv__PlanRoute_Response__FIELD_NAME__error[] = "error";
static char robot_msgs__srv__PlanRoute_Response__FIELD_NAME__route_json[] = "route_json";

static rosidl_runtime_c__type_description__Field robot_msgs__srv__PlanRoute_Response__FIELDS[] = {
  {
    {robot_msgs__srv__PlanRoute_Response__FIELD_NAME__ok, 2, 2},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {robot_msgs__srv__PlanRoute_Response__FIELD_NAME__error, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {robot_msgs__srv__PlanRoute_Response__FIELD_NAME__route_json, 10, 10},
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
robot_msgs__srv__PlanRoute_Response__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {robot_msgs__srv__PlanRoute_Response__TYPE_NAME, 33, 33},
      {robot_msgs__srv__PlanRoute_Response__FIELDS, 3, 3},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char robot_msgs__srv__PlanRoute_Event__FIELD_NAME__info[] = "info";
static char robot_msgs__srv__PlanRoute_Event__FIELD_NAME__request[] = "request";
static char robot_msgs__srv__PlanRoute_Event__FIELD_NAME__response[] = "response";

static rosidl_runtime_c__type_description__Field robot_msgs__srv__PlanRoute_Event__FIELDS[] = {
  {
    {robot_msgs__srv__PlanRoute_Event__FIELD_NAME__info, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    },
    {NULL, 0, 0},
  },
  {
    {robot_msgs__srv__PlanRoute_Event__FIELD_NAME__request, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {robot_msgs__srv__PlanRoute_Request__TYPE_NAME, 32, 32},
    },
    {NULL, 0, 0},
  },
  {
    {robot_msgs__srv__PlanRoute_Event__FIELD_NAME__response, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {robot_msgs__srv__PlanRoute_Response__TYPE_NAME, 33, 33},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription robot_msgs__srv__PlanRoute_Event__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {robot_msgs__srv__PlanRoute_Request__TYPE_NAME, 32, 32},
    {NULL, 0, 0},
  },
  {
    {robot_msgs__srv__PlanRoute_Response__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
robot_msgs__srv__PlanRoute_Event__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {robot_msgs__srv__PlanRoute_Event__TYPE_NAME, 30, 30},
      {robot_msgs__srv__PlanRoute_Event__FIELDS, 3, 3},
    },
    {robot_msgs__srv__PlanRoute_Event__REFERENCED_TYPE_DESCRIPTIONS, 4, 4},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[1].fields = robot_msgs__srv__PlanRoute_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = robot_msgs__srv__PlanRoute_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[3].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "string goal_lm\n"
  "string start_lm\n"
  "bool use_start_pose\n"
  "float64 start_x\n"
  "float64 start_y\n"
  "float64 start_yaw\n"
  "---\n"
  "bool ok\n"
  "string error\n"
  "string route_json";

static char srv_encoding[] = "srv";
static char implicit_encoding[] = "implicit";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
robot_msgs__srv__PlanRoute__get_individual_type_description_source(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {robot_msgs__srv__PlanRoute__TYPE_NAME, 24, 24},
    {srv_encoding, 3, 3},
    {toplevel_type_raw_source, 144, 144},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
robot_msgs__srv__PlanRoute_Request__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {robot_msgs__srv__PlanRoute_Request__TYPE_NAME, 32, 32},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
robot_msgs__srv__PlanRoute_Response__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {robot_msgs__srv__PlanRoute_Response__TYPE_NAME, 33, 33},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
robot_msgs__srv__PlanRoute_Event__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {robot_msgs__srv__PlanRoute_Event__TYPE_NAME, 30, 30},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
robot_msgs__srv__PlanRoute__get_type_description_sources(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[6];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 6, 6};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *robot_msgs__srv__PlanRoute__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *robot_msgs__srv__PlanRoute_Event__get_individual_type_description_source(NULL);
    sources[3] = *robot_msgs__srv__PlanRoute_Request__get_individual_type_description_source(NULL);
    sources[4] = *robot_msgs__srv__PlanRoute_Response__get_individual_type_description_source(NULL);
    sources[5] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
robot_msgs__srv__PlanRoute_Request__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *robot_msgs__srv__PlanRoute_Request__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
robot_msgs__srv__PlanRoute_Response__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *robot_msgs__srv__PlanRoute_Response__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
robot_msgs__srv__PlanRoute_Event__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[5];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 5, 5};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *robot_msgs__srv__PlanRoute_Event__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *robot_msgs__srv__PlanRoute_Request__get_individual_type_description_source(NULL);
    sources[3] = *robot_msgs__srv__PlanRoute_Response__get_individual_type_description_source(NULL);
    sources[4] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
