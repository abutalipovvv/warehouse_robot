// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from robot_msgs:srv/SetTeleop.idl
// generated code does not contain a copyright notice

#include "robot_msgs/srv/detail/set_teleop__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_robot_msgs
const rosidl_type_hash_t *
robot_msgs__srv__SetTeleop__get_type_hash(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x92, 0xa8, 0x7d, 0xce, 0xd6, 0xb2, 0xac, 0x4a,
      0x81, 0x4e, 0x54, 0x27, 0xe8, 0x03, 0x2a, 0xb6,
      0x81, 0xf2, 0x74, 0x2e, 0xf1, 0x1f, 0xbe, 0x6f,
      0xbe, 0x9f, 0xfd, 0x2b, 0x05, 0x25, 0x3b, 0xc9,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_robot_msgs
const rosidl_type_hash_t *
robot_msgs__srv__SetTeleop_Request__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x5d, 0xeb, 0xd3, 0x32, 0xc5, 0x27, 0x59, 0x5b,
      0xaf, 0x01, 0xaa, 0x38, 0xfb, 0xa1, 0x63, 0xf2,
      0x7f, 0xcd, 0x0e, 0xda, 0xe5, 0xc6, 0x42, 0x1c,
      0x75, 0x7f, 0x64, 0xb5, 0x54, 0x21, 0xff, 0xfd,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_robot_msgs
const rosidl_type_hash_t *
robot_msgs__srv__SetTeleop_Response__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x38, 0xf5, 0xb1, 0x80, 0x2c, 0xdd, 0xe2, 0x37,
      0x31, 0x13, 0xea, 0x00, 0x6a, 0x97, 0x8f, 0x05,
      0xb1, 0xbd, 0x6a, 0x9f, 0x6f, 0xb4, 0xdd, 0x79,
      0x33, 0x06, 0x5c, 0xfe, 0xfb, 0x78, 0x87, 0xfe,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_robot_msgs
const rosidl_type_hash_t *
robot_msgs__srv__SetTeleop_Event__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xc3, 0xcd, 0x31, 0x80, 0xab, 0x60, 0x0a, 0x1d,
      0x26, 0x08, 0xec, 0x12, 0xe2, 0x94, 0xaf, 0x50,
      0xcc, 0xe7, 0xec, 0x88, 0x5c, 0x44, 0xfc, 0x0a,
      0x07, 0x78, 0x5b, 0x2f, 0xdd, 0x10, 0x91, 0xc6,
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

static char robot_msgs__srv__SetTeleop__TYPE_NAME[] = "robot_msgs/srv/SetTeleop";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char robot_msgs__srv__SetTeleop_Event__TYPE_NAME[] = "robot_msgs/srv/SetTeleop_Event";
static char robot_msgs__srv__SetTeleop_Request__TYPE_NAME[] = "robot_msgs/srv/SetTeleop_Request";
static char robot_msgs__srv__SetTeleop_Response__TYPE_NAME[] = "robot_msgs/srv/SetTeleop_Response";
static char service_msgs__msg__ServiceEventInfo__TYPE_NAME[] = "service_msgs/msg/ServiceEventInfo";

// Define type names, field names, and default values
static char robot_msgs__srv__SetTeleop__FIELD_NAME__request_message[] = "request_message";
static char robot_msgs__srv__SetTeleop__FIELD_NAME__response_message[] = "response_message";
static char robot_msgs__srv__SetTeleop__FIELD_NAME__event_message[] = "event_message";

static rosidl_runtime_c__type_description__Field robot_msgs__srv__SetTeleop__FIELDS[] = {
  {
    {robot_msgs__srv__SetTeleop__FIELD_NAME__request_message, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {robot_msgs__srv__SetTeleop_Request__TYPE_NAME, 32, 32},
    },
    {NULL, 0, 0},
  },
  {
    {robot_msgs__srv__SetTeleop__FIELD_NAME__response_message, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {robot_msgs__srv__SetTeleop_Response__TYPE_NAME, 33, 33},
    },
    {NULL, 0, 0},
  },
  {
    {robot_msgs__srv__SetTeleop__FIELD_NAME__event_message, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {robot_msgs__srv__SetTeleop_Event__TYPE_NAME, 30, 30},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription robot_msgs__srv__SetTeleop__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {robot_msgs__srv__SetTeleop_Event__TYPE_NAME, 30, 30},
    {NULL, 0, 0},
  },
  {
    {robot_msgs__srv__SetTeleop_Request__TYPE_NAME, 32, 32},
    {NULL, 0, 0},
  },
  {
    {robot_msgs__srv__SetTeleop_Response__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
robot_msgs__srv__SetTeleop__get_type_description(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {robot_msgs__srv__SetTeleop__TYPE_NAME, 24, 24},
      {robot_msgs__srv__SetTeleop__FIELDS, 3, 3},
    },
    {robot_msgs__srv__SetTeleop__REFERENCED_TYPE_DESCRIPTIONS, 5, 5},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[1].fields = robot_msgs__srv__SetTeleop_Event__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = robot_msgs__srv__SetTeleop_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[3].fields = robot_msgs__srv__SetTeleop_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[4].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char robot_msgs__srv__SetTeleop_Request__FIELD_NAME__linear[] = "linear";
static char robot_msgs__srv__SetTeleop_Request__FIELD_NAME__angular[] = "angular";
static char robot_msgs__srv__SetTeleop_Request__FIELD_NAME__timeout_ms[] = "timeout_ms";

static rosidl_runtime_c__type_description__Field robot_msgs__srv__SetTeleop_Request__FIELDS[] = {
  {
    {robot_msgs__srv__SetTeleop_Request__FIELD_NAME__linear, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {robot_msgs__srv__SetTeleop_Request__FIELD_NAME__angular, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {robot_msgs__srv__SetTeleop_Request__FIELD_NAME__timeout_ms, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
robot_msgs__srv__SetTeleop_Request__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {robot_msgs__srv__SetTeleop_Request__TYPE_NAME, 32, 32},
      {robot_msgs__srv__SetTeleop_Request__FIELDS, 3, 3},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char robot_msgs__srv__SetTeleop_Response__FIELD_NAME__ok[] = "ok";
static char robot_msgs__srv__SetTeleop_Response__FIELD_NAME__error[] = "error";

static rosidl_runtime_c__type_description__Field robot_msgs__srv__SetTeleop_Response__FIELDS[] = {
  {
    {robot_msgs__srv__SetTeleop_Response__FIELD_NAME__ok, 2, 2},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {robot_msgs__srv__SetTeleop_Response__FIELD_NAME__error, 5, 5},
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
robot_msgs__srv__SetTeleop_Response__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {robot_msgs__srv__SetTeleop_Response__TYPE_NAME, 33, 33},
      {robot_msgs__srv__SetTeleop_Response__FIELDS, 2, 2},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char robot_msgs__srv__SetTeleop_Event__FIELD_NAME__info[] = "info";
static char robot_msgs__srv__SetTeleop_Event__FIELD_NAME__request[] = "request";
static char robot_msgs__srv__SetTeleop_Event__FIELD_NAME__response[] = "response";

static rosidl_runtime_c__type_description__Field robot_msgs__srv__SetTeleop_Event__FIELDS[] = {
  {
    {robot_msgs__srv__SetTeleop_Event__FIELD_NAME__info, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    },
    {NULL, 0, 0},
  },
  {
    {robot_msgs__srv__SetTeleop_Event__FIELD_NAME__request, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {robot_msgs__srv__SetTeleop_Request__TYPE_NAME, 32, 32},
    },
    {NULL, 0, 0},
  },
  {
    {robot_msgs__srv__SetTeleop_Event__FIELD_NAME__response, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {robot_msgs__srv__SetTeleop_Response__TYPE_NAME, 33, 33},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription robot_msgs__srv__SetTeleop_Event__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {robot_msgs__srv__SetTeleop_Request__TYPE_NAME, 32, 32},
    {NULL, 0, 0},
  },
  {
    {robot_msgs__srv__SetTeleop_Response__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
robot_msgs__srv__SetTeleop_Event__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {robot_msgs__srv__SetTeleop_Event__TYPE_NAME, 30, 30},
      {robot_msgs__srv__SetTeleop_Event__FIELDS, 3, 3},
    },
    {robot_msgs__srv__SetTeleop_Event__REFERENCED_TYPE_DESCRIPTIONS, 4, 4},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[1].fields = robot_msgs__srv__SetTeleop_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = robot_msgs__srv__SetTeleop_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[3].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "float64 linear\n"
  "float64 angular\n"
  "uint32 timeout_ms\n"
  "---\n"
  "bool ok\n"
  "string error";

static char srv_encoding[] = "srv";
static char implicit_encoding[] = "implicit";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
robot_msgs__srv__SetTeleop__get_individual_type_description_source(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {robot_msgs__srv__SetTeleop__TYPE_NAME, 24, 24},
    {srv_encoding, 3, 3},
    {toplevel_type_raw_source, 74, 74},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
robot_msgs__srv__SetTeleop_Request__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {robot_msgs__srv__SetTeleop_Request__TYPE_NAME, 32, 32},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
robot_msgs__srv__SetTeleop_Response__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {robot_msgs__srv__SetTeleop_Response__TYPE_NAME, 33, 33},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
robot_msgs__srv__SetTeleop_Event__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {robot_msgs__srv__SetTeleop_Event__TYPE_NAME, 30, 30},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
robot_msgs__srv__SetTeleop__get_type_description_sources(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[6];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 6, 6};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *robot_msgs__srv__SetTeleop__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *robot_msgs__srv__SetTeleop_Event__get_individual_type_description_source(NULL);
    sources[3] = *robot_msgs__srv__SetTeleop_Request__get_individual_type_description_source(NULL);
    sources[4] = *robot_msgs__srv__SetTeleop_Response__get_individual_type_description_source(NULL);
    sources[5] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
robot_msgs__srv__SetTeleop_Request__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *robot_msgs__srv__SetTeleop_Request__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
robot_msgs__srv__SetTeleop_Response__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *robot_msgs__srv__SetTeleop_Response__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
robot_msgs__srv__SetTeleop_Event__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[5];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 5, 5};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *robot_msgs__srv__SetTeleop_Event__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *robot_msgs__srv__SetTeleop_Request__get_individual_type_description_source(NULL);
    sources[3] = *robot_msgs__srv__SetTeleop_Response__get_individual_type_description_source(NULL);
    sources[4] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
