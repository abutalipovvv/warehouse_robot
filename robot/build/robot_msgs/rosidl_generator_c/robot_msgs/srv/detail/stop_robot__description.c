// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from robot_msgs:srv/StopRobot.idl
// generated code does not contain a copyright notice

#include "robot_msgs/srv/detail/stop_robot__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_robot_msgs
const rosidl_type_hash_t *
robot_msgs__srv__StopRobot__get_type_hash(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x63, 0x51, 0x4f, 0xbb, 0x46, 0x2f, 0x73, 0x64,
      0xc6, 0xd9, 0xd6, 0x44, 0xed, 0x35, 0xfa, 0x5c,
      0x8f, 0xbb, 0x6e, 0xae, 0x6d, 0x52, 0x6c, 0x48,
      0x75, 0xb9, 0x53, 0xa9, 0x19, 0x05, 0x5a, 0x62,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_robot_msgs
const rosidl_type_hash_t *
robot_msgs__srv__StopRobot_Request__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x4a, 0xf7, 0x3d, 0xe6, 0xa1, 0x1d, 0xeb, 0xe3,
      0x9e, 0x85, 0x5f, 0xdd, 0x09, 0x3c, 0xdc, 0x88,
      0x75, 0x91, 0x8a, 0x52, 0xcc, 0x15, 0xbc, 0x81,
      0xa0, 0xbc, 0x17, 0xcf, 0xa0, 0xf7, 0x4f, 0x4f,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_robot_msgs
const rosidl_type_hash_t *
robot_msgs__srv__StopRobot_Response__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xd8, 0x8d, 0x95, 0xad, 0xae, 0x62, 0xcb, 0xa6,
      0x0e, 0xb9, 0x75, 0x68, 0xb3, 0xa6, 0x16, 0x30,
      0xc0, 0x47, 0x8c, 0xa0, 0xac, 0x45, 0x6d, 0xd6,
      0xae, 0xb9, 0x88, 0x07, 0x0c, 0xc1, 0x22, 0x71,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_robot_msgs
const rosidl_type_hash_t *
robot_msgs__srv__StopRobot_Event__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x3f, 0xee, 0x03, 0x5b, 0x82, 0x93, 0x2b, 0x1e,
      0x2b, 0x95, 0x30, 0x64, 0x3b, 0x1f, 0xec, 0x45,
      0xa4, 0xf2, 0x62, 0x81, 0x89, 0xdd, 0x34, 0x50,
      0xe4, 0xc3, 0x5f, 0xf7, 0x95, 0xb2, 0x96, 0xf1,
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

static char robot_msgs__srv__StopRobot__TYPE_NAME[] = "robot_msgs/srv/StopRobot";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char robot_msgs__srv__StopRobot_Event__TYPE_NAME[] = "robot_msgs/srv/StopRobot_Event";
static char robot_msgs__srv__StopRobot_Request__TYPE_NAME[] = "robot_msgs/srv/StopRobot_Request";
static char robot_msgs__srv__StopRobot_Response__TYPE_NAME[] = "robot_msgs/srv/StopRobot_Response";
static char service_msgs__msg__ServiceEventInfo__TYPE_NAME[] = "service_msgs/msg/ServiceEventInfo";

// Define type names, field names, and default values
static char robot_msgs__srv__StopRobot__FIELD_NAME__request_message[] = "request_message";
static char robot_msgs__srv__StopRobot__FIELD_NAME__response_message[] = "response_message";
static char robot_msgs__srv__StopRobot__FIELD_NAME__event_message[] = "event_message";

static rosidl_runtime_c__type_description__Field robot_msgs__srv__StopRobot__FIELDS[] = {
  {
    {robot_msgs__srv__StopRobot__FIELD_NAME__request_message, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {robot_msgs__srv__StopRobot_Request__TYPE_NAME, 32, 32},
    },
    {NULL, 0, 0},
  },
  {
    {robot_msgs__srv__StopRobot__FIELD_NAME__response_message, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {robot_msgs__srv__StopRobot_Response__TYPE_NAME, 33, 33},
    },
    {NULL, 0, 0},
  },
  {
    {robot_msgs__srv__StopRobot__FIELD_NAME__event_message, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {robot_msgs__srv__StopRobot_Event__TYPE_NAME, 30, 30},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription robot_msgs__srv__StopRobot__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {robot_msgs__srv__StopRobot_Event__TYPE_NAME, 30, 30},
    {NULL, 0, 0},
  },
  {
    {robot_msgs__srv__StopRobot_Request__TYPE_NAME, 32, 32},
    {NULL, 0, 0},
  },
  {
    {robot_msgs__srv__StopRobot_Response__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
robot_msgs__srv__StopRobot__get_type_description(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {robot_msgs__srv__StopRobot__TYPE_NAME, 24, 24},
      {robot_msgs__srv__StopRobot__FIELDS, 3, 3},
    },
    {robot_msgs__srv__StopRobot__REFERENCED_TYPE_DESCRIPTIONS, 5, 5},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[1].fields = robot_msgs__srv__StopRobot_Event__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = robot_msgs__srv__StopRobot_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[3].fields = robot_msgs__srv__StopRobot_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[4].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char robot_msgs__srv__StopRobot_Request__FIELD_NAME__message[] = "message";

static rosidl_runtime_c__type_description__Field robot_msgs__srv__StopRobot_Request__FIELDS[] = {
  {
    {robot_msgs__srv__StopRobot_Request__FIELD_NAME__message, 7, 7},
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
robot_msgs__srv__StopRobot_Request__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {robot_msgs__srv__StopRobot_Request__TYPE_NAME, 32, 32},
      {robot_msgs__srv__StopRobot_Request__FIELDS, 1, 1},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char robot_msgs__srv__StopRobot_Response__FIELD_NAME__ok[] = "ok";
static char robot_msgs__srv__StopRobot_Response__FIELD_NAME__error[] = "error";

static rosidl_runtime_c__type_description__Field robot_msgs__srv__StopRobot_Response__FIELDS[] = {
  {
    {robot_msgs__srv__StopRobot_Response__FIELD_NAME__ok, 2, 2},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {robot_msgs__srv__StopRobot_Response__FIELD_NAME__error, 5, 5},
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
robot_msgs__srv__StopRobot_Response__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {robot_msgs__srv__StopRobot_Response__TYPE_NAME, 33, 33},
      {robot_msgs__srv__StopRobot_Response__FIELDS, 2, 2},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char robot_msgs__srv__StopRobot_Event__FIELD_NAME__info[] = "info";
static char robot_msgs__srv__StopRobot_Event__FIELD_NAME__request[] = "request";
static char robot_msgs__srv__StopRobot_Event__FIELD_NAME__response[] = "response";

static rosidl_runtime_c__type_description__Field robot_msgs__srv__StopRobot_Event__FIELDS[] = {
  {
    {robot_msgs__srv__StopRobot_Event__FIELD_NAME__info, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    },
    {NULL, 0, 0},
  },
  {
    {robot_msgs__srv__StopRobot_Event__FIELD_NAME__request, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {robot_msgs__srv__StopRobot_Request__TYPE_NAME, 32, 32},
    },
    {NULL, 0, 0},
  },
  {
    {robot_msgs__srv__StopRobot_Event__FIELD_NAME__response, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {robot_msgs__srv__StopRobot_Response__TYPE_NAME, 33, 33},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription robot_msgs__srv__StopRobot_Event__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {robot_msgs__srv__StopRobot_Request__TYPE_NAME, 32, 32},
    {NULL, 0, 0},
  },
  {
    {robot_msgs__srv__StopRobot_Response__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
robot_msgs__srv__StopRobot_Event__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {robot_msgs__srv__StopRobot_Event__TYPE_NAME, 30, 30},
      {robot_msgs__srv__StopRobot_Event__FIELDS, 3, 3},
    },
    {robot_msgs__srv__StopRobot_Event__REFERENCED_TYPE_DESCRIPTIONS, 4, 4},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[1].fields = robot_msgs__srv__StopRobot_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = robot_msgs__srv__StopRobot_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[3].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "string message\n"
  "---\n"
  "bool ok\n"
  "string error";

static char srv_encoding[] = "srv";
static char implicit_encoding[] = "implicit";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
robot_msgs__srv__StopRobot__get_individual_type_description_source(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {robot_msgs__srv__StopRobot__TYPE_NAME, 24, 24},
    {srv_encoding, 3, 3},
    {toplevel_type_raw_source, 40, 40},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
robot_msgs__srv__StopRobot_Request__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {robot_msgs__srv__StopRobot_Request__TYPE_NAME, 32, 32},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
robot_msgs__srv__StopRobot_Response__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {robot_msgs__srv__StopRobot_Response__TYPE_NAME, 33, 33},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
robot_msgs__srv__StopRobot_Event__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {robot_msgs__srv__StopRobot_Event__TYPE_NAME, 30, 30},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
robot_msgs__srv__StopRobot__get_type_description_sources(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[6];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 6, 6};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *robot_msgs__srv__StopRobot__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *robot_msgs__srv__StopRobot_Event__get_individual_type_description_source(NULL);
    sources[3] = *robot_msgs__srv__StopRobot_Request__get_individual_type_description_source(NULL);
    sources[4] = *robot_msgs__srv__StopRobot_Response__get_individual_type_description_source(NULL);
    sources[5] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
robot_msgs__srv__StopRobot_Request__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *robot_msgs__srv__StopRobot_Request__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
robot_msgs__srv__StopRobot_Response__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *robot_msgs__srv__StopRobot_Response__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
robot_msgs__srv__StopRobot_Event__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[5];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 5, 5};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *robot_msgs__srv__StopRobot_Event__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *robot_msgs__srv__StopRobot_Request__get_individual_type_description_source(NULL);
    sources[3] = *robot_msgs__srv__StopRobot_Response__get_individual_type_description_source(NULL);
    sources[4] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
