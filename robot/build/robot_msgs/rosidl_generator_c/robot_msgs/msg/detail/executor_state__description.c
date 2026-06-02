// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from robot_msgs:msg/ExecutorState.idl
// generated code does not contain a copyright notice

#include "robot_msgs/msg/detail/executor_state__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_robot_msgs
const rosidl_type_hash_t *
robot_msgs__msg__ExecutorState__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xb6, 0xff, 0x01, 0x63, 0x00, 0x8b, 0xbb, 0xbc,
      0x04, 0xc3, 0x83, 0x98, 0x1e, 0xa1, 0xba, 0xf7,
      0xfd, 0x84, 0xb7, 0x1b, 0x41, 0x80, 0x09, 0x84,
      0xa3, 0x5d, 0xf4, 0x3e, 0x3f, 0x42, 0xb5, 0x05,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "builtin_interfaces/msg/detail/time__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t builtin_interfaces__msg__Time__EXPECTED_HASH = {1, {
    0xb1, 0x06, 0x23, 0x5e, 0x25, 0xa4, 0xc5, 0xed,
    0x35, 0x09, 0x8a, 0xa0, 0xa6, 0x1a, 0x3e, 0xe9,
    0xc9, 0xb1, 0x8d, 0x19, 0x7f, 0x39, 0x8b, 0x0e,
    0x42, 0x06, 0xce, 0xa9, 0xac, 0xf9, 0xc1, 0x97,
  }};
#endif

static char robot_msgs__msg__ExecutorState__TYPE_NAME[] = "robot_msgs/msg/ExecutorState";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";

// Define type names, field names, and default values
static char robot_msgs__msg__ExecutorState__FIELD_NAME__stamp[] = "stamp";
static char robot_msgs__msg__ExecutorState__FIELD_NAME__robot_id[] = "robot_id";
static char robot_msgs__msg__ExecutorState__FIELD_NAME__map_id[] = "map_id";
static char robot_msgs__msg__ExecutorState__FIELD_NAME__route_active[] = "route_active";
static char robot_msgs__msg__ExecutorState__FIELD_NAME__state[] = "state";
static char robot_msgs__msg__ExecutorState__FIELD_NAME__message[] = "message";
static char robot_msgs__msg__ExecutorState__FIELD_NAME__target_lm[] = "target_lm";
static char robot_msgs__msg__ExecutorState__FIELD_NAME__current_edge_id[] = "current_edge_id";
static char robot_msgs__msg__ExecutorState__FIELD_NAME__route_id[] = "route_id";
static char robot_msgs__msg__ExecutorState__FIELD_NAME__route_progress[] = "route_progress";

static rosidl_runtime_c__type_description__Field robot_msgs__msg__ExecutorState__FIELDS[] = {
  {
    {robot_msgs__msg__ExecutorState__FIELD_NAME__stamp, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    },
    {NULL, 0, 0},
  },
  {
    {robot_msgs__msg__ExecutorState__FIELD_NAME__robot_id, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {robot_msgs__msg__ExecutorState__FIELD_NAME__map_id, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {robot_msgs__msg__ExecutorState__FIELD_NAME__route_active, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {robot_msgs__msg__ExecutorState__FIELD_NAME__state, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {robot_msgs__msg__ExecutorState__FIELD_NAME__message, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {robot_msgs__msg__ExecutorState__FIELD_NAME__target_lm, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {robot_msgs__msg__ExecutorState__FIELD_NAME__current_edge_id, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {robot_msgs__msg__ExecutorState__FIELD_NAME__route_id, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {robot_msgs__msg__ExecutorState__FIELD_NAME__route_progress, 14, 14},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription robot_msgs__msg__ExecutorState__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
robot_msgs__msg__ExecutorState__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {robot_msgs__msg__ExecutorState__TYPE_NAME, 28, 28},
      {robot_msgs__msg__ExecutorState__FIELDS, 10, 10},
    },
    {robot_msgs__msg__ExecutorState__REFERENCED_TYPE_DESCRIPTIONS, 1, 1},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "builtin_interfaces/Time stamp\n"
  "string robot_id\n"
  "string map_id\n"
  "bool route_active\n"
  "string state\n"
  "string message\n"
  "string target_lm\n"
  "string current_edge_id\n"
  "string route_id\n"
  "float32 route_progress";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
robot_msgs__msg__ExecutorState__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {robot_msgs__msg__ExecutorState__TYPE_NAME, 28, 28},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 185, 185},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
robot_msgs__msg__ExecutorState__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[2];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 2, 2};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *robot_msgs__msg__ExecutorState__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
