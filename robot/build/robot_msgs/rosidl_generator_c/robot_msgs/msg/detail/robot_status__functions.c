// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from robot_msgs:msg/RobotStatus.idl
// generated code does not contain a copyright notice
#include "robot_msgs/msg/detail/robot_status__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `stamp`
#include "builtin_interfaces/msg/detail/time__functions.h"
// Member `robot_id`
// Member `map_id`
// Member `state`
// Member `message`
// Member `target_lm`
// Member `nearest_lm`
// Member `current_edge_id`
// Member `route_id`
#include "rosidl_runtime_c/string_functions.h"

bool
robot_msgs__msg__RobotStatus__init(robot_msgs__msg__RobotStatus * msg)
{
  if (!msg) {
    return false;
  }
  // stamp
  if (!builtin_interfaces__msg__Time__init(&msg->stamp)) {
    robot_msgs__msg__RobotStatus__fini(msg);
    return false;
  }
  // robot_id
  if (!rosidl_runtime_c__String__init(&msg->robot_id)) {
    robot_msgs__msg__RobotStatus__fini(msg);
    return false;
  }
  // map_id
  if (!rosidl_runtime_c__String__init(&msg->map_id)) {
    robot_msgs__msg__RobotStatus__fini(msg);
    return false;
  }
  // connected
  // localization_ok
  // localization_age_sec
  // state
  if (!rosidl_runtime_c__String__init(&msg->state)) {
    robot_msgs__msg__RobotStatus__fini(msg);
    return false;
  }
  // message
  if (!rosidl_runtime_c__String__init(&msg->message)) {
    robot_msgs__msg__RobotStatus__fini(msg);
    return false;
  }
  // target_lm
  if (!rosidl_runtime_c__String__init(&msg->target_lm)) {
    robot_msgs__msg__RobotStatus__fini(msg);
    return false;
  }
  // nearest_lm
  if (!rosidl_runtime_c__String__init(&msg->nearest_lm)) {
    robot_msgs__msg__RobotStatus__fini(msg);
    return false;
  }
  // current_edge_id
  if (!rosidl_runtime_c__String__init(&msg->current_edge_id)) {
    robot_msgs__msg__RobotStatus__fini(msg);
    return false;
  }
  // route_id
  if (!rosidl_runtime_c__String__init(&msg->route_id)) {
    robot_msgs__msg__RobotStatus__fini(msg);
    return false;
  }
  // route_progress
  // pose_x
  // pose_y
  // pose_yaw
  // linear_velocity
  // angular_velocity
  return true;
}

void
robot_msgs__msg__RobotStatus__fini(robot_msgs__msg__RobotStatus * msg)
{
  if (!msg) {
    return;
  }
  // stamp
  builtin_interfaces__msg__Time__fini(&msg->stamp);
  // robot_id
  rosidl_runtime_c__String__fini(&msg->robot_id);
  // map_id
  rosidl_runtime_c__String__fini(&msg->map_id);
  // connected
  // localization_ok
  // localization_age_sec
  // state
  rosidl_runtime_c__String__fini(&msg->state);
  // message
  rosidl_runtime_c__String__fini(&msg->message);
  // target_lm
  rosidl_runtime_c__String__fini(&msg->target_lm);
  // nearest_lm
  rosidl_runtime_c__String__fini(&msg->nearest_lm);
  // current_edge_id
  rosidl_runtime_c__String__fini(&msg->current_edge_id);
  // route_id
  rosidl_runtime_c__String__fini(&msg->route_id);
  // route_progress
  // pose_x
  // pose_y
  // pose_yaw
  // linear_velocity
  // angular_velocity
}

bool
robot_msgs__msg__RobotStatus__are_equal(const robot_msgs__msg__RobotStatus * lhs, const robot_msgs__msg__RobotStatus * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // stamp
  if (!builtin_interfaces__msg__Time__are_equal(
      &(lhs->stamp), &(rhs->stamp)))
  {
    return false;
  }
  // robot_id
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->robot_id), &(rhs->robot_id)))
  {
    return false;
  }
  // map_id
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->map_id), &(rhs->map_id)))
  {
    return false;
  }
  // connected
  if (lhs->connected != rhs->connected) {
    return false;
  }
  // localization_ok
  if (lhs->localization_ok != rhs->localization_ok) {
    return false;
  }
  // localization_age_sec
  if (lhs->localization_age_sec != rhs->localization_age_sec) {
    return false;
  }
  // state
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->state), &(rhs->state)))
  {
    return false;
  }
  // message
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->message), &(rhs->message)))
  {
    return false;
  }
  // target_lm
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->target_lm), &(rhs->target_lm)))
  {
    return false;
  }
  // nearest_lm
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->nearest_lm), &(rhs->nearest_lm)))
  {
    return false;
  }
  // current_edge_id
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->current_edge_id), &(rhs->current_edge_id)))
  {
    return false;
  }
  // route_id
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->route_id), &(rhs->route_id)))
  {
    return false;
  }
  // route_progress
  if (lhs->route_progress != rhs->route_progress) {
    return false;
  }
  // pose_x
  if (lhs->pose_x != rhs->pose_x) {
    return false;
  }
  // pose_y
  if (lhs->pose_y != rhs->pose_y) {
    return false;
  }
  // pose_yaw
  if (lhs->pose_yaw != rhs->pose_yaw) {
    return false;
  }
  // linear_velocity
  if (lhs->linear_velocity != rhs->linear_velocity) {
    return false;
  }
  // angular_velocity
  if (lhs->angular_velocity != rhs->angular_velocity) {
    return false;
  }
  return true;
}

bool
robot_msgs__msg__RobotStatus__copy(
  const robot_msgs__msg__RobotStatus * input,
  robot_msgs__msg__RobotStatus * output)
{
  if (!input || !output) {
    return false;
  }
  // stamp
  if (!builtin_interfaces__msg__Time__copy(
      &(input->stamp), &(output->stamp)))
  {
    return false;
  }
  // robot_id
  if (!rosidl_runtime_c__String__copy(
      &(input->robot_id), &(output->robot_id)))
  {
    return false;
  }
  // map_id
  if (!rosidl_runtime_c__String__copy(
      &(input->map_id), &(output->map_id)))
  {
    return false;
  }
  // connected
  output->connected = input->connected;
  // localization_ok
  output->localization_ok = input->localization_ok;
  // localization_age_sec
  output->localization_age_sec = input->localization_age_sec;
  // state
  if (!rosidl_runtime_c__String__copy(
      &(input->state), &(output->state)))
  {
    return false;
  }
  // message
  if (!rosidl_runtime_c__String__copy(
      &(input->message), &(output->message)))
  {
    return false;
  }
  // target_lm
  if (!rosidl_runtime_c__String__copy(
      &(input->target_lm), &(output->target_lm)))
  {
    return false;
  }
  // nearest_lm
  if (!rosidl_runtime_c__String__copy(
      &(input->nearest_lm), &(output->nearest_lm)))
  {
    return false;
  }
  // current_edge_id
  if (!rosidl_runtime_c__String__copy(
      &(input->current_edge_id), &(output->current_edge_id)))
  {
    return false;
  }
  // route_id
  if (!rosidl_runtime_c__String__copy(
      &(input->route_id), &(output->route_id)))
  {
    return false;
  }
  // route_progress
  output->route_progress = input->route_progress;
  // pose_x
  output->pose_x = input->pose_x;
  // pose_y
  output->pose_y = input->pose_y;
  // pose_yaw
  output->pose_yaw = input->pose_yaw;
  // linear_velocity
  output->linear_velocity = input->linear_velocity;
  // angular_velocity
  output->angular_velocity = input->angular_velocity;
  return true;
}

robot_msgs__msg__RobotStatus *
robot_msgs__msg__RobotStatus__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robot_msgs__msg__RobotStatus * msg = (robot_msgs__msg__RobotStatus *)allocator.allocate(sizeof(robot_msgs__msg__RobotStatus), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(robot_msgs__msg__RobotStatus));
  bool success = robot_msgs__msg__RobotStatus__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
robot_msgs__msg__RobotStatus__destroy(robot_msgs__msg__RobotStatus * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    robot_msgs__msg__RobotStatus__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
robot_msgs__msg__RobotStatus__Sequence__init(robot_msgs__msg__RobotStatus__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robot_msgs__msg__RobotStatus * data = NULL;

  if (size) {
    data = (robot_msgs__msg__RobotStatus *)allocator.zero_allocate(size, sizeof(robot_msgs__msg__RobotStatus), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = robot_msgs__msg__RobotStatus__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        robot_msgs__msg__RobotStatus__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
robot_msgs__msg__RobotStatus__Sequence__fini(robot_msgs__msg__RobotStatus__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      robot_msgs__msg__RobotStatus__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

robot_msgs__msg__RobotStatus__Sequence *
robot_msgs__msg__RobotStatus__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robot_msgs__msg__RobotStatus__Sequence * array = (robot_msgs__msg__RobotStatus__Sequence *)allocator.allocate(sizeof(robot_msgs__msg__RobotStatus__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = robot_msgs__msg__RobotStatus__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
robot_msgs__msg__RobotStatus__Sequence__destroy(robot_msgs__msg__RobotStatus__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    robot_msgs__msg__RobotStatus__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
robot_msgs__msg__RobotStatus__Sequence__are_equal(const robot_msgs__msg__RobotStatus__Sequence * lhs, const robot_msgs__msg__RobotStatus__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!robot_msgs__msg__RobotStatus__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
robot_msgs__msg__RobotStatus__Sequence__copy(
  const robot_msgs__msg__RobotStatus__Sequence * input,
  robot_msgs__msg__RobotStatus__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(robot_msgs__msg__RobotStatus);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    robot_msgs__msg__RobotStatus * data =
      (robot_msgs__msg__RobotStatus *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!robot_msgs__msg__RobotStatus__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          robot_msgs__msg__RobotStatus__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!robot_msgs__msg__RobotStatus__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
