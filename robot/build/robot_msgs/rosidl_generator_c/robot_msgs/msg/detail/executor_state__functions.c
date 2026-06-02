// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from robot_msgs:msg/ExecutorState.idl
// generated code does not contain a copyright notice
#include "robot_msgs/msg/detail/executor_state__functions.h"

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
// Member `current_edge_id`
// Member `route_id`
#include "rosidl_runtime_c/string_functions.h"

bool
robot_msgs__msg__ExecutorState__init(robot_msgs__msg__ExecutorState * msg)
{
  if (!msg) {
    return false;
  }
  // stamp
  if (!builtin_interfaces__msg__Time__init(&msg->stamp)) {
    robot_msgs__msg__ExecutorState__fini(msg);
    return false;
  }
  // robot_id
  if (!rosidl_runtime_c__String__init(&msg->robot_id)) {
    robot_msgs__msg__ExecutorState__fini(msg);
    return false;
  }
  // map_id
  if (!rosidl_runtime_c__String__init(&msg->map_id)) {
    robot_msgs__msg__ExecutorState__fini(msg);
    return false;
  }
  // route_active
  // state
  if (!rosidl_runtime_c__String__init(&msg->state)) {
    robot_msgs__msg__ExecutorState__fini(msg);
    return false;
  }
  // message
  if (!rosidl_runtime_c__String__init(&msg->message)) {
    robot_msgs__msg__ExecutorState__fini(msg);
    return false;
  }
  // target_lm
  if (!rosidl_runtime_c__String__init(&msg->target_lm)) {
    robot_msgs__msg__ExecutorState__fini(msg);
    return false;
  }
  // current_edge_id
  if (!rosidl_runtime_c__String__init(&msg->current_edge_id)) {
    robot_msgs__msg__ExecutorState__fini(msg);
    return false;
  }
  // route_id
  if (!rosidl_runtime_c__String__init(&msg->route_id)) {
    robot_msgs__msg__ExecutorState__fini(msg);
    return false;
  }
  // route_progress
  return true;
}

void
robot_msgs__msg__ExecutorState__fini(robot_msgs__msg__ExecutorState * msg)
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
  // route_active
  // state
  rosidl_runtime_c__String__fini(&msg->state);
  // message
  rosidl_runtime_c__String__fini(&msg->message);
  // target_lm
  rosidl_runtime_c__String__fini(&msg->target_lm);
  // current_edge_id
  rosidl_runtime_c__String__fini(&msg->current_edge_id);
  // route_id
  rosidl_runtime_c__String__fini(&msg->route_id);
  // route_progress
}

bool
robot_msgs__msg__ExecutorState__are_equal(const robot_msgs__msg__ExecutorState * lhs, const robot_msgs__msg__ExecutorState * rhs)
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
  // route_active
  if (lhs->route_active != rhs->route_active) {
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
  return true;
}

bool
robot_msgs__msg__ExecutorState__copy(
  const robot_msgs__msg__ExecutorState * input,
  robot_msgs__msg__ExecutorState * output)
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
  // route_active
  output->route_active = input->route_active;
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
  return true;
}

robot_msgs__msg__ExecutorState *
robot_msgs__msg__ExecutorState__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robot_msgs__msg__ExecutorState * msg = (robot_msgs__msg__ExecutorState *)allocator.allocate(sizeof(robot_msgs__msg__ExecutorState), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(robot_msgs__msg__ExecutorState));
  bool success = robot_msgs__msg__ExecutorState__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
robot_msgs__msg__ExecutorState__destroy(robot_msgs__msg__ExecutorState * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    robot_msgs__msg__ExecutorState__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
robot_msgs__msg__ExecutorState__Sequence__init(robot_msgs__msg__ExecutorState__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robot_msgs__msg__ExecutorState * data = NULL;

  if (size) {
    data = (robot_msgs__msg__ExecutorState *)allocator.zero_allocate(size, sizeof(robot_msgs__msg__ExecutorState), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = robot_msgs__msg__ExecutorState__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        robot_msgs__msg__ExecutorState__fini(&data[i - 1]);
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
robot_msgs__msg__ExecutorState__Sequence__fini(robot_msgs__msg__ExecutorState__Sequence * array)
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
      robot_msgs__msg__ExecutorState__fini(&array->data[i]);
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

robot_msgs__msg__ExecutorState__Sequence *
robot_msgs__msg__ExecutorState__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robot_msgs__msg__ExecutorState__Sequence * array = (robot_msgs__msg__ExecutorState__Sequence *)allocator.allocate(sizeof(robot_msgs__msg__ExecutorState__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = robot_msgs__msg__ExecutorState__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
robot_msgs__msg__ExecutorState__Sequence__destroy(robot_msgs__msg__ExecutorState__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    robot_msgs__msg__ExecutorState__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
robot_msgs__msg__ExecutorState__Sequence__are_equal(const robot_msgs__msg__ExecutorState__Sequence * lhs, const robot_msgs__msg__ExecutorState__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!robot_msgs__msg__ExecutorState__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
robot_msgs__msg__ExecutorState__Sequence__copy(
  const robot_msgs__msg__ExecutorState__Sequence * input,
  robot_msgs__msg__ExecutorState__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(robot_msgs__msg__ExecutorState);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    robot_msgs__msg__ExecutorState * data =
      (robot_msgs__msg__ExecutorState *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!robot_msgs__msg__ExecutorState__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          robot_msgs__msg__ExecutorState__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!robot_msgs__msg__ExecutorState__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
