// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from bolide_interfaces:msg/MultipleRange.idl
// generated code does not contain a copyright notice
#include "bolide_interfaces/msg/detail/multiple_range__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `ir_rear_left`
// Member `ir_rear_right`
// Member `sonar_rear`
#include "sensor_msgs/msg/detail/range__functions.h"

bool
bolide_interfaces__msg__MultipleRange__init(bolide_interfaces__msg__MultipleRange * msg)
{
  if (!msg) {
    return false;
  }
  // ir_rear_left
  if (!sensor_msgs__msg__Range__init(&msg->ir_rear_left)) {
    bolide_interfaces__msg__MultipleRange__fini(msg);
    return false;
  }
  // ir_rear_right
  if (!sensor_msgs__msg__Range__init(&msg->ir_rear_right)) {
    bolide_interfaces__msg__MultipleRange__fini(msg);
    return false;
  }
  // sonar_rear
  if (!sensor_msgs__msg__Range__init(&msg->sonar_rear)) {
    bolide_interfaces__msg__MultipleRange__fini(msg);
    return false;
  }
  return true;
}

void
bolide_interfaces__msg__MultipleRange__fini(bolide_interfaces__msg__MultipleRange * msg)
{
  if (!msg) {
    return;
  }
  // ir_rear_left
  sensor_msgs__msg__Range__fini(&msg->ir_rear_left);
  // ir_rear_right
  sensor_msgs__msg__Range__fini(&msg->ir_rear_right);
  // sonar_rear
  sensor_msgs__msg__Range__fini(&msg->sonar_rear);
}

bool
bolide_interfaces__msg__MultipleRange__are_equal(const bolide_interfaces__msg__MultipleRange * lhs, const bolide_interfaces__msg__MultipleRange * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // ir_rear_left
  if (!sensor_msgs__msg__Range__are_equal(
      &(lhs->ir_rear_left), &(rhs->ir_rear_left)))
  {
    return false;
  }
  // ir_rear_right
  if (!sensor_msgs__msg__Range__are_equal(
      &(lhs->ir_rear_right), &(rhs->ir_rear_right)))
  {
    return false;
  }
  // sonar_rear
  if (!sensor_msgs__msg__Range__are_equal(
      &(lhs->sonar_rear), &(rhs->sonar_rear)))
  {
    return false;
  }
  return true;
}

bool
bolide_interfaces__msg__MultipleRange__copy(
  const bolide_interfaces__msg__MultipleRange * input,
  bolide_interfaces__msg__MultipleRange * output)
{
  if (!input || !output) {
    return false;
  }
  // ir_rear_left
  if (!sensor_msgs__msg__Range__copy(
      &(input->ir_rear_left), &(output->ir_rear_left)))
  {
    return false;
  }
  // ir_rear_right
  if (!sensor_msgs__msg__Range__copy(
      &(input->ir_rear_right), &(output->ir_rear_right)))
  {
    return false;
  }
  // sonar_rear
  if (!sensor_msgs__msg__Range__copy(
      &(input->sonar_rear), &(output->sonar_rear)))
  {
    return false;
  }
  return true;
}

bolide_interfaces__msg__MultipleRange *
bolide_interfaces__msg__MultipleRange__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  bolide_interfaces__msg__MultipleRange * msg = (bolide_interfaces__msg__MultipleRange *)allocator.allocate(sizeof(bolide_interfaces__msg__MultipleRange), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(bolide_interfaces__msg__MultipleRange));
  bool success = bolide_interfaces__msg__MultipleRange__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
bolide_interfaces__msg__MultipleRange__destroy(bolide_interfaces__msg__MultipleRange * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    bolide_interfaces__msg__MultipleRange__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
bolide_interfaces__msg__MultipleRange__Sequence__init(bolide_interfaces__msg__MultipleRange__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  bolide_interfaces__msg__MultipleRange * data = NULL;

  if (size) {
    data = (bolide_interfaces__msg__MultipleRange *)allocator.zero_allocate(size, sizeof(bolide_interfaces__msg__MultipleRange), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = bolide_interfaces__msg__MultipleRange__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        bolide_interfaces__msg__MultipleRange__fini(&data[i - 1]);
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
bolide_interfaces__msg__MultipleRange__Sequence__fini(bolide_interfaces__msg__MultipleRange__Sequence * array)
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
      bolide_interfaces__msg__MultipleRange__fini(&array->data[i]);
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

bolide_interfaces__msg__MultipleRange__Sequence *
bolide_interfaces__msg__MultipleRange__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  bolide_interfaces__msg__MultipleRange__Sequence * array = (bolide_interfaces__msg__MultipleRange__Sequence *)allocator.allocate(sizeof(bolide_interfaces__msg__MultipleRange__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = bolide_interfaces__msg__MultipleRange__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
bolide_interfaces__msg__MultipleRange__Sequence__destroy(bolide_interfaces__msg__MultipleRange__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    bolide_interfaces__msg__MultipleRange__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
bolide_interfaces__msg__MultipleRange__Sequence__are_equal(const bolide_interfaces__msg__MultipleRange__Sequence * lhs, const bolide_interfaces__msg__MultipleRange__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!bolide_interfaces__msg__MultipleRange__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
bolide_interfaces__msg__MultipleRange__Sequence__copy(
  const bolide_interfaces__msg__MultipleRange__Sequence * input,
  bolide_interfaces__msg__MultipleRange__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(bolide_interfaces__msg__MultipleRange);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    bolide_interfaces__msg__MultipleRange * data =
      (bolide_interfaces__msg__MultipleRange *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!bolide_interfaces__msg__MultipleRange__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          bolide_interfaces__msg__MultipleRange__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!bolide_interfaces__msg__MultipleRange__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
