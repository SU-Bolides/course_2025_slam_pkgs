// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from bolide_interfaces:msg/ForkSpeed.idl
// generated code does not contain a copyright notice
#include "bolide_interfaces/msg/detail/fork_speed__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"

bool
bolide_interfaces__msg__ForkSpeed__init(bolide_interfaces__msg__ForkSpeed * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    bolide_interfaces__msg__ForkSpeed__fini(msg);
    return false;
  }
  // speed
  return true;
}

void
bolide_interfaces__msg__ForkSpeed__fini(bolide_interfaces__msg__ForkSpeed * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // speed
}

bool
bolide_interfaces__msg__ForkSpeed__are_equal(const bolide_interfaces__msg__ForkSpeed * lhs, const bolide_interfaces__msg__ForkSpeed * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // speed
  if (lhs->speed != rhs->speed) {
    return false;
  }
  return true;
}

bool
bolide_interfaces__msg__ForkSpeed__copy(
  const bolide_interfaces__msg__ForkSpeed * input,
  bolide_interfaces__msg__ForkSpeed * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // speed
  output->speed = input->speed;
  return true;
}

bolide_interfaces__msg__ForkSpeed *
bolide_interfaces__msg__ForkSpeed__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  bolide_interfaces__msg__ForkSpeed * msg = (bolide_interfaces__msg__ForkSpeed *)allocator.allocate(sizeof(bolide_interfaces__msg__ForkSpeed), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(bolide_interfaces__msg__ForkSpeed));
  bool success = bolide_interfaces__msg__ForkSpeed__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
bolide_interfaces__msg__ForkSpeed__destroy(bolide_interfaces__msg__ForkSpeed * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    bolide_interfaces__msg__ForkSpeed__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
bolide_interfaces__msg__ForkSpeed__Sequence__init(bolide_interfaces__msg__ForkSpeed__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  bolide_interfaces__msg__ForkSpeed * data = NULL;

  if (size) {
    data = (bolide_interfaces__msg__ForkSpeed *)allocator.zero_allocate(size, sizeof(bolide_interfaces__msg__ForkSpeed), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = bolide_interfaces__msg__ForkSpeed__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        bolide_interfaces__msg__ForkSpeed__fini(&data[i - 1]);
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
bolide_interfaces__msg__ForkSpeed__Sequence__fini(bolide_interfaces__msg__ForkSpeed__Sequence * array)
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
      bolide_interfaces__msg__ForkSpeed__fini(&array->data[i]);
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

bolide_interfaces__msg__ForkSpeed__Sequence *
bolide_interfaces__msg__ForkSpeed__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  bolide_interfaces__msg__ForkSpeed__Sequence * array = (bolide_interfaces__msg__ForkSpeed__Sequence *)allocator.allocate(sizeof(bolide_interfaces__msg__ForkSpeed__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = bolide_interfaces__msg__ForkSpeed__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
bolide_interfaces__msg__ForkSpeed__Sequence__destroy(bolide_interfaces__msg__ForkSpeed__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    bolide_interfaces__msg__ForkSpeed__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
bolide_interfaces__msg__ForkSpeed__Sequence__are_equal(const bolide_interfaces__msg__ForkSpeed__Sequence * lhs, const bolide_interfaces__msg__ForkSpeed__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!bolide_interfaces__msg__ForkSpeed__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
bolide_interfaces__msg__ForkSpeed__Sequence__copy(
  const bolide_interfaces__msg__ForkSpeed__Sequence * input,
  bolide_interfaces__msg__ForkSpeed__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(bolide_interfaces__msg__ForkSpeed);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    bolide_interfaces__msg__ForkSpeed * data =
      (bolide_interfaces__msg__ForkSpeed *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!bolide_interfaces__msg__ForkSpeed__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          bolide_interfaces__msg__ForkSpeed__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!bolide_interfaces__msg__ForkSpeed__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
