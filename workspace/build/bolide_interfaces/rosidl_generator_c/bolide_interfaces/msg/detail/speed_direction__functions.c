// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from bolide_interfaces:msg/SpeedDirection.idl
// generated code does not contain a copyright notice
#include "bolide_interfaces/msg/detail/speed_direction__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
bolide_interfaces__msg__SpeedDirection__init(bolide_interfaces__msg__SpeedDirection * msg)
{
  if (!msg) {
    return false;
  }
  // speed
  // direction
  return true;
}

void
bolide_interfaces__msg__SpeedDirection__fini(bolide_interfaces__msg__SpeedDirection * msg)
{
  if (!msg) {
    return;
  }
  // speed
  // direction
}

bool
bolide_interfaces__msg__SpeedDirection__are_equal(const bolide_interfaces__msg__SpeedDirection * lhs, const bolide_interfaces__msg__SpeedDirection * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // speed
  if (lhs->speed != rhs->speed) {
    return false;
  }
  // direction
  if (lhs->direction != rhs->direction) {
    return false;
  }
  return true;
}

bool
bolide_interfaces__msg__SpeedDirection__copy(
  const bolide_interfaces__msg__SpeedDirection * input,
  bolide_interfaces__msg__SpeedDirection * output)
{
  if (!input || !output) {
    return false;
  }
  // speed
  output->speed = input->speed;
  // direction
  output->direction = input->direction;
  return true;
}

bolide_interfaces__msg__SpeedDirection *
bolide_interfaces__msg__SpeedDirection__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  bolide_interfaces__msg__SpeedDirection * msg = (bolide_interfaces__msg__SpeedDirection *)allocator.allocate(sizeof(bolide_interfaces__msg__SpeedDirection), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(bolide_interfaces__msg__SpeedDirection));
  bool success = bolide_interfaces__msg__SpeedDirection__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
bolide_interfaces__msg__SpeedDirection__destroy(bolide_interfaces__msg__SpeedDirection * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    bolide_interfaces__msg__SpeedDirection__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
bolide_interfaces__msg__SpeedDirection__Sequence__init(bolide_interfaces__msg__SpeedDirection__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  bolide_interfaces__msg__SpeedDirection * data = NULL;

  if (size) {
    data = (bolide_interfaces__msg__SpeedDirection *)allocator.zero_allocate(size, sizeof(bolide_interfaces__msg__SpeedDirection), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = bolide_interfaces__msg__SpeedDirection__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        bolide_interfaces__msg__SpeedDirection__fini(&data[i - 1]);
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
bolide_interfaces__msg__SpeedDirection__Sequence__fini(bolide_interfaces__msg__SpeedDirection__Sequence * array)
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
      bolide_interfaces__msg__SpeedDirection__fini(&array->data[i]);
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

bolide_interfaces__msg__SpeedDirection__Sequence *
bolide_interfaces__msg__SpeedDirection__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  bolide_interfaces__msg__SpeedDirection__Sequence * array = (bolide_interfaces__msg__SpeedDirection__Sequence *)allocator.allocate(sizeof(bolide_interfaces__msg__SpeedDirection__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = bolide_interfaces__msg__SpeedDirection__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
bolide_interfaces__msg__SpeedDirection__Sequence__destroy(bolide_interfaces__msg__SpeedDirection__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    bolide_interfaces__msg__SpeedDirection__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
bolide_interfaces__msg__SpeedDirection__Sequence__are_equal(const bolide_interfaces__msg__SpeedDirection__Sequence * lhs, const bolide_interfaces__msg__SpeedDirection__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!bolide_interfaces__msg__SpeedDirection__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
bolide_interfaces__msg__SpeedDirection__Sequence__copy(
  const bolide_interfaces__msg__SpeedDirection__Sequence * input,
  bolide_interfaces__msg__SpeedDirection__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(bolide_interfaces__msg__SpeedDirection);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    bolide_interfaces__msg__SpeedDirection * data =
      (bolide_interfaces__msg__SpeedDirection *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!bolide_interfaces__msg__SpeedDirection__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          bolide_interfaces__msg__SpeedDirection__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!bolide_interfaces__msg__SpeedDirection__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
