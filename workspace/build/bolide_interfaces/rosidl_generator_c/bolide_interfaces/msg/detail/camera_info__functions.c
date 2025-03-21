// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from bolide_interfaces:msg/CameraInfo.idl
// generated code does not contain a copyright notice
#include "bolide_interfaces/msg/detail/camera_info__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `front_color`
// Member `left_color`
// Member `right_color`
#include "rosidl_runtime_c/string_functions.h"

bool
bolide_interfaces__msg__CameraInfo__init(bolide_interfaces__msg__CameraInfo * msg)
{
  if (!msg) {
    return false;
  }
  // wrong_way
  // front_color
  if (!rosidl_runtime_c__String__init(&msg->front_color)) {
    bolide_interfaces__msg__CameraInfo__fini(msg);
    return false;
  }
  // left_color
  if (!rosidl_runtime_c__String__init(&msg->left_color)) {
    bolide_interfaces__msg__CameraInfo__fini(msg);
    return false;
  }
  // right_color
  if (!rosidl_runtime_c__String__init(&msg->right_color)) {
    bolide_interfaces__msg__CameraInfo__fini(msg);
    return false;
  }
  return true;
}

void
bolide_interfaces__msg__CameraInfo__fini(bolide_interfaces__msg__CameraInfo * msg)
{
  if (!msg) {
    return;
  }
  // wrong_way
  // front_color
  rosidl_runtime_c__String__fini(&msg->front_color);
  // left_color
  rosidl_runtime_c__String__fini(&msg->left_color);
  // right_color
  rosidl_runtime_c__String__fini(&msg->right_color);
}

bool
bolide_interfaces__msg__CameraInfo__are_equal(const bolide_interfaces__msg__CameraInfo * lhs, const bolide_interfaces__msg__CameraInfo * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // wrong_way
  if (lhs->wrong_way != rhs->wrong_way) {
    return false;
  }
  // front_color
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->front_color), &(rhs->front_color)))
  {
    return false;
  }
  // left_color
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->left_color), &(rhs->left_color)))
  {
    return false;
  }
  // right_color
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->right_color), &(rhs->right_color)))
  {
    return false;
  }
  return true;
}

bool
bolide_interfaces__msg__CameraInfo__copy(
  const bolide_interfaces__msg__CameraInfo * input,
  bolide_interfaces__msg__CameraInfo * output)
{
  if (!input || !output) {
    return false;
  }
  // wrong_way
  output->wrong_way = input->wrong_way;
  // front_color
  if (!rosidl_runtime_c__String__copy(
      &(input->front_color), &(output->front_color)))
  {
    return false;
  }
  // left_color
  if (!rosidl_runtime_c__String__copy(
      &(input->left_color), &(output->left_color)))
  {
    return false;
  }
  // right_color
  if (!rosidl_runtime_c__String__copy(
      &(input->right_color), &(output->right_color)))
  {
    return false;
  }
  return true;
}

bolide_interfaces__msg__CameraInfo *
bolide_interfaces__msg__CameraInfo__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  bolide_interfaces__msg__CameraInfo * msg = (bolide_interfaces__msg__CameraInfo *)allocator.allocate(sizeof(bolide_interfaces__msg__CameraInfo), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(bolide_interfaces__msg__CameraInfo));
  bool success = bolide_interfaces__msg__CameraInfo__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
bolide_interfaces__msg__CameraInfo__destroy(bolide_interfaces__msg__CameraInfo * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    bolide_interfaces__msg__CameraInfo__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
bolide_interfaces__msg__CameraInfo__Sequence__init(bolide_interfaces__msg__CameraInfo__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  bolide_interfaces__msg__CameraInfo * data = NULL;

  if (size) {
    data = (bolide_interfaces__msg__CameraInfo *)allocator.zero_allocate(size, sizeof(bolide_interfaces__msg__CameraInfo), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = bolide_interfaces__msg__CameraInfo__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        bolide_interfaces__msg__CameraInfo__fini(&data[i - 1]);
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
bolide_interfaces__msg__CameraInfo__Sequence__fini(bolide_interfaces__msg__CameraInfo__Sequence * array)
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
      bolide_interfaces__msg__CameraInfo__fini(&array->data[i]);
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

bolide_interfaces__msg__CameraInfo__Sequence *
bolide_interfaces__msg__CameraInfo__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  bolide_interfaces__msg__CameraInfo__Sequence * array = (bolide_interfaces__msg__CameraInfo__Sequence *)allocator.allocate(sizeof(bolide_interfaces__msg__CameraInfo__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = bolide_interfaces__msg__CameraInfo__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
bolide_interfaces__msg__CameraInfo__Sequence__destroy(bolide_interfaces__msg__CameraInfo__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    bolide_interfaces__msg__CameraInfo__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
bolide_interfaces__msg__CameraInfo__Sequence__are_equal(const bolide_interfaces__msg__CameraInfo__Sequence * lhs, const bolide_interfaces__msg__CameraInfo__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!bolide_interfaces__msg__CameraInfo__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
bolide_interfaces__msg__CameraInfo__Sequence__copy(
  const bolide_interfaces__msg__CameraInfo__Sequence * input,
  bolide_interfaces__msg__CameraInfo__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(bolide_interfaces__msg__CameraInfo);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    bolide_interfaces__msg__CameraInfo * data =
      (bolide_interfaces__msg__CameraInfo *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!bolide_interfaces__msg__CameraInfo__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          bolide_interfaces__msg__CameraInfo__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!bolide_interfaces__msg__CameraInfo__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
