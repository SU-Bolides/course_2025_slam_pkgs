// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from bolide_interfaces:srv/LaunchEkf.idl
// generated code does not contain a copyright notice
#include "bolide_interfaces/srv/detail/launch_ekf__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

bool
bolide_interfaces__srv__LaunchEkf_Request__init(bolide_interfaces__srv__LaunchEkf_Request * msg)
{
  if (!msg) {
    return false;
  }
  // start
  return true;
}

void
bolide_interfaces__srv__LaunchEkf_Request__fini(bolide_interfaces__srv__LaunchEkf_Request * msg)
{
  if (!msg) {
    return;
  }
  // start
}

bool
bolide_interfaces__srv__LaunchEkf_Request__are_equal(const bolide_interfaces__srv__LaunchEkf_Request * lhs, const bolide_interfaces__srv__LaunchEkf_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // start
  if (lhs->start != rhs->start) {
    return false;
  }
  return true;
}

bool
bolide_interfaces__srv__LaunchEkf_Request__copy(
  const bolide_interfaces__srv__LaunchEkf_Request * input,
  bolide_interfaces__srv__LaunchEkf_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // start
  output->start = input->start;
  return true;
}

bolide_interfaces__srv__LaunchEkf_Request *
bolide_interfaces__srv__LaunchEkf_Request__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  bolide_interfaces__srv__LaunchEkf_Request * msg = (bolide_interfaces__srv__LaunchEkf_Request *)allocator.allocate(sizeof(bolide_interfaces__srv__LaunchEkf_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(bolide_interfaces__srv__LaunchEkf_Request));
  bool success = bolide_interfaces__srv__LaunchEkf_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
bolide_interfaces__srv__LaunchEkf_Request__destroy(bolide_interfaces__srv__LaunchEkf_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    bolide_interfaces__srv__LaunchEkf_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
bolide_interfaces__srv__LaunchEkf_Request__Sequence__init(bolide_interfaces__srv__LaunchEkf_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  bolide_interfaces__srv__LaunchEkf_Request * data = NULL;

  if (size) {
    data = (bolide_interfaces__srv__LaunchEkf_Request *)allocator.zero_allocate(size, sizeof(bolide_interfaces__srv__LaunchEkf_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = bolide_interfaces__srv__LaunchEkf_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        bolide_interfaces__srv__LaunchEkf_Request__fini(&data[i - 1]);
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
bolide_interfaces__srv__LaunchEkf_Request__Sequence__fini(bolide_interfaces__srv__LaunchEkf_Request__Sequence * array)
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
      bolide_interfaces__srv__LaunchEkf_Request__fini(&array->data[i]);
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

bolide_interfaces__srv__LaunchEkf_Request__Sequence *
bolide_interfaces__srv__LaunchEkf_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  bolide_interfaces__srv__LaunchEkf_Request__Sequence * array = (bolide_interfaces__srv__LaunchEkf_Request__Sequence *)allocator.allocate(sizeof(bolide_interfaces__srv__LaunchEkf_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = bolide_interfaces__srv__LaunchEkf_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
bolide_interfaces__srv__LaunchEkf_Request__Sequence__destroy(bolide_interfaces__srv__LaunchEkf_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    bolide_interfaces__srv__LaunchEkf_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
bolide_interfaces__srv__LaunchEkf_Request__Sequence__are_equal(const bolide_interfaces__srv__LaunchEkf_Request__Sequence * lhs, const bolide_interfaces__srv__LaunchEkf_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!bolide_interfaces__srv__LaunchEkf_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
bolide_interfaces__srv__LaunchEkf_Request__Sequence__copy(
  const bolide_interfaces__srv__LaunchEkf_Request__Sequence * input,
  bolide_interfaces__srv__LaunchEkf_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(bolide_interfaces__srv__LaunchEkf_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    bolide_interfaces__srv__LaunchEkf_Request * data =
      (bolide_interfaces__srv__LaunchEkf_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!bolide_interfaces__srv__LaunchEkf_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          bolide_interfaces__srv__LaunchEkf_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!bolide_interfaces__srv__LaunchEkf_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


bool
bolide_interfaces__srv__LaunchEkf_Response__init(bolide_interfaces__srv__LaunchEkf_Response * msg)
{
  if (!msg) {
    return false;
  }
  // gotcha
  return true;
}

void
bolide_interfaces__srv__LaunchEkf_Response__fini(bolide_interfaces__srv__LaunchEkf_Response * msg)
{
  if (!msg) {
    return;
  }
  // gotcha
}

bool
bolide_interfaces__srv__LaunchEkf_Response__are_equal(const bolide_interfaces__srv__LaunchEkf_Response * lhs, const bolide_interfaces__srv__LaunchEkf_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // gotcha
  if (lhs->gotcha != rhs->gotcha) {
    return false;
  }
  return true;
}

bool
bolide_interfaces__srv__LaunchEkf_Response__copy(
  const bolide_interfaces__srv__LaunchEkf_Response * input,
  bolide_interfaces__srv__LaunchEkf_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // gotcha
  output->gotcha = input->gotcha;
  return true;
}

bolide_interfaces__srv__LaunchEkf_Response *
bolide_interfaces__srv__LaunchEkf_Response__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  bolide_interfaces__srv__LaunchEkf_Response * msg = (bolide_interfaces__srv__LaunchEkf_Response *)allocator.allocate(sizeof(bolide_interfaces__srv__LaunchEkf_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(bolide_interfaces__srv__LaunchEkf_Response));
  bool success = bolide_interfaces__srv__LaunchEkf_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
bolide_interfaces__srv__LaunchEkf_Response__destroy(bolide_interfaces__srv__LaunchEkf_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    bolide_interfaces__srv__LaunchEkf_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
bolide_interfaces__srv__LaunchEkf_Response__Sequence__init(bolide_interfaces__srv__LaunchEkf_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  bolide_interfaces__srv__LaunchEkf_Response * data = NULL;

  if (size) {
    data = (bolide_interfaces__srv__LaunchEkf_Response *)allocator.zero_allocate(size, sizeof(bolide_interfaces__srv__LaunchEkf_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = bolide_interfaces__srv__LaunchEkf_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        bolide_interfaces__srv__LaunchEkf_Response__fini(&data[i - 1]);
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
bolide_interfaces__srv__LaunchEkf_Response__Sequence__fini(bolide_interfaces__srv__LaunchEkf_Response__Sequence * array)
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
      bolide_interfaces__srv__LaunchEkf_Response__fini(&array->data[i]);
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

bolide_interfaces__srv__LaunchEkf_Response__Sequence *
bolide_interfaces__srv__LaunchEkf_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  bolide_interfaces__srv__LaunchEkf_Response__Sequence * array = (bolide_interfaces__srv__LaunchEkf_Response__Sequence *)allocator.allocate(sizeof(bolide_interfaces__srv__LaunchEkf_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = bolide_interfaces__srv__LaunchEkf_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
bolide_interfaces__srv__LaunchEkf_Response__Sequence__destroy(bolide_interfaces__srv__LaunchEkf_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    bolide_interfaces__srv__LaunchEkf_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
bolide_interfaces__srv__LaunchEkf_Response__Sequence__are_equal(const bolide_interfaces__srv__LaunchEkf_Response__Sequence * lhs, const bolide_interfaces__srv__LaunchEkf_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!bolide_interfaces__srv__LaunchEkf_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
bolide_interfaces__srv__LaunchEkf_Response__Sequence__copy(
  const bolide_interfaces__srv__LaunchEkf_Response__Sequence * input,
  bolide_interfaces__srv__LaunchEkf_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(bolide_interfaces__srv__LaunchEkf_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    bolide_interfaces__srv__LaunchEkf_Response * data =
      (bolide_interfaces__srv__LaunchEkf_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!bolide_interfaces__srv__LaunchEkf_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          bolide_interfaces__srv__LaunchEkf_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!bolide_interfaces__srv__LaunchEkf_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `info`
#include "service_msgs/msg/detail/service_event_info__functions.h"
// Member `request`
// Member `response`
// already included above
// #include "bolide_interfaces/srv/detail/launch_ekf__functions.h"

bool
bolide_interfaces__srv__LaunchEkf_Event__init(bolide_interfaces__srv__LaunchEkf_Event * msg)
{
  if (!msg) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__init(&msg->info)) {
    bolide_interfaces__srv__LaunchEkf_Event__fini(msg);
    return false;
  }
  // request
  if (!bolide_interfaces__srv__LaunchEkf_Request__Sequence__init(&msg->request, 0)) {
    bolide_interfaces__srv__LaunchEkf_Event__fini(msg);
    return false;
  }
  // response
  if (!bolide_interfaces__srv__LaunchEkf_Response__Sequence__init(&msg->response, 0)) {
    bolide_interfaces__srv__LaunchEkf_Event__fini(msg);
    return false;
  }
  return true;
}

void
bolide_interfaces__srv__LaunchEkf_Event__fini(bolide_interfaces__srv__LaunchEkf_Event * msg)
{
  if (!msg) {
    return;
  }
  // info
  service_msgs__msg__ServiceEventInfo__fini(&msg->info);
  // request
  bolide_interfaces__srv__LaunchEkf_Request__Sequence__fini(&msg->request);
  // response
  bolide_interfaces__srv__LaunchEkf_Response__Sequence__fini(&msg->response);
}

bool
bolide_interfaces__srv__LaunchEkf_Event__are_equal(const bolide_interfaces__srv__LaunchEkf_Event * lhs, const bolide_interfaces__srv__LaunchEkf_Event * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__are_equal(
      &(lhs->info), &(rhs->info)))
  {
    return false;
  }
  // request
  if (!bolide_interfaces__srv__LaunchEkf_Request__Sequence__are_equal(
      &(lhs->request), &(rhs->request)))
  {
    return false;
  }
  // response
  if (!bolide_interfaces__srv__LaunchEkf_Response__Sequence__are_equal(
      &(lhs->response), &(rhs->response)))
  {
    return false;
  }
  return true;
}

bool
bolide_interfaces__srv__LaunchEkf_Event__copy(
  const bolide_interfaces__srv__LaunchEkf_Event * input,
  bolide_interfaces__srv__LaunchEkf_Event * output)
{
  if (!input || !output) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__copy(
      &(input->info), &(output->info)))
  {
    return false;
  }
  // request
  if (!bolide_interfaces__srv__LaunchEkf_Request__Sequence__copy(
      &(input->request), &(output->request)))
  {
    return false;
  }
  // response
  if (!bolide_interfaces__srv__LaunchEkf_Response__Sequence__copy(
      &(input->response), &(output->response)))
  {
    return false;
  }
  return true;
}

bolide_interfaces__srv__LaunchEkf_Event *
bolide_interfaces__srv__LaunchEkf_Event__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  bolide_interfaces__srv__LaunchEkf_Event * msg = (bolide_interfaces__srv__LaunchEkf_Event *)allocator.allocate(sizeof(bolide_interfaces__srv__LaunchEkf_Event), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(bolide_interfaces__srv__LaunchEkf_Event));
  bool success = bolide_interfaces__srv__LaunchEkf_Event__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
bolide_interfaces__srv__LaunchEkf_Event__destroy(bolide_interfaces__srv__LaunchEkf_Event * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    bolide_interfaces__srv__LaunchEkf_Event__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
bolide_interfaces__srv__LaunchEkf_Event__Sequence__init(bolide_interfaces__srv__LaunchEkf_Event__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  bolide_interfaces__srv__LaunchEkf_Event * data = NULL;

  if (size) {
    data = (bolide_interfaces__srv__LaunchEkf_Event *)allocator.zero_allocate(size, sizeof(bolide_interfaces__srv__LaunchEkf_Event), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = bolide_interfaces__srv__LaunchEkf_Event__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        bolide_interfaces__srv__LaunchEkf_Event__fini(&data[i - 1]);
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
bolide_interfaces__srv__LaunchEkf_Event__Sequence__fini(bolide_interfaces__srv__LaunchEkf_Event__Sequence * array)
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
      bolide_interfaces__srv__LaunchEkf_Event__fini(&array->data[i]);
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

bolide_interfaces__srv__LaunchEkf_Event__Sequence *
bolide_interfaces__srv__LaunchEkf_Event__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  bolide_interfaces__srv__LaunchEkf_Event__Sequence * array = (bolide_interfaces__srv__LaunchEkf_Event__Sequence *)allocator.allocate(sizeof(bolide_interfaces__srv__LaunchEkf_Event__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = bolide_interfaces__srv__LaunchEkf_Event__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
bolide_interfaces__srv__LaunchEkf_Event__Sequence__destroy(bolide_interfaces__srv__LaunchEkf_Event__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    bolide_interfaces__srv__LaunchEkf_Event__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
bolide_interfaces__srv__LaunchEkf_Event__Sequence__are_equal(const bolide_interfaces__srv__LaunchEkf_Event__Sequence * lhs, const bolide_interfaces__srv__LaunchEkf_Event__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!bolide_interfaces__srv__LaunchEkf_Event__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
bolide_interfaces__srv__LaunchEkf_Event__Sequence__copy(
  const bolide_interfaces__srv__LaunchEkf_Event__Sequence * input,
  bolide_interfaces__srv__LaunchEkf_Event__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(bolide_interfaces__srv__LaunchEkf_Event);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    bolide_interfaces__srv__LaunchEkf_Event * data =
      (bolide_interfaces__srv__LaunchEkf_Event *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!bolide_interfaces__srv__LaunchEkf_Event__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          bolide_interfaces__srv__LaunchEkf_Event__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!bolide_interfaces__srv__LaunchEkf_Event__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
