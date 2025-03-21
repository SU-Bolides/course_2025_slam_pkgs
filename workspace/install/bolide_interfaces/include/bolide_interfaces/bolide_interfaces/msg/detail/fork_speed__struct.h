// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from bolide_interfaces:msg/ForkSpeed.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "bolide_interfaces/msg/fork_speed.h"


#ifndef BOLIDE_INTERFACES__MSG__DETAIL__FORK_SPEED__STRUCT_H_
#define BOLIDE_INTERFACES__MSG__DETAIL__FORK_SPEED__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"

/// Struct defined in msg/ForkSpeed in the package bolide_interfaces.
typedef struct bolide_interfaces__msg__ForkSpeed
{
  std_msgs__msg__Header header;
  float speed;
} bolide_interfaces__msg__ForkSpeed;

// Struct for a sequence of bolide_interfaces__msg__ForkSpeed.
typedef struct bolide_interfaces__msg__ForkSpeed__Sequence
{
  bolide_interfaces__msg__ForkSpeed * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} bolide_interfaces__msg__ForkSpeed__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // BOLIDE_INTERFACES__MSG__DETAIL__FORK_SPEED__STRUCT_H_
