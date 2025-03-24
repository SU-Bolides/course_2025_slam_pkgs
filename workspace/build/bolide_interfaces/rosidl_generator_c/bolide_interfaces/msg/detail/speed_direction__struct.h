// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from bolide_interfaces:msg/SpeedDirection.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "bolide_interfaces/msg/speed_direction.h"


#ifndef BOLIDE_INTERFACES__MSG__DETAIL__SPEED_DIRECTION__STRUCT_H_
#define BOLIDE_INTERFACES__MSG__DETAIL__SPEED_DIRECTION__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

/// Struct defined in msg/SpeedDirection in the package bolide_interfaces.
typedef struct bolide_interfaces__msg__SpeedDirection
{
  double speed;
  double direction;
} bolide_interfaces__msg__SpeedDirection;

// Struct for a sequence of bolide_interfaces__msg__SpeedDirection.
typedef struct bolide_interfaces__msg__SpeedDirection__Sequence
{
  bolide_interfaces__msg__SpeedDirection * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} bolide_interfaces__msg__SpeedDirection__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // BOLIDE_INTERFACES__MSG__DETAIL__SPEED_DIRECTION__STRUCT_H_
