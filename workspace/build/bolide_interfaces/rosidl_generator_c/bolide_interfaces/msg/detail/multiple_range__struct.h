// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from bolide_interfaces:msg/MultipleRange.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "bolide_interfaces/msg/multiple_range.h"


#ifndef BOLIDE_INTERFACES__MSG__DETAIL__MULTIPLE_RANGE__STRUCT_H_
#define BOLIDE_INTERFACES__MSG__DETAIL__MULTIPLE_RANGE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

// Include directives for member types
// Member 'ir_rear_left'
// Member 'ir_rear_right'
// Member 'sonar_rear'
#include "sensor_msgs/msg/detail/range__struct.h"

/// Struct defined in msg/MultipleRange in the package bolide_interfaces.
typedef struct bolide_interfaces__msg__MultipleRange
{
  sensor_msgs__msg__Range ir_rear_left;
  sensor_msgs__msg__Range ir_rear_right;
  sensor_msgs__msg__Range sonar_rear;
} bolide_interfaces__msg__MultipleRange;

// Struct for a sequence of bolide_interfaces__msg__MultipleRange.
typedef struct bolide_interfaces__msg__MultipleRange__Sequence
{
  bolide_interfaces__msg__MultipleRange * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} bolide_interfaces__msg__MultipleRange__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // BOLIDE_INTERFACES__MSG__DETAIL__MULTIPLE_RANGE__STRUCT_H_
