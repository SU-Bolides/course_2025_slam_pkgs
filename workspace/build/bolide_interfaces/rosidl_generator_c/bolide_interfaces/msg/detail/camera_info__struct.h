// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from bolide_interfaces:msg/CameraInfo.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "bolide_interfaces/msg/camera_info.h"


#ifndef BOLIDE_INTERFACES__MSG__DETAIL__CAMERA_INFO__STRUCT_H_
#define BOLIDE_INTERFACES__MSG__DETAIL__CAMERA_INFO__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

// Include directives for member types
// Member 'front_color'
// Member 'left_color'
// Member 'right_color'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/CameraInfo in the package bolide_interfaces.
typedef struct bolide_interfaces__msg__CameraInfo
{
  bool wrong_way;
  rosidl_runtime_c__String front_color;
  rosidl_runtime_c__String left_color;
  rosidl_runtime_c__String right_color;
} bolide_interfaces__msg__CameraInfo;

// Struct for a sequence of bolide_interfaces__msg__CameraInfo.
typedef struct bolide_interfaces__msg__CameraInfo__Sequence
{
  bolide_interfaces__msg__CameraInfo * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} bolide_interfaces__msg__CameraInfo__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // BOLIDE_INTERFACES__MSG__DETAIL__CAMERA_INFO__STRUCT_H_
