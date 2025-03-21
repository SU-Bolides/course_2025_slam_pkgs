// generated from rosidl_typesupport_fastrtps_c/resource/idl__rosidl_typesupport_fastrtps_c.h.em
// with input from bolide_interfaces:msg/CameraInfo.idl
// generated code does not contain a copyright notice
#ifndef BOLIDE_INTERFACES__MSG__DETAIL__CAMERA_INFO__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_
#define BOLIDE_INTERFACES__MSG__DETAIL__CAMERA_INFO__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_


#include <stddef.h>
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "bolide_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "bolide_interfaces/msg/detail/camera_info__struct.h"
#include "fastcdr/Cdr.h"

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_bolide_interfaces
bool cdr_serialize_bolide_interfaces__msg__CameraInfo(
  const bolide_interfaces__msg__CameraInfo * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_bolide_interfaces
bool cdr_deserialize_bolide_interfaces__msg__CameraInfo(
  eprosima::fastcdr::Cdr &,
  bolide_interfaces__msg__CameraInfo * ros_message);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_bolide_interfaces
size_t get_serialized_size_bolide_interfaces__msg__CameraInfo(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_bolide_interfaces
size_t max_serialized_size_bolide_interfaces__msg__CameraInfo(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_bolide_interfaces
bool cdr_serialize_key_bolide_interfaces__msg__CameraInfo(
  const bolide_interfaces__msg__CameraInfo * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_bolide_interfaces
size_t get_serialized_size_key_bolide_interfaces__msg__CameraInfo(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_bolide_interfaces
size_t max_serialized_size_key_bolide_interfaces__msg__CameraInfo(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_bolide_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, bolide_interfaces, msg, CameraInfo)();

#ifdef __cplusplus
}
#endif

#endif  // BOLIDE_INTERFACES__MSG__DETAIL__CAMERA_INFO__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_
