// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__rosidl_typesupport_fastrtps_cpp.hpp.em
// with input from bolide_interfaces:msg/CameraInfo.idl
// generated code does not contain a copyright notice

#ifndef BOLIDE_INTERFACES__MSG__DETAIL__CAMERA_INFO__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
#define BOLIDE_INTERFACES__MSG__DETAIL__CAMERA_INFO__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

#include <cstddef>
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "bolide_interfaces/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
#include "bolide_interfaces/msg/detail/camera_info__struct.hpp"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

#include "fastcdr/Cdr.h"

namespace bolide_interfaces
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_bolide_interfaces
cdr_serialize(
  const bolide_interfaces::msg::CameraInfo & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_bolide_interfaces
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  bolide_interfaces::msg::CameraInfo & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_bolide_interfaces
get_serialized_size(
  const bolide_interfaces::msg::CameraInfo & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_bolide_interfaces
max_serialized_size_CameraInfo(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_bolide_interfaces
cdr_serialize_key(
  const bolide_interfaces::msg::CameraInfo & ros_message,
  eprosima::fastcdr::Cdr &);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_bolide_interfaces
get_serialized_size_key(
  const bolide_interfaces::msg::CameraInfo & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_bolide_interfaces
max_serialized_size_key_CameraInfo(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace bolide_interfaces

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_bolide_interfaces
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, bolide_interfaces, msg, CameraInfo)();

#ifdef __cplusplus
}
#endif

#endif  // BOLIDE_INTERFACES__MSG__DETAIL__CAMERA_INFO__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
