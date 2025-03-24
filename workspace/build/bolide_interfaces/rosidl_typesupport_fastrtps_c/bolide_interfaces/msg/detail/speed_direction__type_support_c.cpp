// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from bolide_interfaces:msg/SpeedDirection.idl
// generated code does not contain a copyright notice
#include "bolide_interfaces/msg/detail/speed_direction__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <cstddef>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/serialization_helpers.hpp"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "bolide_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "bolide_interfaces/msg/detail/speed_direction__struct.h"
#include "bolide_interfaces/msg/detail/speed_direction__functions.h"
#include "fastcdr/Cdr.h"

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

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif


// forward declare type support functions


using _SpeedDirection__ros_msg_type = bolide_interfaces__msg__SpeedDirection;


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_bolide_interfaces
bool cdr_serialize_bolide_interfaces__msg__SpeedDirection(
  const bolide_interfaces__msg__SpeedDirection * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: speed
  {
    cdr << ros_message->speed;
  }

  // Field name: direction
  {
    cdr << ros_message->direction;
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_bolide_interfaces
bool cdr_deserialize_bolide_interfaces__msg__SpeedDirection(
  eprosima::fastcdr::Cdr & cdr,
  bolide_interfaces__msg__SpeedDirection * ros_message)
{
  // Field name: speed
  {
    cdr >> ros_message->speed;
  }

  // Field name: direction
  {
    cdr >> ros_message->direction;
  }

  return true;
}  // NOLINT(readability/fn_size)


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_bolide_interfaces
size_t get_serialized_size_bolide_interfaces__msg__SpeedDirection(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _SpeedDirection__ros_msg_type * ros_message = static_cast<const _SpeedDirection__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: speed
  {
    size_t item_size = sizeof(ros_message->speed);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: direction
  {
    size_t item_size = sizeof(ros_message->direction);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_bolide_interfaces
size_t max_serialized_size_bolide_interfaces__msg__SpeedDirection(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // Field name: speed
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Field name: direction
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }


  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = bolide_interfaces__msg__SpeedDirection;
    is_plain =
      (
      offsetof(DataType, direction) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_bolide_interfaces
bool cdr_serialize_key_bolide_interfaces__msg__SpeedDirection(
  const bolide_interfaces__msg__SpeedDirection * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: speed
  {
    cdr << ros_message->speed;
  }

  // Field name: direction
  {
    cdr << ros_message->direction;
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_bolide_interfaces
size_t get_serialized_size_key_bolide_interfaces__msg__SpeedDirection(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _SpeedDirection__ros_msg_type * ros_message = static_cast<const _SpeedDirection__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;

  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: speed
  {
    size_t item_size = sizeof(ros_message->speed);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: direction
  {
    size_t item_size = sizeof(ros_message->direction);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_bolide_interfaces
size_t max_serialized_size_key_bolide_interfaces__msg__SpeedDirection(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;
  // Field name: speed
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Field name: direction
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = bolide_interfaces__msg__SpeedDirection;
    is_plain =
      (
      offsetof(DataType, direction) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}


static bool _SpeedDirection__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const bolide_interfaces__msg__SpeedDirection * ros_message = static_cast<const bolide_interfaces__msg__SpeedDirection *>(untyped_ros_message);
  (void)ros_message;
  return cdr_serialize_bolide_interfaces__msg__SpeedDirection(ros_message, cdr);
}

static bool _SpeedDirection__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  bolide_interfaces__msg__SpeedDirection * ros_message = static_cast<bolide_interfaces__msg__SpeedDirection *>(untyped_ros_message);
  (void)ros_message;
  return cdr_deserialize_bolide_interfaces__msg__SpeedDirection(cdr, ros_message);
}

static uint32_t _SpeedDirection__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_bolide_interfaces__msg__SpeedDirection(
      untyped_ros_message, 0));
}

static size_t _SpeedDirection__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_bolide_interfaces__msg__SpeedDirection(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_SpeedDirection = {
  "bolide_interfaces::msg",
  "SpeedDirection",
  _SpeedDirection__cdr_serialize,
  _SpeedDirection__cdr_deserialize,
  _SpeedDirection__get_serialized_size,
  _SpeedDirection__max_serialized_size,
  nullptr
};

static rosidl_message_type_support_t _SpeedDirection__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_SpeedDirection,
  get_message_typesupport_handle_function,
  &bolide_interfaces__msg__SpeedDirection__get_type_hash,
  &bolide_interfaces__msg__SpeedDirection__get_type_description,
  &bolide_interfaces__msg__SpeedDirection__get_type_description_sources,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, bolide_interfaces, msg, SpeedDirection)() {
  return &_SpeedDirection__type_support;
}

#if defined(__cplusplus)
}
#endif
