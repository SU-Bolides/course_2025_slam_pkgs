// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from bolide_interfaces:msg/MultipleRange.idl
// generated code does not contain a copyright notice
#include "bolide_interfaces/msg/detail/multiple_range__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <cstddef>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/serialization_helpers.hpp"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "bolide_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "bolide_interfaces/msg/detail/multiple_range__struct.h"
#include "bolide_interfaces/msg/detail/multiple_range__functions.h"
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

#include "sensor_msgs/msg/detail/range__functions.h"  // ir_rear_left, ir_rear_right, sonar_rear

// forward declare type support functions

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_bolide_interfaces
bool cdr_serialize_sensor_msgs__msg__Range(
  const sensor_msgs__msg__Range * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_bolide_interfaces
bool cdr_deserialize_sensor_msgs__msg__Range(
  eprosima::fastcdr::Cdr & cdr,
  sensor_msgs__msg__Range * ros_message);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_bolide_interfaces
size_t get_serialized_size_sensor_msgs__msg__Range(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_bolide_interfaces
size_t max_serialized_size_sensor_msgs__msg__Range(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_bolide_interfaces
bool cdr_serialize_key_sensor_msgs__msg__Range(
  const sensor_msgs__msg__Range * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_bolide_interfaces
size_t get_serialized_size_key_sensor_msgs__msg__Range(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_bolide_interfaces
size_t max_serialized_size_key_sensor_msgs__msg__Range(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_bolide_interfaces
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, sensor_msgs, msg, Range)();


using _MultipleRange__ros_msg_type = bolide_interfaces__msg__MultipleRange;


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_bolide_interfaces
bool cdr_serialize_bolide_interfaces__msg__MultipleRange(
  const bolide_interfaces__msg__MultipleRange * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: ir_rear_left
  {
    cdr_serialize_sensor_msgs__msg__Range(
      &ros_message->ir_rear_left, cdr);
  }

  // Field name: ir_rear_right
  {
    cdr_serialize_sensor_msgs__msg__Range(
      &ros_message->ir_rear_right, cdr);
  }

  // Field name: sonar_rear
  {
    cdr_serialize_sensor_msgs__msg__Range(
      &ros_message->sonar_rear, cdr);
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_bolide_interfaces
bool cdr_deserialize_bolide_interfaces__msg__MultipleRange(
  eprosima::fastcdr::Cdr & cdr,
  bolide_interfaces__msg__MultipleRange * ros_message)
{
  // Field name: ir_rear_left
  {
    cdr_deserialize_sensor_msgs__msg__Range(cdr, &ros_message->ir_rear_left);
  }

  // Field name: ir_rear_right
  {
    cdr_deserialize_sensor_msgs__msg__Range(cdr, &ros_message->ir_rear_right);
  }

  // Field name: sonar_rear
  {
    cdr_deserialize_sensor_msgs__msg__Range(cdr, &ros_message->sonar_rear);
  }

  return true;
}  // NOLINT(readability/fn_size)


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_bolide_interfaces
size_t get_serialized_size_bolide_interfaces__msg__MultipleRange(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _MultipleRange__ros_msg_type * ros_message = static_cast<const _MultipleRange__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: ir_rear_left
  current_alignment += get_serialized_size_sensor_msgs__msg__Range(
    &(ros_message->ir_rear_left), current_alignment);

  // Field name: ir_rear_right
  current_alignment += get_serialized_size_sensor_msgs__msg__Range(
    &(ros_message->ir_rear_right), current_alignment);

  // Field name: sonar_rear
  current_alignment += get_serialized_size_sensor_msgs__msg__Range(
    &(ros_message->sonar_rear), current_alignment);

  return current_alignment - initial_alignment;
}


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_bolide_interfaces
size_t max_serialized_size_bolide_interfaces__msg__MultipleRange(
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

  // Field name: ir_rear_left
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_sensor_msgs__msg__Range(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: ir_rear_right
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_sensor_msgs__msg__Range(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: sonar_rear
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_sensor_msgs__msg__Range(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }


  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = bolide_interfaces__msg__MultipleRange;
    is_plain =
      (
      offsetof(DataType, sonar_rear) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_bolide_interfaces
bool cdr_serialize_key_bolide_interfaces__msg__MultipleRange(
  const bolide_interfaces__msg__MultipleRange * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: ir_rear_left
  {
    cdr_serialize_key_sensor_msgs__msg__Range(
      &ros_message->ir_rear_left, cdr);
  }

  // Field name: ir_rear_right
  {
    cdr_serialize_key_sensor_msgs__msg__Range(
      &ros_message->ir_rear_right, cdr);
  }

  // Field name: sonar_rear
  {
    cdr_serialize_key_sensor_msgs__msg__Range(
      &ros_message->sonar_rear, cdr);
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_bolide_interfaces
size_t get_serialized_size_key_bolide_interfaces__msg__MultipleRange(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _MultipleRange__ros_msg_type * ros_message = static_cast<const _MultipleRange__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;

  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: ir_rear_left
  current_alignment += get_serialized_size_key_sensor_msgs__msg__Range(
    &(ros_message->ir_rear_left), current_alignment);

  // Field name: ir_rear_right
  current_alignment += get_serialized_size_key_sensor_msgs__msg__Range(
    &(ros_message->ir_rear_right), current_alignment);

  // Field name: sonar_rear
  current_alignment += get_serialized_size_key_sensor_msgs__msg__Range(
    &(ros_message->sonar_rear), current_alignment);

  return current_alignment - initial_alignment;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_bolide_interfaces
size_t max_serialized_size_key_bolide_interfaces__msg__MultipleRange(
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
  // Field name: ir_rear_left
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_key_sensor_msgs__msg__Range(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: ir_rear_right
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_key_sensor_msgs__msg__Range(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: sonar_rear
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_key_sensor_msgs__msg__Range(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = bolide_interfaces__msg__MultipleRange;
    is_plain =
      (
      offsetof(DataType, sonar_rear) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}


static bool _MultipleRange__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const bolide_interfaces__msg__MultipleRange * ros_message = static_cast<const bolide_interfaces__msg__MultipleRange *>(untyped_ros_message);
  (void)ros_message;
  return cdr_serialize_bolide_interfaces__msg__MultipleRange(ros_message, cdr);
}

static bool _MultipleRange__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  bolide_interfaces__msg__MultipleRange * ros_message = static_cast<bolide_interfaces__msg__MultipleRange *>(untyped_ros_message);
  (void)ros_message;
  return cdr_deserialize_bolide_interfaces__msg__MultipleRange(cdr, ros_message);
}

static uint32_t _MultipleRange__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_bolide_interfaces__msg__MultipleRange(
      untyped_ros_message, 0));
}

static size_t _MultipleRange__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_bolide_interfaces__msg__MultipleRange(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_MultipleRange = {
  "bolide_interfaces::msg",
  "MultipleRange",
  _MultipleRange__cdr_serialize,
  _MultipleRange__cdr_deserialize,
  _MultipleRange__get_serialized_size,
  _MultipleRange__max_serialized_size,
  nullptr
};

static rosidl_message_type_support_t _MultipleRange__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_MultipleRange,
  get_message_typesupport_handle_function,
  &bolide_interfaces__msg__MultipleRange__get_type_hash,
  &bolide_interfaces__msg__MultipleRange__get_type_description,
  &bolide_interfaces__msg__MultipleRange__get_type_description_sources,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, bolide_interfaces, msg, MultipleRange)() {
  return &_MultipleRange__type_support;
}

#if defined(__cplusplus)
}
#endif
