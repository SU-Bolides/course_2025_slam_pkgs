// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from bolide_interfaces:msg/CameraInfo.idl
// generated code does not contain a copyright notice
#include "bolide_interfaces/msg/detail/camera_info__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <cstddef>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/serialization_helpers.hpp"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "bolide_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "bolide_interfaces/msg/detail/camera_info__struct.h"
#include "bolide_interfaces/msg/detail/camera_info__functions.h"
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

#include "rosidl_runtime_c/string.h"  // front_color, left_color, right_color
#include "rosidl_runtime_c/string_functions.h"  // front_color, left_color, right_color

// forward declare type support functions


using _CameraInfo__ros_msg_type = bolide_interfaces__msg__CameraInfo;


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_bolide_interfaces
bool cdr_serialize_bolide_interfaces__msg__CameraInfo(
  const bolide_interfaces__msg__CameraInfo * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: wrong_way
  {
    cdr << (ros_message->wrong_way ? true : false);
  }

  // Field name: front_color
  {
    const rosidl_runtime_c__String * str = &ros_message->front_color;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: left_color
  {
    const rosidl_runtime_c__String * str = &ros_message->left_color;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: right_color
  {
    const rosidl_runtime_c__String * str = &ros_message->right_color;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_bolide_interfaces
bool cdr_deserialize_bolide_interfaces__msg__CameraInfo(
  eprosima::fastcdr::Cdr & cdr,
  bolide_interfaces__msg__CameraInfo * ros_message)
{
  // Field name: wrong_way
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->wrong_way = tmp ? true : false;
  }

  // Field name: front_color
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->front_color.data) {
      rosidl_runtime_c__String__init(&ros_message->front_color);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->front_color,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'front_color'\n");
      return false;
    }
  }

  // Field name: left_color
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->left_color.data) {
      rosidl_runtime_c__String__init(&ros_message->left_color);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->left_color,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'left_color'\n");
      return false;
    }
  }

  // Field name: right_color
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->right_color.data) {
      rosidl_runtime_c__String__init(&ros_message->right_color);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->right_color,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'right_color'\n");
      return false;
    }
  }

  return true;
}  // NOLINT(readability/fn_size)


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_bolide_interfaces
size_t get_serialized_size_bolide_interfaces__msg__CameraInfo(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _CameraInfo__ros_msg_type * ros_message = static_cast<const _CameraInfo__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: wrong_way
  {
    size_t item_size = sizeof(ros_message->wrong_way);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: front_color
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->front_color.size + 1);

  // Field name: left_color
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->left_color.size + 1);

  // Field name: right_color
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->right_color.size + 1);

  return current_alignment - initial_alignment;
}


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_bolide_interfaces
size_t max_serialized_size_bolide_interfaces__msg__CameraInfo(
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

  // Field name: wrong_way
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: front_color
  {
    size_t array_size = 1;
    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  // Field name: left_color
  {
    size_t array_size = 1;
    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  // Field name: right_color
  {
    size_t array_size = 1;
    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }


  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = bolide_interfaces__msg__CameraInfo;
    is_plain =
      (
      offsetof(DataType, right_color) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_bolide_interfaces
bool cdr_serialize_key_bolide_interfaces__msg__CameraInfo(
  const bolide_interfaces__msg__CameraInfo * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: wrong_way
  {
    cdr << (ros_message->wrong_way ? true : false);
  }

  // Field name: front_color
  {
    const rosidl_runtime_c__String * str = &ros_message->front_color;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: left_color
  {
    const rosidl_runtime_c__String * str = &ros_message->left_color;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: right_color
  {
    const rosidl_runtime_c__String * str = &ros_message->right_color;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_bolide_interfaces
size_t get_serialized_size_key_bolide_interfaces__msg__CameraInfo(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _CameraInfo__ros_msg_type * ros_message = static_cast<const _CameraInfo__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;

  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: wrong_way
  {
    size_t item_size = sizeof(ros_message->wrong_way);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: front_color
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->front_color.size + 1);

  // Field name: left_color
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->left_color.size + 1);

  // Field name: right_color
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->right_color.size + 1);

  return current_alignment - initial_alignment;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_bolide_interfaces
size_t max_serialized_size_key_bolide_interfaces__msg__CameraInfo(
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
  // Field name: wrong_way
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: front_color
  {
    size_t array_size = 1;
    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  // Field name: left_color
  {
    size_t array_size = 1;
    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  // Field name: right_color
  {
    size_t array_size = 1;
    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = bolide_interfaces__msg__CameraInfo;
    is_plain =
      (
      offsetof(DataType, right_color) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}


static bool _CameraInfo__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const bolide_interfaces__msg__CameraInfo * ros_message = static_cast<const bolide_interfaces__msg__CameraInfo *>(untyped_ros_message);
  (void)ros_message;
  return cdr_serialize_bolide_interfaces__msg__CameraInfo(ros_message, cdr);
}

static bool _CameraInfo__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  bolide_interfaces__msg__CameraInfo * ros_message = static_cast<bolide_interfaces__msg__CameraInfo *>(untyped_ros_message);
  (void)ros_message;
  return cdr_deserialize_bolide_interfaces__msg__CameraInfo(cdr, ros_message);
}

static uint32_t _CameraInfo__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_bolide_interfaces__msg__CameraInfo(
      untyped_ros_message, 0));
}

static size_t _CameraInfo__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_bolide_interfaces__msg__CameraInfo(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_CameraInfo = {
  "bolide_interfaces::msg",
  "CameraInfo",
  _CameraInfo__cdr_serialize,
  _CameraInfo__cdr_deserialize,
  _CameraInfo__get_serialized_size,
  _CameraInfo__max_serialized_size,
  nullptr
};

static rosidl_message_type_support_t _CameraInfo__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_CameraInfo,
  get_message_typesupport_handle_function,
  &bolide_interfaces__msg__CameraInfo__get_type_hash,
  &bolide_interfaces__msg__CameraInfo__get_type_description,
  &bolide_interfaces__msg__CameraInfo__get_type_description_sources,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, bolide_interfaces, msg, CameraInfo)() {
  return &_CameraInfo__type_support;
}

#if defined(__cplusplus)
}
#endif
