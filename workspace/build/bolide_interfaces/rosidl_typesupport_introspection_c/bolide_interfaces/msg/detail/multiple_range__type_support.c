// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from bolide_interfaces:msg/MultipleRange.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "bolide_interfaces/msg/detail/multiple_range__rosidl_typesupport_introspection_c.h"
#include "bolide_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "bolide_interfaces/msg/detail/multiple_range__functions.h"
#include "bolide_interfaces/msg/detail/multiple_range__struct.h"


// Include directives for member types
// Member `ir_rear_left`
// Member `ir_rear_right`
// Member `sonar_rear`
#include "sensor_msgs/msg/range.h"
// Member `ir_rear_left`
// Member `ir_rear_right`
// Member `sonar_rear`
#include "sensor_msgs/msg/detail/range__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void bolide_interfaces__msg__MultipleRange__rosidl_typesupport_introspection_c__MultipleRange_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  bolide_interfaces__msg__MultipleRange__init(message_memory);
}

void bolide_interfaces__msg__MultipleRange__rosidl_typesupport_introspection_c__MultipleRange_fini_function(void * message_memory)
{
  bolide_interfaces__msg__MultipleRange__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember bolide_interfaces__msg__MultipleRange__rosidl_typesupport_introspection_c__MultipleRange_message_member_array[3] = {
  {
    "ir_rear_left",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(bolide_interfaces__msg__MultipleRange, ir_rear_left),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "ir_rear_right",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(bolide_interfaces__msg__MultipleRange, ir_rear_right),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "sonar_rear",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(bolide_interfaces__msg__MultipleRange, sonar_rear),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers bolide_interfaces__msg__MultipleRange__rosidl_typesupport_introspection_c__MultipleRange_message_members = {
  "bolide_interfaces__msg",  // message namespace
  "MultipleRange",  // message name
  3,  // number of fields
  sizeof(bolide_interfaces__msg__MultipleRange),
  false,  // has_any_key_member_
  bolide_interfaces__msg__MultipleRange__rosidl_typesupport_introspection_c__MultipleRange_message_member_array,  // message members
  bolide_interfaces__msg__MultipleRange__rosidl_typesupport_introspection_c__MultipleRange_init_function,  // function to initialize message memory (memory has to be allocated)
  bolide_interfaces__msg__MultipleRange__rosidl_typesupport_introspection_c__MultipleRange_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t bolide_interfaces__msg__MultipleRange__rosidl_typesupport_introspection_c__MultipleRange_message_type_support_handle = {
  0,
  &bolide_interfaces__msg__MultipleRange__rosidl_typesupport_introspection_c__MultipleRange_message_members,
  get_message_typesupport_handle_function,
  &bolide_interfaces__msg__MultipleRange__get_type_hash,
  &bolide_interfaces__msg__MultipleRange__get_type_description,
  &bolide_interfaces__msg__MultipleRange__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_bolide_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, bolide_interfaces, msg, MultipleRange)() {
  bolide_interfaces__msg__MultipleRange__rosidl_typesupport_introspection_c__MultipleRange_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, sensor_msgs, msg, Range)();
  bolide_interfaces__msg__MultipleRange__rosidl_typesupport_introspection_c__MultipleRange_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, sensor_msgs, msg, Range)();
  bolide_interfaces__msg__MultipleRange__rosidl_typesupport_introspection_c__MultipleRange_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, sensor_msgs, msg, Range)();
  if (!bolide_interfaces__msg__MultipleRange__rosidl_typesupport_introspection_c__MultipleRange_message_type_support_handle.typesupport_identifier) {
    bolide_interfaces__msg__MultipleRange__rosidl_typesupport_introspection_c__MultipleRange_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &bolide_interfaces__msg__MultipleRange__rosidl_typesupport_introspection_c__MultipleRange_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
