// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from bolide_interfaces:msg/SpeedDirection.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "bolide_interfaces/msg/detail/speed_direction__functions.h"
#include "bolide_interfaces/msg/detail/speed_direction__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace bolide_interfaces
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void SpeedDirection_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) bolide_interfaces::msg::SpeedDirection(_init);
}

void SpeedDirection_fini_function(void * message_memory)
{
  auto typed_message = static_cast<bolide_interfaces::msg::SpeedDirection *>(message_memory);
  typed_message->~SpeedDirection();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember SpeedDirection_message_member_array[2] = {
  {
    "speed",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(bolide_interfaces::msg::SpeedDirection, speed),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "direction",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(bolide_interfaces::msg::SpeedDirection, direction),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers SpeedDirection_message_members = {
  "bolide_interfaces::msg",  // message namespace
  "SpeedDirection",  // message name
  2,  // number of fields
  sizeof(bolide_interfaces::msg::SpeedDirection),
  false,  // has_any_key_member_
  SpeedDirection_message_member_array,  // message members
  SpeedDirection_init_function,  // function to initialize message memory (memory has to be allocated)
  SpeedDirection_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t SpeedDirection_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &SpeedDirection_message_members,
  get_message_typesupport_handle_function,
  &bolide_interfaces__msg__SpeedDirection__get_type_hash,
  &bolide_interfaces__msg__SpeedDirection__get_type_description,
  &bolide_interfaces__msg__SpeedDirection__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace bolide_interfaces


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<bolide_interfaces::msg::SpeedDirection>()
{
  return &::bolide_interfaces::msg::rosidl_typesupport_introspection_cpp::SpeedDirection_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, bolide_interfaces, msg, SpeedDirection)() {
  return &::bolide_interfaces::msg::rosidl_typesupport_introspection_cpp::SpeedDirection_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
