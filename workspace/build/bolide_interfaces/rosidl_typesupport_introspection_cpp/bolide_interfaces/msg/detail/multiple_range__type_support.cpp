// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from bolide_interfaces:msg/MultipleRange.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "bolide_interfaces/msg/detail/multiple_range__functions.h"
#include "bolide_interfaces/msg/detail/multiple_range__struct.hpp"
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

void MultipleRange_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) bolide_interfaces::msg::MultipleRange(_init);
}

void MultipleRange_fini_function(void * message_memory)
{
  auto typed_message = static_cast<bolide_interfaces::msg::MultipleRange *>(message_memory);
  typed_message->~MultipleRange();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember MultipleRange_message_member_array[3] = {
  {
    "ir_rear_left",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<sensor_msgs::msg::Range>(),  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(bolide_interfaces::msg::MultipleRange, ir_rear_left),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "ir_rear_right",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<sensor_msgs::msg::Range>(),  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(bolide_interfaces::msg::MultipleRange, ir_rear_right),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "sonar_rear",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<sensor_msgs::msg::Range>(),  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(bolide_interfaces::msg::MultipleRange, sonar_rear),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers MultipleRange_message_members = {
  "bolide_interfaces::msg",  // message namespace
  "MultipleRange",  // message name
  3,  // number of fields
  sizeof(bolide_interfaces::msg::MultipleRange),
  false,  // has_any_key_member_
  MultipleRange_message_member_array,  // message members
  MultipleRange_init_function,  // function to initialize message memory (memory has to be allocated)
  MultipleRange_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t MultipleRange_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &MultipleRange_message_members,
  get_message_typesupport_handle_function,
  &bolide_interfaces__msg__MultipleRange__get_type_hash,
  &bolide_interfaces__msg__MultipleRange__get_type_description,
  &bolide_interfaces__msg__MultipleRange__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace bolide_interfaces


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<bolide_interfaces::msg::MultipleRange>()
{
  return &::bolide_interfaces::msg::rosidl_typesupport_introspection_cpp::MultipleRange_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, bolide_interfaces, msg, MultipleRange)() {
  return &::bolide_interfaces::msg::rosidl_typesupport_introspection_cpp::MultipleRange_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
