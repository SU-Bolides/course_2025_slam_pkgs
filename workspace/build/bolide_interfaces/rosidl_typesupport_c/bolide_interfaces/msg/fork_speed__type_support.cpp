// generated from rosidl_typesupport_c/resource/idl__type_support.cpp.em
// with input from bolide_interfaces:msg/ForkSpeed.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "bolide_interfaces/msg/detail/fork_speed__struct.h"
#include "bolide_interfaces/msg/detail/fork_speed__type_support.h"
#include "bolide_interfaces/msg/detail/fork_speed__functions.h"
#include "rosidl_typesupport_c/identifier.h"
#include "rosidl_typesupport_c/message_type_support_dispatch.h"
#include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_c/visibility_control.h"
#include "rosidl_typesupport_interface/macros.h"

namespace bolide_interfaces
{

namespace msg
{

namespace rosidl_typesupport_c
{

typedef struct _ForkSpeed_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _ForkSpeed_type_support_ids_t;

static const _ForkSpeed_type_support_ids_t _ForkSpeed_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _ForkSpeed_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _ForkSpeed_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _ForkSpeed_type_support_symbol_names_t _ForkSpeed_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, bolide_interfaces, msg, ForkSpeed)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, bolide_interfaces, msg, ForkSpeed)),
  }
};

typedef struct _ForkSpeed_type_support_data_t
{
  void * data[2];
} _ForkSpeed_type_support_data_t;

static _ForkSpeed_type_support_data_t _ForkSpeed_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _ForkSpeed_message_typesupport_map = {
  2,
  "bolide_interfaces",
  &_ForkSpeed_message_typesupport_ids.typesupport_identifier[0],
  &_ForkSpeed_message_typesupport_symbol_names.symbol_name[0],
  &_ForkSpeed_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t ForkSpeed_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_ForkSpeed_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
  &bolide_interfaces__msg__ForkSpeed__get_type_hash,
  &bolide_interfaces__msg__ForkSpeed__get_type_description,
  &bolide_interfaces__msg__ForkSpeed__get_type_description_sources,
};

}  // namespace rosidl_typesupport_c

}  // namespace msg

}  // namespace bolide_interfaces

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, bolide_interfaces, msg, ForkSpeed)() {
  return &::bolide_interfaces::msg::rosidl_typesupport_c::ForkSpeed_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
