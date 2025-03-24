// generated from rosidl_typesupport_c/resource/idl__type_support.cpp.em
// with input from bolide_interfaces:srv/LaunchEkf.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "bolide_interfaces/srv/detail/launch_ekf__struct.h"
#include "bolide_interfaces/srv/detail/launch_ekf__type_support.h"
#include "bolide_interfaces/srv/detail/launch_ekf__functions.h"
#include "rosidl_typesupport_c/identifier.h"
#include "rosidl_typesupport_c/message_type_support_dispatch.h"
#include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_c/visibility_control.h"
#include "rosidl_typesupport_interface/macros.h"

namespace bolide_interfaces
{

namespace srv
{

namespace rosidl_typesupport_c
{

typedef struct _LaunchEkf_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _LaunchEkf_Request_type_support_ids_t;

static const _LaunchEkf_Request_type_support_ids_t _LaunchEkf_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _LaunchEkf_Request_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _LaunchEkf_Request_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _LaunchEkf_Request_type_support_symbol_names_t _LaunchEkf_Request_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, bolide_interfaces, srv, LaunchEkf_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, bolide_interfaces, srv, LaunchEkf_Request)),
  }
};

typedef struct _LaunchEkf_Request_type_support_data_t
{
  void * data[2];
} _LaunchEkf_Request_type_support_data_t;

static _LaunchEkf_Request_type_support_data_t _LaunchEkf_Request_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _LaunchEkf_Request_message_typesupport_map = {
  2,
  "bolide_interfaces",
  &_LaunchEkf_Request_message_typesupport_ids.typesupport_identifier[0],
  &_LaunchEkf_Request_message_typesupport_symbol_names.symbol_name[0],
  &_LaunchEkf_Request_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t LaunchEkf_Request_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_LaunchEkf_Request_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
  &bolide_interfaces__srv__LaunchEkf_Request__get_type_hash,
  &bolide_interfaces__srv__LaunchEkf_Request__get_type_description,
  &bolide_interfaces__srv__LaunchEkf_Request__get_type_description_sources,
};

}  // namespace rosidl_typesupport_c

}  // namespace srv

}  // namespace bolide_interfaces

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, bolide_interfaces, srv, LaunchEkf_Request)() {
  return &::bolide_interfaces::srv::rosidl_typesupport_c::LaunchEkf_Request_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "bolide_interfaces/srv/detail/launch_ekf__struct.h"
// already included above
// #include "bolide_interfaces/srv/detail/launch_ekf__type_support.h"
// already included above
// #include "bolide_interfaces/srv/detail/launch_ekf__functions.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/message_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_c/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace bolide_interfaces
{

namespace srv
{

namespace rosidl_typesupport_c
{

typedef struct _LaunchEkf_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _LaunchEkf_Response_type_support_ids_t;

static const _LaunchEkf_Response_type_support_ids_t _LaunchEkf_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _LaunchEkf_Response_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _LaunchEkf_Response_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _LaunchEkf_Response_type_support_symbol_names_t _LaunchEkf_Response_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, bolide_interfaces, srv, LaunchEkf_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, bolide_interfaces, srv, LaunchEkf_Response)),
  }
};

typedef struct _LaunchEkf_Response_type_support_data_t
{
  void * data[2];
} _LaunchEkf_Response_type_support_data_t;

static _LaunchEkf_Response_type_support_data_t _LaunchEkf_Response_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _LaunchEkf_Response_message_typesupport_map = {
  2,
  "bolide_interfaces",
  &_LaunchEkf_Response_message_typesupport_ids.typesupport_identifier[0],
  &_LaunchEkf_Response_message_typesupport_symbol_names.symbol_name[0],
  &_LaunchEkf_Response_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t LaunchEkf_Response_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_LaunchEkf_Response_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
  &bolide_interfaces__srv__LaunchEkf_Response__get_type_hash,
  &bolide_interfaces__srv__LaunchEkf_Response__get_type_description,
  &bolide_interfaces__srv__LaunchEkf_Response__get_type_description_sources,
};

}  // namespace rosidl_typesupport_c

}  // namespace srv

}  // namespace bolide_interfaces

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, bolide_interfaces, srv, LaunchEkf_Response)() {
  return &::bolide_interfaces::srv::rosidl_typesupport_c::LaunchEkf_Response_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "bolide_interfaces/srv/detail/launch_ekf__struct.h"
// already included above
// #include "bolide_interfaces/srv/detail/launch_ekf__type_support.h"
// already included above
// #include "bolide_interfaces/srv/detail/launch_ekf__functions.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/message_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_c/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace bolide_interfaces
{

namespace srv
{

namespace rosidl_typesupport_c
{

typedef struct _LaunchEkf_Event_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _LaunchEkf_Event_type_support_ids_t;

static const _LaunchEkf_Event_type_support_ids_t _LaunchEkf_Event_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _LaunchEkf_Event_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _LaunchEkf_Event_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _LaunchEkf_Event_type_support_symbol_names_t _LaunchEkf_Event_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, bolide_interfaces, srv, LaunchEkf_Event)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, bolide_interfaces, srv, LaunchEkf_Event)),
  }
};

typedef struct _LaunchEkf_Event_type_support_data_t
{
  void * data[2];
} _LaunchEkf_Event_type_support_data_t;

static _LaunchEkf_Event_type_support_data_t _LaunchEkf_Event_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _LaunchEkf_Event_message_typesupport_map = {
  2,
  "bolide_interfaces",
  &_LaunchEkf_Event_message_typesupport_ids.typesupport_identifier[0],
  &_LaunchEkf_Event_message_typesupport_symbol_names.symbol_name[0],
  &_LaunchEkf_Event_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t LaunchEkf_Event_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_LaunchEkf_Event_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
  &bolide_interfaces__srv__LaunchEkf_Event__get_type_hash,
  &bolide_interfaces__srv__LaunchEkf_Event__get_type_description,
  &bolide_interfaces__srv__LaunchEkf_Event__get_type_description_sources,
};

}  // namespace rosidl_typesupport_c

}  // namespace srv

}  // namespace bolide_interfaces

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, bolide_interfaces, srv, LaunchEkf_Event)() {
  return &::bolide_interfaces::srv::rosidl_typesupport_c::LaunchEkf_Event_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "bolide_interfaces/srv/detail/launch_ekf__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
#include "rosidl_typesupport_c/service_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
#include "service_msgs/msg/service_event_info.h"
#include "builtin_interfaces/msg/time.h"

namespace bolide_interfaces
{

namespace srv
{

namespace rosidl_typesupport_c
{
typedef struct _LaunchEkf_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _LaunchEkf_type_support_ids_t;

static const _LaunchEkf_type_support_ids_t _LaunchEkf_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _LaunchEkf_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _LaunchEkf_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _LaunchEkf_type_support_symbol_names_t _LaunchEkf_service_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, bolide_interfaces, srv, LaunchEkf)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, bolide_interfaces, srv, LaunchEkf)),
  }
};

typedef struct _LaunchEkf_type_support_data_t
{
  void * data[2];
} _LaunchEkf_type_support_data_t;

static _LaunchEkf_type_support_data_t _LaunchEkf_service_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _LaunchEkf_service_typesupport_map = {
  2,
  "bolide_interfaces",
  &_LaunchEkf_service_typesupport_ids.typesupport_identifier[0],
  &_LaunchEkf_service_typesupport_symbol_names.symbol_name[0],
  &_LaunchEkf_service_typesupport_data.data[0],
};

static const rosidl_service_type_support_t LaunchEkf_service_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_LaunchEkf_service_typesupport_map),
  rosidl_typesupport_c__get_service_typesupport_handle_function,
  &LaunchEkf_Request_message_type_support_handle,
  &LaunchEkf_Response_message_type_support_handle,
  &LaunchEkf_Event_message_type_support_handle,
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_CREATE_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    bolide_interfaces,
    srv,
    LaunchEkf
  ),
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_DESTROY_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    bolide_interfaces,
    srv,
    LaunchEkf
  ),
  &bolide_interfaces__srv__LaunchEkf__get_type_hash,
  &bolide_interfaces__srv__LaunchEkf__get_type_description,
  &bolide_interfaces__srv__LaunchEkf__get_type_description_sources,
};

}  // namespace rosidl_typesupport_c

}  // namespace srv

}  // namespace bolide_interfaces

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_c, bolide_interfaces, srv, LaunchEkf)() {
  return &::bolide_interfaces::srv::rosidl_typesupport_c::LaunchEkf_service_type_support_handle;
}

#ifdef __cplusplus
}
#endif
