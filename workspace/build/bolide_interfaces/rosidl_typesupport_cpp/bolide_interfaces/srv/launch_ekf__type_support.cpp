// generated from rosidl_typesupport_cpp/resource/idl__type_support.cpp.em
// with input from bolide_interfaces:srv/LaunchEkf.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "bolide_interfaces/srv/detail/launch_ekf__functions.h"
#include "bolide_interfaces/srv/detail/launch_ekf__struct.hpp"
#include "rosidl_typesupport_cpp/identifier.hpp"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
#include "rosidl_typesupport_cpp/visibility_control.h"
#include "rosidl_typesupport_interface/macros.h"

namespace bolide_interfaces
{

namespace srv
{

namespace rosidl_typesupport_cpp
{

typedef struct _LaunchEkf_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _LaunchEkf_Request_type_support_ids_t;

static const _LaunchEkf_Request_type_support_ids_t _LaunchEkf_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
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
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, bolide_interfaces, srv, LaunchEkf_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, bolide_interfaces, srv, LaunchEkf_Request)),
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
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_LaunchEkf_Request_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &bolide_interfaces__srv__LaunchEkf_Request__get_type_hash,
  &bolide_interfaces__srv__LaunchEkf_Request__get_type_description,
  &bolide_interfaces__srv__LaunchEkf_Request__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace bolide_interfaces

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<bolide_interfaces::srv::LaunchEkf_Request>()
{
  return &::bolide_interfaces::srv::rosidl_typesupport_cpp::LaunchEkf_Request_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, bolide_interfaces, srv, LaunchEkf_Request)() {
  return get_message_type_support_handle<bolide_interfaces::srv::LaunchEkf_Request>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "bolide_interfaces/srv/detail/launch_ekf__functions.h"
// already included above
// #include "bolide_interfaces/srv/detail/launch_ekf__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace bolide_interfaces
{

namespace srv
{

namespace rosidl_typesupport_cpp
{

typedef struct _LaunchEkf_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _LaunchEkf_Response_type_support_ids_t;

static const _LaunchEkf_Response_type_support_ids_t _LaunchEkf_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
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
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, bolide_interfaces, srv, LaunchEkf_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, bolide_interfaces, srv, LaunchEkf_Response)),
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
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_LaunchEkf_Response_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &bolide_interfaces__srv__LaunchEkf_Response__get_type_hash,
  &bolide_interfaces__srv__LaunchEkf_Response__get_type_description,
  &bolide_interfaces__srv__LaunchEkf_Response__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace bolide_interfaces

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<bolide_interfaces::srv::LaunchEkf_Response>()
{
  return &::bolide_interfaces::srv::rosidl_typesupport_cpp::LaunchEkf_Response_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, bolide_interfaces, srv, LaunchEkf_Response)() {
  return get_message_type_support_handle<bolide_interfaces::srv::LaunchEkf_Response>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "bolide_interfaces/srv/detail/launch_ekf__functions.h"
// already included above
// #include "bolide_interfaces/srv/detail/launch_ekf__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace bolide_interfaces
{

namespace srv
{

namespace rosidl_typesupport_cpp
{

typedef struct _LaunchEkf_Event_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _LaunchEkf_Event_type_support_ids_t;

static const _LaunchEkf_Event_type_support_ids_t _LaunchEkf_Event_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
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
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, bolide_interfaces, srv, LaunchEkf_Event)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, bolide_interfaces, srv, LaunchEkf_Event)),
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
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_LaunchEkf_Event_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &bolide_interfaces__srv__LaunchEkf_Event__get_type_hash,
  &bolide_interfaces__srv__LaunchEkf_Event__get_type_description,
  &bolide_interfaces__srv__LaunchEkf_Event__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace bolide_interfaces

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<bolide_interfaces::srv::LaunchEkf_Event>()
{
  return &::bolide_interfaces::srv::rosidl_typesupport_cpp::LaunchEkf_Event_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, bolide_interfaces, srv, LaunchEkf_Event)() {
  return get_message_type_support_handle<bolide_interfaces::srv::LaunchEkf_Event>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
#include "rosidl_runtime_c/service_type_support_struct.h"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "bolide_interfaces/srv/detail/launch_ekf__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_cpp/service_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace bolide_interfaces
{

namespace srv
{

namespace rosidl_typesupport_cpp
{

typedef struct _LaunchEkf_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _LaunchEkf_type_support_ids_t;

static const _LaunchEkf_type_support_ids_t _LaunchEkf_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
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
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, bolide_interfaces, srv, LaunchEkf)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, bolide_interfaces, srv, LaunchEkf)),
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
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_LaunchEkf_service_typesupport_map),
  ::rosidl_typesupport_cpp::get_service_typesupport_handle_function,
  ::rosidl_typesupport_cpp::get_message_type_support_handle<bolide_interfaces::srv::LaunchEkf_Request>(),
  ::rosidl_typesupport_cpp::get_message_type_support_handle<bolide_interfaces::srv::LaunchEkf_Response>(),
  ::rosidl_typesupport_cpp::get_message_type_support_handle<bolide_interfaces::srv::LaunchEkf_Event>(),
  &::rosidl_typesupport_cpp::service_create_event_message<bolide_interfaces::srv::LaunchEkf>,
  &::rosidl_typesupport_cpp::service_destroy_event_message<bolide_interfaces::srv::LaunchEkf>,
  &bolide_interfaces__srv__LaunchEkf__get_type_hash,
  &bolide_interfaces__srv__LaunchEkf__get_type_description,
  &bolide_interfaces__srv__LaunchEkf__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace bolide_interfaces

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
get_service_type_support_handle<bolide_interfaces::srv::LaunchEkf>()
{
  return &::bolide_interfaces::srv::rosidl_typesupport_cpp::LaunchEkf_service_type_support_handle;
}

}  // namespace rosidl_typesupport_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_cpp, bolide_interfaces, srv, LaunchEkf)() {
  return ::rosidl_typesupport_cpp::get_service_type_support_handle<bolide_interfaces::srv::LaunchEkf>();
}

#ifdef __cplusplus
}
#endif
