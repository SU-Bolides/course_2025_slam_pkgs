// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from bolide_interfaces:srv/LaunchEkf.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "bolide_interfaces/srv/launch_ekf.h"


#ifndef BOLIDE_INTERFACES__SRV__DETAIL__LAUNCH_EKF__STRUCT_H_
#define BOLIDE_INTERFACES__SRV__DETAIL__LAUNCH_EKF__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/LaunchEkf in the package bolide_interfaces.
typedef struct bolide_interfaces__srv__LaunchEkf_Request
{
  bool start;
} bolide_interfaces__srv__LaunchEkf_Request;

// Struct for a sequence of bolide_interfaces__srv__LaunchEkf_Request.
typedef struct bolide_interfaces__srv__LaunchEkf_Request__Sequence
{
  bolide_interfaces__srv__LaunchEkf_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} bolide_interfaces__srv__LaunchEkf_Request__Sequence;

// Constants defined in the message

/// Struct defined in srv/LaunchEkf in the package bolide_interfaces.
typedef struct bolide_interfaces__srv__LaunchEkf_Response
{
  bool gotcha;
} bolide_interfaces__srv__LaunchEkf_Response;

// Struct for a sequence of bolide_interfaces__srv__LaunchEkf_Response.
typedef struct bolide_interfaces__srv__LaunchEkf_Response__Sequence
{
  bolide_interfaces__srv__LaunchEkf_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} bolide_interfaces__srv__LaunchEkf_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  bolide_interfaces__srv__LaunchEkf_Event__request__MAX_SIZE = 1
};
// response
enum
{
  bolide_interfaces__srv__LaunchEkf_Event__response__MAX_SIZE = 1
};

/// Struct defined in srv/LaunchEkf in the package bolide_interfaces.
typedef struct bolide_interfaces__srv__LaunchEkf_Event
{
  service_msgs__msg__ServiceEventInfo info;
  bolide_interfaces__srv__LaunchEkf_Request__Sequence request;
  bolide_interfaces__srv__LaunchEkf_Response__Sequence response;
} bolide_interfaces__srv__LaunchEkf_Event;

// Struct for a sequence of bolide_interfaces__srv__LaunchEkf_Event.
typedef struct bolide_interfaces__srv__LaunchEkf_Event__Sequence
{
  bolide_interfaces__srv__LaunchEkf_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} bolide_interfaces__srv__LaunchEkf_Event__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // BOLIDE_INTERFACES__SRV__DETAIL__LAUNCH_EKF__STRUCT_H_
