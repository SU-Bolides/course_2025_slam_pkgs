// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from bolide_interfaces:srv/LaunchEkf.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "bolide_interfaces/srv/launch_ekf.hpp"


#ifndef BOLIDE_INTERFACES__SRV__DETAIL__LAUNCH_EKF__TRAITS_HPP_
#define BOLIDE_INTERFACES__SRV__DETAIL__LAUNCH_EKF__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "bolide_interfaces/srv/detail/launch_ekf__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace bolide_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const LaunchEkf_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: start
  {
    out << "start: ";
    rosidl_generator_traits::value_to_yaml(msg.start, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const LaunchEkf_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: start
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "start: ";
    rosidl_generator_traits::value_to_yaml(msg.start, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const LaunchEkf_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace bolide_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use bolide_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const bolide_interfaces::srv::LaunchEkf_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  bolide_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use bolide_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const bolide_interfaces::srv::LaunchEkf_Request & msg)
{
  return bolide_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<bolide_interfaces::srv::LaunchEkf_Request>()
{
  return "bolide_interfaces::srv::LaunchEkf_Request";
}

template<>
inline const char * name<bolide_interfaces::srv::LaunchEkf_Request>()
{
  return "bolide_interfaces/srv/LaunchEkf_Request";
}

template<>
struct has_fixed_size<bolide_interfaces::srv::LaunchEkf_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<bolide_interfaces::srv::LaunchEkf_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<bolide_interfaces::srv::LaunchEkf_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace bolide_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const LaunchEkf_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: gotcha
  {
    out << "gotcha: ";
    rosidl_generator_traits::value_to_yaml(msg.gotcha, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const LaunchEkf_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: gotcha
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "gotcha: ";
    rosidl_generator_traits::value_to_yaml(msg.gotcha, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const LaunchEkf_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace bolide_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use bolide_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const bolide_interfaces::srv::LaunchEkf_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  bolide_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use bolide_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const bolide_interfaces::srv::LaunchEkf_Response & msg)
{
  return bolide_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<bolide_interfaces::srv::LaunchEkf_Response>()
{
  return "bolide_interfaces::srv::LaunchEkf_Response";
}

template<>
inline const char * name<bolide_interfaces::srv::LaunchEkf_Response>()
{
  return "bolide_interfaces/srv/LaunchEkf_Response";
}

template<>
struct has_fixed_size<bolide_interfaces::srv::LaunchEkf_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<bolide_interfaces::srv::LaunchEkf_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<bolide_interfaces::srv::LaunchEkf_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__traits.hpp"

namespace bolide_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const LaunchEkf_Event & msg,
  std::ostream & out)
{
  out << "{";
  // member: info
  {
    out << "info: ";
    to_flow_style_yaml(msg.info, out);
    out << ", ";
  }

  // member: request
  {
    if (msg.request.size() == 0) {
      out << "request: []";
    } else {
      out << "request: [";
      size_t pending_items = msg.request.size();
      for (auto item : msg.request) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: response
  {
    if (msg.response.size() == 0) {
      out << "response: []";
    } else {
      out << "response: [";
      size_t pending_items = msg.response.size();
      for (auto item : msg.response) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const LaunchEkf_Event & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: info
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "info:\n";
    to_block_style_yaml(msg.info, out, indentation + 2);
  }

  // member: request
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.request.size() == 0) {
      out << "request: []\n";
    } else {
      out << "request:\n";
      for (auto item : msg.request) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: response
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.response.size() == 0) {
      out << "response: []\n";
    } else {
      out << "response:\n";
      for (auto item : msg.response) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const LaunchEkf_Event & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace bolide_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use bolide_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const bolide_interfaces::srv::LaunchEkf_Event & msg,
  std::ostream & out, size_t indentation = 0)
{
  bolide_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use bolide_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const bolide_interfaces::srv::LaunchEkf_Event & msg)
{
  return bolide_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<bolide_interfaces::srv::LaunchEkf_Event>()
{
  return "bolide_interfaces::srv::LaunchEkf_Event";
}

template<>
inline const char * name<bolide_interfaces::srv::LaunchEkf_Event>()
{
  return "bolide_interfaces/srv/LaunchEkf_Event";
}

template<>
struct has_fixed_size<bolide_interfaces::srv::LaunchEkf_Event>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<bolide_interfaces::srv::LaunchEkf_Event>
  : std::integral_constant<bool, has_bounded_size<bolide_interfaces::srv::LaunchEkf_Request>::value && has_bounded_size<bolide_interfaces::srv::LaunchEkf_Response>::value && has_bounded_size<service_msgs::msg::ServiceEventInfo>::value> {};

template<>
struct is_message<bolide_interfaces::srv::LaunchEkf_Event>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<bolide_interfaces::srv::LaunchEkf>()
{
  return "bolide_interfaces::srv::LaunchEkf";
}

template<>
inline const char * name<bolide_interfaces::srv::LaunchEkf>()
{
  return "bolide_interfaces/srv/LaunchEkf";
}

template<>
struct has_fixed_size<bolide_interfaces::srv::LaunchEkf>
  : std::integral_constant<
    bool,
    has_fixed_size<bolide_interfaces::srv::LaunchEkf_Request>::value &&
    has_fixed_size<bolide_interfaces::srv::LaunchEkf_Response>::value
  >
{
};

template<>
struct has_bounded_size<bolide_interfaces::srv::LaunchEkf>
  : std::integral_constant<
    bool,
    has_bounded_size<bolide_interfaces::srv::LaunchEkf_Request>::value &&
    has_bounded_size<bolide_interfaces::srv::LaunchEkf_Response>::value
  >
{
};

template<>
struct is_service<bolide_interfaces::srv::LaunchEkf>
  : std::true_type
{
};

template<>
struct is_service_request<bolide_interfaces::srv::LaunchEkf_Request>
  : std::true_type
{
};

template<>
struct is_service_response<bolide_interfaces::srv::LaunchEkf_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // BOLIDE_INTERFACES__SRV__DETAIL__LAUNCH_EKF__TRAITS_HPP_
