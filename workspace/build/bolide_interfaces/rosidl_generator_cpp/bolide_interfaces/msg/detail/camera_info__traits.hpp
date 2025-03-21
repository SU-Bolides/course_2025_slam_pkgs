// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from bolide_interfaces:msg/CameraInfo.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "bolide_interfaces/msg/camera_info.hpp"


#ifndef BOLIDE_INTERFACES__MSG__DETAIL__CAMERA_INFO__TRAITS_HPP_
#define BOLIDE_INTERFACES__MSG__DETAIL__CAMERA_INFO__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "bolide_interfaces/msg/detail/camera_info__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace bolide_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const CameraInfo & msg,
  std::ostream & out)
{
  out << "{";
  // member: wrong_way
  {
    out << "wrong_way: ";
    rosidl_generator_traits::value_to_yaml(msg.wrong_way, out);
    out << ", ";
  }

  // member: front_color
  {
    out << "front_color: ";
    rosidl_generator_traits::value_to_yaml(msg.front_color, out);
    out << ", ";
  }

  // member: left_color
  {
    out << "left_color: ";
    rosidl_generator_traits::value_to_yaml(msg.left_color, out);
    out << ", ";
  }

  // member: right_color
  {
    out << "right_color: ";
    rosidl_generator_traits::value_to_yaml(msg.right_color, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const CameraInfo & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: wrong_way
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "wrong_way: ";
    rosidl_generator_traits::value_to_yaml(msg.wrong_way, out);
    out << "\n";
  }

  // member: front_color
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "front_color: ";
    rosidl_generator_traits::value_to_yaml(msg.front_color, out);
    out << "\n";
  }

  // member: left_color
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "left_color: ";
    rosidl_generator_traits::value_to_yaml(msg.left_color, out);
    out << "\n";
  }

  // member: right_color
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "right_color: ";
    rosidl_generator_traits::value_to_yaml(msg.right_color, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const CameraInfo & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace bolide_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use bolide_interfaces::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const bolide_interfaces::msg::CameraInfo & msg,
  std::ostream & out, size_t indentation = 0)
{
  bolide_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use bolide_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const bolide_interfaces::msg::CameraInfo & msg)
{
  return bolide_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<bolide_interfaces::msg::CameraInfo>()
{
  return "bolide_interfaces::msg::CameraInfo";
}

template<>
inline const char * name<bolide_interfaces::msg::CameraInfo>()
{
  return "bolide_interfaces/msg/CameraInfo";
}

template<>
struct has_fixed_size<bolide_interfaces::msg::CameraInfo>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<bolide_interfaces::msg::CameraInfo>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<bolide_interfaces::msg::CameraInfo>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // BOLIDE_INTERFACES__MSG__DETAIL__CAMERA_INFO__TRAITS_HPP_
