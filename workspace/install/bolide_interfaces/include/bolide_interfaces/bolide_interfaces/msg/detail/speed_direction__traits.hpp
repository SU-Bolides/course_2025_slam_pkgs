// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from bolide_interfaces:msg/SpeedDirection.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "bolide_interfaces/msg/speed_direction.hpp"


#ifndef BOLIDE_INTERFACES__MSG__DETAIL__SPEED_DIRECTION__TRAITS_HPP_
#define BOLIDE_INTERFACES__MSG__DETAIL__SPEED_DIRECTION__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "bolide_interfaces/msg/detail/speed_direction__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace bolide_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const SpeedDirection & msg,
  std::ostream & out)
{
  out << "{";
  // member: speed
  {
    out << "speed: ";
    rosidl_generator_traits::value_to_yaml(msg.speed, out);
    out << ", ";
  }

  // member: direction
  {
    out << "direction: ";
    rosidl_generator_traits::value_to_yaml(msg.direction, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SpeedDirection & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: speed
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "speed: ";
    rosidl_generator_traits::value_to_yaml(msg.speed, out);
    out << "\n";
  }

  // member: direction
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "direction: ";
    rosidl_generator_traits::value_to_yaml(msg.direction, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SpeedDirection & msg, bool use_flow_style = false)
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
  const bolide_interfaces::msg::SpeedDirection & msg,
  std::ostream & out, size_t indentation = 0)
{
  bolide_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use bolide_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const bolide_interfaces::msg::SpeedDirection & msg)
{
  return bolide_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<bolide_interfaces::msg::SpeedDirection>()
{
  return "bolide_interfaces::msg::SpeedDirection";
}

template<>
inline const char * name<bolide_interfaces::msg::SpeedDirection>()
{
  return "bolide_interfaces/msg/SpeedDirection";
}

template<>
struct has_fixed_size<bolide_interfaces::msg::SpeedDirection>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<bolide_interfaces::msg::SpeedDirection>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<bolide_interfaces::msg::SpeedDirection>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // BOLIDE_INTERFACES__MSG__DETAIL__SPEED_DIRECTION__TRAITS_HPP_
