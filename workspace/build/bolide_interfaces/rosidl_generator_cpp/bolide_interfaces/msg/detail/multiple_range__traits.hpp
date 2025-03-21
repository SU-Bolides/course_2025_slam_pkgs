// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from bolide_interfaces:msg/MultipleRange.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "bolide_interfaces/msg/multiple_range.hpp"


#ifndef BOLIDE_INTERFACES__MSG__DETAIL__MULTIPLE_RANGE__TRAITS_HPP_
#define BOLIDE_INTERFACES__MSG__DETAIL__MULTIPLE_RANGE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "bolide_interfaces/msg/detail/multiple_range__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'ir_rear_left'
// Member 'ir_rear_right'
// Member 'sonar_rear'
#include "sensor_msgs/msg/detail/range__traits.hpp"

namespace bolide_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const MultipleRange & msg,
  std::ostream & out)
{
  out << "{";
  // member: ir_rear_left
  {
    out << "ir_rear_left: ";
    to_flow_style_yaml(msg.ir_rear_left, out);
    out << ", ";
  }

  // member: ir_rear_right
  {
    out << "ir_rear_right: ";
    to_flow_style_yaml(msg.ir_rear_right, out);
    out << ", ";
  }

  // member: sonar_rear
  {
    out << "sonar_rear: ";
    to_flow_style_yaml(msg.sonar_rear, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const MultipleRange & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: ir_rear_left
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ir_rear_left:\n";
    to_block_style_yaml(msg.ir_rear_left, out, indentation + 2);
  }

  // member: ir_rear_right
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ir_rear_right:\n";
    to_block_style_yaml(msg.ir_rear_right, out, indentation + 2);
  }

  // member: sonar_rear
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "sonar_rear:\n";
    to_block_style_yaml(msg.sonar_rear, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const MultipleRange & msg, bool use_flow_style = false)
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
  const bolide_interfaces::msg::MultipleRange & msg,
  std::ostream & out, size_t indentation = 0)
{
  bolide_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use bolide_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const bolide_interfaces::msg::MultipleRange & msg)
{
  return bolide_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<bolide_interfaces::msg::MultipleRange>()
{
  return "bolide_interfaces::msg::MultipleRange";
}

template<>
inline const char * name<bolide_interfaces::msg::MultipleRange>()
{
  return "bolide_interfaces/msg/MultipleRange";
}

template<>
struct has_fixed_size<bolide_interfaces::msg::MultipleRange>
  : std::integral_constant<bool, has_fixed_size<sensor_msgs::msg::Range>::value> {};

template<>
struct has_bounded_size<bolide_interfaces::msg::MultipleRange>
  : std::integral_constant<bool, has_bounded_size<sensor_msgs::msg::Range>::value> {};

template<>
struct is_message<bolide_interfaces::msg::MultipleRange>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // BOLIDE_INTERFACES__MSG__DETAIL__MULTIPLE_RANGE__TRAITS_HPP_
