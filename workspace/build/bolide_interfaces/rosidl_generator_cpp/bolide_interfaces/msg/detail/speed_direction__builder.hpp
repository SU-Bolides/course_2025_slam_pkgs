// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from bolide_interfaces:msg/SpeedDirection.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "bolide_interfaces/msg/speed_direction.hpp"


#ifndef BOLIDE_INTERFACES__MSG__DETAIL__SPEED_DIRECTION__BUILDER_HPP_
#define BOLIDE_INTERFACES__MSG__DETAIL__SPEED_DIRECTION__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "bolide_interfaces/msg/detail/speed_direction__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace bolide_interfaces
{

namespace msg
{

namespace builder
{

class Init_SpeedDirection_direction
{
public:
  explicit Init_SpeedDirection_direction(::bolide_interfaces::msg::SpeedDirection & msg)
  : msg_(msg)
  {}
  ::bolide_interfaces::msg::SpeedDirection direction(::bolide_interfaces::msg::SpeedDirection::_direction_type arg)
  {
    msg_.direction = std::move(arg);
    return std::move(msg_);
  }

private:
  ::bolide_interfaces::msg::SpeedDirection msg_;
};

class Init_SpeedDirection_speed
{
public:
  Init_SpeedDirection_speed()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SpeedDirection_direction speed(::bolide_interfaces::msg::SpeedDirection::_speed_type arg)
  {
    msg_.speed = std::move(arg);
    return Init_SpeedDirection_direction(msg_);
  }

private:
  ::bolide_interfaces::msg::SpeedDirection msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::bolide_interfaces::msg::SpeedDirection>()
{
  return bolide_interfaces::msg::builder::Init_SpeedDirection_speed();
}

}  // namespace bolide_interfaces

#endif  // BOLIDE_INTERFACES__MSG__DETAIL__SPEED_DIRECTION__BUILDER_HPP_
