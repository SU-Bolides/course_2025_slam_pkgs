// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from bolide_interfaces:msg/ForkSpeed.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "bolide_interfaces/msg/fork_speed.hpp"


#ifndef BOLIDE_INTERFACES__MSG__DETAIL__FORK_SPEED__BUILDER_HPP_
#define BOLIDE_INTERFACES__MSG__DETAIL__FORK_SPEED__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "bolide_interfaces/msg/detail/fork_speed__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace bolide_interfaces
{

namespace msg
{

namespace builder
{

class Init_ForkSpeed_speed
{
public:
  explicit Init_ForkSpeed_speed(::bolide_interfaces::msg::ForkSpeed & msg)
  : msg_(msg)
  {}
  ::bolide_interfaces::msg::ForkSpeed speed(::bolide_interfaces::msg::ForkSpeed::_speed_type arg)
  {
    msg_.speed = std::move(arg);
    return std::move(msg_);
  }

private:
  ::bolide_interfaces::msg::ForkSpeed msg_;
};

class Init_ForkSpeed_header
{
public:
  Init_ForkSpeed_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ForkSpeed_speed header(::bolide_interfaces::msg::ForkSpeed::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_ForkSpeed_speed(msg_);
  }

private:
  ::bolide_interfaces::msg::ForkSpeed msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::bolide_interfaces::msg::ForkSpeed>()
{
  return bolide_interfaces::msg::builder::Init_ForkSpeed_header();
}

}  // namespace bolide_interfaces

#endif  // BOLIDE_INTERFACES__MSG__DETAIL__FORK_SPEED__BUILDER_HPP_
