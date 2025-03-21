// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from bolide_interfaces:msg/MultipleRange.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "bolide_interfaces/msg/multiple_range.hpp"


#ifndef BOLIDE_INTERFACES__MSG__DETAIL__MULTIPLE_RANGE__BUILDER_HPP_
#define BOLIDE_INTERFACES__MSG__DETAIL__MULTIPLE_RANGE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "bolide_interfaces/msg/detail/multiple_range__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace bolide_interfaces
{

namespace msg
{

namespace builder
{

class Init_MultipleRange_sonar_rear
{
public:
  explicit Init_MultipleRange_sonar_rear(::bolide_interfaces::msg::MultipleRange & msg)
  : msg_(msg)
  {}
  ::bolide_interfaces::msg::MultipleRange sonar_rear(::bolide_interfaces::msg::MultipleRange::_sonar_rear_type arg)
  {
    msg_.sonar_rear = std::move(arg);
    return std::move(msg_);
  }

private:
  ::bolide_interfaces::msg::MultipleRange msg_;
};

class Init_MultipleRange_ir_rear_right
{
public:
  explicit Init_MultipleRange_ir_rear_right(::bolide_interfaces::msg::MultipleRange & msg)
  : msg_(msg)
  {}
  Init_MultipleRange_sonar_rear ir_rear_right(::bolide_interfaces::msg::MultipleRange::_ir_rear_right_type arg)
  {
    msg_.ir_rear_right = std::move(arg);
    return Init_MultipleRange_sonar_rear(msg_);
  }

private:
  ::bolide_interfaces::msg::MultipleRange msg_;
};

class Init_MultipleRange_ir_rear_left
{
public:
  Init_MultipleRange_ir_rear_left()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MultipleRange_ir_rear_right ir_rear_left(::bolide_interfaces::msg::MultipleRange::_ir_rear_left_type arg)
  {
    msg_.ir_rear_left = std::move(arg);
    return Init_MultipleRange_ir_rear_right(msg_);
  }

private:
  ::bolide_interfaces::msg::MultipleRange msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::bolide_interfaces::msg::MultipleRange>()
{
  return bolide_interfaces::msg::builder::Init_MultipleRange_ir_rear_left();
}

}  // namespace bolide_interfaces

#endif  // BOLIDE_INTERFACES__MSG__DETAIL__MULTIPLE_RANGE__BUILDER_HPP_
