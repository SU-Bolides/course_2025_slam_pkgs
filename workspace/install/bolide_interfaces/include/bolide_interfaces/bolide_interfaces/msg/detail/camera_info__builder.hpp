// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from bolide_interfaces:msg/CameraInfo.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "bolide_interfaces/msg/camera_info.hpp"


#ifndef BOLIDE_INTERFACES__MSG__DETAIL__CAMERA_INFO__BUILDER_HPP_
#define BOLIDE_INTERFACES__MSG__DETAIL__CAMERA_INFO__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "bolide_interfaces/msg/detail/camera_info__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace bolide_interfaces
{

namespace msg
{

namespace builder
{

class Init_CameraInfo_right_color
{
public:
  explicit Init_CameraInfo_right_color(::bolide_interfaces::msg::CameraInfo & msg)
  : msg_(msg)
  {}
  ::bolide_interfaces::msg::CameraInfo right_color(::bolide_interfaces::msg::CameraInfo::_right_color_type arg)
  {
    msg_.right_color = std::move(arg);
    return std::move(msg_);
  }

private:
  ::bolide_interfaces::msg::CameraInfo msg_;
};

class Init_CameraInfo_left_color
{
public:
  explicit Init_CameraInfo_left_color(::bolide_interfaces::msg::CameraInfo & msg)
  : msg_(msg)
  {}
  Init_CameraInfo_right_color left_color(::bolide_interfaces::msg::CameraInfo::_left_color_type arg)
  {
    msg_.left_color = std::move(arg);
    return Init_CameraInfo_right_color(msg_);
  }

private:
  ::bolide_interfaces::msg::CameraInfo msg_;
};

class Init_CameraInfo_front_color
{
public:
  explicit Init_CameraInfo_front_color(::bolide_interfaces::msg::CameraInfo & msg)
  : msg_(msg)
  {}
  Init_CameraInfo_left_color front_color(::bolide_interfaces::msg::CameraInfo::_front_color_type arg)
  {
    msg_.front_color = std::move(arg);
    return Init_CameraInfo_left_color(msg_);
  }

private:
  ::bolide_interfaces::msg::CameraInfo msg_;
};

class Init_CameraInfo_wrong_way
{
public:
  Init_CameraInfo_wrong_way()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CameraInfo_front_color wrong_way(::bolide_interfaces::msg::CameraInfo::_wrong_way_type arg)
  {
    msg_.wrong_way = std::move(arg);
    return Init_CameraInfo_front_color(msg_);
  }

private:
  ::bolide_interfaces::msg::CameraInfo msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::bolide_interfaces::msg::CameraInfo>()
{
  return bolide_interfaces::msg::builder::Init_CameraInfo_wrong_way();
}

}  // namespace bolide_interfaces

#endif  // BOLIDE_INTERFACES__MSG__DETAIL__CAMERA_INFO__BUILDER_HPP_
