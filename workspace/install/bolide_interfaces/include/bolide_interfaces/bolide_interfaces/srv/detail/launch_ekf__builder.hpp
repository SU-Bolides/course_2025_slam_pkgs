// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from bolide_interfaces:srv/LaunchEkf.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "bolide_interfaces/srv/launch_ekf.hpp"


#ifndef BOLIDE_INTERFACES__SRV__DETAIL__LAUNCH_EKF__BUILDER_HPP_
#define BOLIDE_INTERFACES__SRV__DETAIL__LAUNCH_EKF__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "bolide_interfaces/srv/detail/launch_ekf__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace bolide_interfaces
{

namespace srv
{

namespace builder
{

class Init_LaunchEkf_Request_start
{
public:
  Init_LaunchEkf_Request_start()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::bolide_interfaces::srv::LaunchEkf_Request start(::bolide_interfaces::srv::LaunchEkf_Request::_start_type arg)
  {
    msg_.start = std::move(arg);
    return std::move(msg_);
  }

private:
  ::bolide_interfaces::srv::LaunchEkf_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::bolide_interfaces::srv::LaunchEkf_Request>()
{
  return bolide_interfaces::srv::builder::Init_LaunchEkf_Request_start();
}

}  // namespace bolide_interfaces


namespace bolide_interfaces
{

namespace srv
{

namespace builder
{

class Init_LaunchEkf_Response_gotcha
{
public:
  Init_LaunchEkf_Response_gotcha()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::bolide_interfaces::srv::LaunchEkf_Response gotcha(::bolide_interfaces::srv::LaunchEkf_Response::_gotcha_type arg)
  {
    msg_.gotcha = std::move(arg);
    return std::move(msg_);
  }

private:
  ::bolide_interfaces::srv::LaunchEkf_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::bolide_interfaces::srv::LaunchEkf_Response>()
{
  return bolide_interfaces::srv::builder::Init_LaunchEkf_Response_gotcha();
}

}  // namespace bolide_interfaces


namespace bolide_interfaces
{

namespace srv
{

namespace builder
{

class Init_LaunchEkf_Event_response
{
public:
  explicit Init_LaunchEkf_Event_response(::bolide_interfaces::srv::LaunchEkf_Event & msg)
  : msg_(msg)
  {}
  ::bolide_interfaces::srv::LaunchEkf_Event response(::bolide_interfaces::srv::LaunchEkf_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::bolide_interfaces::srv::LaunchEkf_Event msg_;
};

class Init_LaunchEkf_Event_request
{
public:
  explicit Init_LaunchEkf_Event_request(::bolide_interfaces::srv::LaunchEkf_Event & msg)
  : msg_(msg)
  {}
  Init_LaunchEkf_Event_response request(::bolide_interfaces::srv::LaunchEkf_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_LaunchEkf_Event_response(msg_);
  }

private:
  ::bolide_interfaces::srv::LaunchEkf_Event msg_;
};

class Init_LaunchEkf_Event_info
{
public:
  Init_LaunchEkf_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_LaunchEkf_Event_request info(::bolide_interfaces::srv::LaunchEkf_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_LaunchEkf_Event_request(msg_);
  }

private:
  ::bolide_interfaces::srv::LaunchEkf_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::bolide_interfaces::srv::LaunchEkf_Event>()
{
  return bolide_interfaces::srv::builder::Init_LaunchEkf_Event_info();
}

}  // namespace bolide_interfaces

#endif  // BOLIDE_INTERFACES__SRV__DETAIL__LAUNCH_EKF__BUILDER_HPP_
