// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from bolide_interfaces:msg/ForkSpeed.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "bolide_interfaces/msg/fork_speed.hpp"


#ifndef BOLIDE_INTERFACES__MSG__DETAIL__FORK_SPEED__STRUCT_HPP_
#define BOLIDE_INTERFACES__MSG__DETAIL__FORK_SPEED__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__bolide_interfaces__msg__ForkSpeed __attribute__((deprecated))
#else
# define DEPRECATED__bolide_interfaces__msg__ForkSpeed __declspec(deprecated)
#endif

namespace bolide_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ForkSpeed_
{
  using Type = ForkSpeed_<ContainerAllocator>;

  explicit ForkSpeed_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->speed = 0.0f;
    }
  }

  explicit ForkSpeed_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->speed = 0.0f;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _speed_type =
    float;
  _speed_type speed;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__speed(
    const float & _arg)
  {
    this->speed = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    bolide_interfaces::msg::ForkSpeed_<ContainerAllocator> *;
  using ConstRawPtr =
    const bolide_interfaces::msg::ForkSpeed_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<bolide_interfaces::msg::ForkSpeed_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<bolide_interfaces::msg::ForkSpeed_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      bolide_interfaces::msg::ForkSpeed_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<bolide_interfaces::msg::ForkSpeed_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      bolide_interfaces::msg::ForkSpeed_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<bolide_interfaces::msg::ForkSpeed_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<bolide_interfaces::msg::ForkSpeed_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<bolide_interfaces::msg::ForkSpeed_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__bolide_interfaces__msg__ForkSpeed
    std::shared_ptr<bolide_interfaces::msg::ForkSpeed_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__bolide_interfaces__msg__ForkSpeed
    std::shared_ptr<bolide_interfaces::msg::ForkSpeed_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ForkSpeed_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->speed != other.speed) {
      return false;
    }
    return true;
  }
  bool operator!=(const ForkSpeed_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ForkSpeed_

// alias to use template instance with default allocator
using ForkSpeed =
  bolide_interfaces::msg::ForkSpeed_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace bolide_interfaces

#endif  // BOLIDE_INTERFACES__MSG__DETAIL__FORK_SPEED__STRUCT_HPP_
