// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from bolide_interfaces:msg/SpeedDirection.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "bolide_interfaces/msg/speed_direction.hpp"


#ifndef BOLIDE_INTERFACES__MSG__DETAIL__SPEED_DIRECTION__STRUCT_HPP_
#define BOLIDE_INTERFACES__MSG__DETAIL__SPEED_DIRECTION__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__bolide_interfaces__msg__SpeedDirection __attribute__((deprecated))
#else
# define DEPRECATED__bolide_interfaces__msg__SpeedDirection __declspec(deprecated)
#endif

namespace bolide_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct SpeedDirection_
{
  using Type = SpeedDirection_<ContainerAllocator>;

  explicit SpeedDirection_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->speed = 0.0;
      this->direction = 0.0;
    }
  }

  explicit SpeedDirection_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->speed = 0.0;
      this->direction = 0.0;
    }
  }

  // field types and members
  using _speed_type =
    double;
  _speed_type speed;
  using _direction_type =
    double;
  _direction_type direction;

  // setters for named parameter idiom
  Type & set__speed(
    const double & _arg)
  {
    this->speed = _arg;
    return *this;
  }
  Type & set__direction(
    const double & _arg)
  {
    this->direction = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    bolide_interfaces::msg::SpeedDirection_<ContainerAllocator> *;
  using ConstRawPtr =
    const bolide_interfaces::msg::SpeedDirection_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<bolide_interfaces::msg::SpeedDirection_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<bolide_interfaces::msg::SpeedDirection_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      bolide_interfaces::msg::SpeedDirection_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<bolide_interfaces::msg::SpeedDirection_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      bolide_interfaces::msg::SpeedDirection_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<bolide_interfaces::msg::SpeedDirection_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<bolide_interfaces::msg::SpeedDirection_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<bolide_interfaces::msg::SpeedDirection_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__bolide_interfaces__msg__SpeedDirection
    std::shared_ptr<bolide_interfaces::msg::SpeedDirection_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__bolide_interfaces__msg__SpeedDirection
    std::shared_ptr<bolide_interfaces::msg::SpeedDirection_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SpeedDirection_ & other) const
  {
    if (this->speed != other.speed) {
      return false;
    }
    if (this->direction != other.direction) {
      return false;
    }
    return true;
  }
  bool operator!=(const SpeedDirection_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SpeedDirection_

// alias to use template instance with default allocator
using SpeedDirection =
  bolide_interfaces::msg::SpeedDirection_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace bolide_interfaces

#endif  // BOLIDE_INTERFACES__MSG__DETAIL__SPEED_DIRECTION__STRUCT_HPP_
