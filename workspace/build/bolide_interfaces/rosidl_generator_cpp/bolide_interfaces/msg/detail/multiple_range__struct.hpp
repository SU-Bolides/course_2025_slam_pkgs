// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from bolide_interfaces:msg/MultipleRange.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "bolide_interfaces/msg/multiple_range.hpp"


#ifndef BOLIDE_INTERFACES__MSG__DETAIL__MULTIPLE_RANGE__STRUCT_HPP_
#define BOLIDE_INTERFACES__MSG__DETAIL__MULTIPLE_RANGE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'ir_rear_left'
// Member 'ir_rear_right'
// Member 'sonar_rear'
#include "sensor_msgs/msg/detail/range__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__bolide_interfaces__msg__MultipleRange __attribute__((deprecated))
#else
# define DEPRECATED__bolide_interfaces__msg__MultipleRange __declspec(deprecated)
#endif

namespace bolide_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MultipleRange_
{
  using Type = MultipleRange_<ContainerAllocator>;

  explicit MultipleRange_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : ir_rear_left(_init),
    ir_rear_right(_init),
    sonar_rear(_init)
  {
    (void)_init;
  }

  explicit MultipleRange_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : ir_rear_left(_alloc, _init),
    ir_rear_right(_alloc, _init),
    sonar_rear(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _ir_rear_left_type =
    sensor_msgs::msg::Range_<ContainerAllocator>;
  _ir_rear_left_type ir_rear_left;
  using _ir_rear_right_type =
    sensor_msgs::msg::Range_<ContainerAllocator>;
  _ir_rear_right_type ir_rear_right;
  using _sonar_rear_type =
    sensor_msgs::msg::Range_<ContainerAllocator>;
  _sonar_rear_type sonar_rear;

  // setters for named parameter idiom
  Type & set__ir_rear_left(
    const sensor_msgs::msg::Range_<ContainerAllocator> & _arg)
  {
    this->ir_rear_left = _arg;
    return *this;
  }
  Type & set__ir_rear_right(
    const sensor_msgs::msg::Range_<ContainerAllocator> & _arg)
  {
    this->ir_rear_right = _arg;
    return *this;
  }
  Type & set__sonar_rear(
    const sensor_msgs::msg::Range_<ContainerAllocator> & _arg)
  {
    this->sonar_rear = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    bolide_interfaces::msg::MultipleRange_<ContainerAllocator> *;
  using ConstRawPtr =
    const bolide_interfaces::msg::MultipleRange_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<bolide_interfaces::msg::MultipleRange_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<bolide_interfaces::msg::MultipleRange_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      bolide_interfaces::msg::MultipleRange_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<bolide_interfaces::msg::MultipleRange_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      bolide_interfaces::msg::MultipleRange_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<bolide_interfaces::msg::MultipleRange_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<bolide_interfaces::msg::MultipleRange_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<bolide_interfaces::msg::MultipleRange_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__bolide_interfaces__msg__MultipleRange
    std::shared_ptr<bolide_interfaces::msg::MultipleRange_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__bolide_interfaces__msg__MultipleRange
    std::shared_ptr<bolide_interfaces::msg::MultipleRange_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MultipleRange_ & other) const
  {
    if (this->ir_rear_left != other.ir_rear_left) {
      return false;
    }
    if (this->ir_rear_right != other.ir_rear_right) {
      return false;
    }
    if (this->sonar_rear != other.sonar_rear) {
      return false;
    }
    return true;
  }
  bool operator!=(const MultipleRange_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MultipleRange_

// alias to use template instance with default allocator
using MultipleRange =
  bolide_interfaces::msg::MultipleRange_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace bolide_interfaces

#endif  // BOLIDE_INTERFACES__MSG__DETAIL__MULTIPLE_RANGE__STRUCT_HPP_
