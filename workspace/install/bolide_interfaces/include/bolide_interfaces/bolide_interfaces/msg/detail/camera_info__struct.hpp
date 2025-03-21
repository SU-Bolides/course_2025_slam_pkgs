// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from bolide_interfaces:msg/CameraInfo.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "bolide_interfaces/msg/camera_info.hpp"


#ifndef BOLIDE_INTERFACES__MSG__DETAIL__CAMERA_INFO__STRUCT_HPP_
#define BOLIDE_INTERFACES__MSG__DETAIL__CAMERA_INFO__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__bolide_interfaces__msg__CameraInfo __attribute__((deprecated))
#else
# define DEPRECATED__bolide_interfaces__msg__CameraInfo __declspec(deprecated)
#endif

namespace bolide_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct CameraInfo_
{
  using Type = CameraInfo_<ContainerAllocator>;

  explicit CameraInfo_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->wrong_way = false;
      this->front_color = "";
      this->left_color = "";
      this->right_color = "";
    }
  }

  explicit CameraInfo_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : front_color(_alloc),
    left_color(_alloc),
    right_color(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->wrong_way = false;
      this->front_color = "";
      this->left_color = "";
      this->right_color = "";
    }
  }

  // field types and members
  using _wrong_way_type =
    bool;
  _wrong_way_type wrong_way;
  using _front_color_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _front_color_type front_color;
  using _left_color_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _left_color_type left_color;
  using _right_color_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _right_color_type right_color;

  // setters for named parameter idiom
  Type & set__wrong_way(
    const bool & _arg)
  {
    this->wrong_way = _arg;
    return *this;
  }
  Type & set__front_color(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->front_color = _arg;
    return *this;
  }
  Type & set__left_color(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->left_color = _arg;
    return *this;
  }
  Type & set__right_color(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->right_color = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    bolide_interfaces::msg::CameraInfo_<ContainerAllocator> *;
  using ConstRawPtr =
    const bolide_interfaces::msg::CameraInfo_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<bolide_interfaces::msg::CameraInfo_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<bolide_interfaces::msg::CameraInfo_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      bolide_interfaces::msg::CameraInfo_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<bolide_interfaces::msg::CameraInfo_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      bolide_interfaces::msg::CameraInfo_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<bolide_interfaces::msg::CameraInfo_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<bolide_interfaces::msg::CameraInfo_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<bolide_interfaces::msg::CameraInfo_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__bolide_interfaces__msg__CameraInfo
    std::shared_ptr<bolide_interfaces::msg::CameraInfo_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__bolide_interfaces__msg__CameraInfo
    std::shared_ptr<bolide_interfaces::msg::CameraInfo_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const CameraInfo_ & other) const
  {
    if (this->wrong_way != other.wrong_way) {
      return false;
    }
    if (this->front_color != other.front_color) {
      return false;
    }
    if (this->left_color != other.left_color) {
      return false;
    }
    if (this->right_color != other.right_color) {
      return false;
    }
    return true;
  }
  bool operator!=(const CameraInfo_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct CameraInfo_

// alias to use template instance with default allocator
using CameraInfo =
  bolide_interfaces::msg::CameraInfo_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace bolide_interfaces

#endif  // BOLIDE_INTERFACES__MSG__DETAIL__CAMERA_INFO__STRUCT_HPP_
