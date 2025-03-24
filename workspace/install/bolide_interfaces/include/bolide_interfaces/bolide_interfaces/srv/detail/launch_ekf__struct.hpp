// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from bolide_interfaces:srv/LaunchEkf.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "bolide_interfaces/srv/launch_ekf.hpp"


#ifndef BOLIDE_INTERFACES__SRV__DETAIL__LAUNCH_EKF__STRUCT_HPP_
#define BOLIDE_INTERFACES__SRV__DETAIL__LAUNCH_EKF__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__bolide_interfaces__srv__LaunchEkf_Request __attribute__((deprecated))
#else
# define DEPRECATED__bolide_interfaces__srv__LaunchEkf_Request __declspec(deprecated)
#endif

namespace bolide_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct LaunchEkf_Request_
{
  using Type = LaunchEkf_Request_<ContainerAllocator>;

  explicit LaunchEkf_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->start = false;
    }
  }

  explicit LaunchEkf_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->start = false;
    }
  }

  // field types and members
  using _start_type =
    bool;
  _start_type start;

  // setters for named parameter idiom
  Type & set__start(
    const bool & _arg)
  {
    this->start = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    bolide_interfaces::srv::LaunchEkf_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const bolide_interfaces::srv::LaunchEkf_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<bolide_interfaces::srv::LaunchEkf_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<bolide_interfaces::srv::LaunchEkf_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      bolide_interfaces::srv::LaunchEkf_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<bolide_interfaces::srv::LaunchEkf_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      bolide_interfaces::srv::LaunchEkf_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<bolide_interfaces::srv::LaunchEkf_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<bolide_interfaces::srv::LaunchEkf_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<bolide_interfaces::srv::LaunchEkf_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__bolide_interfaces__srv__LaunchEkf_Request
    std::shared_ptr<bolide_interfaces::srv::LaunchEkf_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__bolide_interfaces__srv__LaunchEkf_Request
    std::shared_ptr<bolide_interfaces::srv::LaunchEkf_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const LaunchEkf_Request_ & other) const
  {
    if (this->start != other.start) {
      return false;
    }
    return true;
  }
  bool operator!=(const LaunchEkf_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct LaunchEkf_Request_

// alias to use template instance with default allocator
using LaunchEkf_Request =
  bolide_interfaces::srv::LaunchEkf_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace bolide_interfaces


#ifndef _WIN32
# define DEPRECATED__bolide_interfaces__srv__LaunchEkf_Response __attribute__((deprecated))
#else
# define DEPRECATED__bolide_interfaces__srv__LaunchEkf_Response __declspec(deprecated)
#endif

namespace bolide_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct LaunchEkf_Response_
{
  using Type = LaunchEkf_Response_<ContainerAllocator>;

  explicit LaunchEkf_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->gotcha = false;
    }
  }

  explicit LaunchEkf_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->gotcha = false;
    }
  }

  // field types and members
  using _gotcha_type =
    bool;
  _gotcha_type gotcha;

  // setters for named parameter idiom
  Type & set__gotcha(
    const bool & _arg)
  {
    this->gotcha = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    bolide_interfaces::srv::LaunchEkf_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const bolide_interfaces::srv::LaunchEkf_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<bolide_interfaces::srv::LaunchEkf_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<bolide_interfaces::srv::LaunchEkf_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      bolide_interfaces::srv::LaunchEkf_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<bolide_interfaces::srv::LaunchEkf_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      bolide_interfaces::srv::LaunchEkf_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<bolide_interfaces::srv::LaunchEkf_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<bolide_interfaces::srv::LaunchEkf_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<bolide_interfaces::srv::LaunchEkf_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__bolide_interfaces__srv__LaunchEkf_Response
    std::shared_ptr<bolide_interfaces::srv::LaunchEkf_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__bolide_interfaces__srv__LaunchEkf_Response
    std::shared_ptr<bolide_interfaces::srv::LaunchEkf_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const LaunchEkf_Response_ & other) const
  {
    if (this->gotcha != other.gotcha) {
      return false;
    }
    return true;
  }
  bool operator!=(const LaunchEkf_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct LaunchEkf_Response_

// alias to use template instance with default allocator
using LaunchEkf_Response =
  bolide_interfaces::srv::LaunchEkf_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace bolide_interfaces


// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__bolide_interfaces__srv__LaunchEkf_Event __attribute__((deprecated))
#else
# define DEPRECATED__bolide_interfaces__srv__LaunchEkf_Event __declspec(deprecated)
#endif

namespace bolide_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct LaunchEkf_Event_
{
  using Type = LaunchEkf_Event_<ContainerAllocator>;

  explicit LaunchEkf_Event_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_init)
  {
    (void)_init;
  }

  explicit LaunchEkf_Event_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _info_type =
    service_msgs::msg::ServiceEventInfo_<ContainerAllocator>;
  _info_type info;
  using _request_type =
    rosidl_runtime_cpp::BoundedVector<bolide_interfaces::srv::LaunchEkf_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<bolide_interfaces::srv::LaunchEkf_Request_<ContainerAllocator>>>;
  _request_type request;
  using _response_type =
    rosidl_runtime_cpp::BoundedVector<bolide_interfaces::srv::LaunchEkf_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<bolide_interfaces::srv::LaunchEkf_Response_<ContainerAllocator>>>;
  _response_type response;

  // setters for named parameter idiom
  Type & set__info(
    const service_msgs::msg::ServiceEventInfo_<ContainerAllocator> & _arg)
  {
    this->info = _arg;
    return *this;
  }
  Type & set__request(
    const rosidl_runtime_cpp::BoundedVector<bolide_interfaces::srv::LaunchEkf_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<bolide_interfaces::srv::LaunchEkf_Request_<ContainerAllocator>>> & _arg)
  {
    this->request = _arg;
    return *this;
  }
  Type & set__response(
    const rosidl_runtime_cpp::BoundedVector<bolide_interfaces::srv::LaunchEkf_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<bolide_interfaces::srv::LaunchEkf_Response_<ContainerAllocator>>> & _arg)
  {
    this->response = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    bolide_interfaces::srv::LaunchEkf_Event_<ContainerAllocator> *;
  using ConstRawPtr =
    const bolide_interfaces::srv::LaunchEkf_Event_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<bolide_interfaces::srv::LaunchEkf_Event_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<bolide_interfaces::srv::LaunchEkf_Event_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      bolide_interfaces::srv::LaunchEkf_Event_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<bolide_interfaces::srv::LaunchEkf_Event_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      bolide_interfaces::srv::LaunchEkf_Event_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<bolide_interfaces::srv::LaunchEkf_Event_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<bolide_interfaces::srv::LaunchEkf_Event_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<bolide_interfaces::srv::LaunchEkf_Event_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__bolide_interfaces__srv__LaunchEkf_Event
    std::shared_ptr<bolide_interfaces::srv::LaunchEkf_Event_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__bolide_interfaces__srv__LaunchEkf_Event
    std::shared_ptr<bolide_interfaces::srv::LaunchEkf_Event_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const LaunchEkf_Event_ & other) const
  {
    if (this->info != other.info) {
      return false;
    }
    if (this->request != other.request) {
      return false;
    }
    if (this->response != other.response) {
      return false;
    }
    return true;
  }
  bool operator!=(const LaunchEkf_Event_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct LaunchEkf_Event_

// alias to use template instance with default allocator
using LaunchEkf_Event =
  bolide_interfaces::srv::LaunchEkf_Event_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace bolide_interfaces

namespace bolide_interfaces
{

namespace srv
{

struct LaunchEkf
{
  using Request = bolide_interfaces::srv::LaunchEkf_Request;
  using Response = bolide_interfaces::srv::LaunchEkf_Response;
  using Event = bolide_interfaces::srv::LaunchEkf_Event;
};

}  // namespace srv

}  // namespace bolide_interfaces

#endif  // BOLIDE_INTERFACES__SRV__DETAIL__LAUNCH_EKF__STRUCT_HPP_
