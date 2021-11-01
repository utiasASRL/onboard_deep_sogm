// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from msg_interfaces:msg/VoxGrid.idl
// generated code does not contain a copyright notice

#ifndef MSG_INTERFACES__MSG__VOX_GRID__STRUCT_HPP_
#define MSG_INTERFACES__MSG__VOX_GRID__STRUCT_HPP_

#include <rosidl_generator_cpp/bounded_vector.hpp>
#include <rosidl_generator_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/header__struct.hpp"
// Member 'origin'
#include "geometry_msgs/msg/point__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__msg_interfaces__msg__VoxGrid __attribute__((deprecated))
#else
# define DEPRECATED__msg_interfaces__msg__VoxGrid __declspec(deprecated)
#endif

namespace msg_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct VoxGrid_
{
  using Type = VoxGrid_<ContainerAllocator>;

  explicit VoxGrid_(rosidl_generator_cpp::MessageInitialization _init = rosidl_generator_cpp::MessageInitialization::ALL)
  : header(_init),
    origin(_init)
  {
    if (rosidl_generator_cpp::MessageInitialization::ALL == _init ||
      rosidl_generator_cpp::MessageInitialization::ZERO == _init)
    {
      this->height = 0ul;
      this->width = 0ul;
      this->depth = 0ul;
      this->dl = 0.0f;
      this->dt = 0.0f;
      this->theta = 0.0f;
    }
  }

  explicit VoxGrid_(const ContainerAllocator & _alloc, rosidl_generator_cpp::MessageInitialization _init = rosidl_generator_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    origin(_alloc, _init)
  {
    if (rosidl_generator_cpp::MessageInitialization::ALL == _init ||
      rosidl_generator_cpp::MessageInitialization::ZERO == _init)
    {
      this->height = 0ul;
      this->width = 0ul;
      this->depth = 0ul;
      this->dl = 0.0f;
      this->dt = 0.0f;
      this->theta = 0.0f;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _height_type =
    uint32_t;
  _height_type height;
  using _width_type =
    uint32_t;
  _width_type width;
  using _depth_type =
    uint32_t;
  _depth_type depth;
  using _dl_type =
    float;
  _dl_type dl;
  using _dt_type =
    float;
  _dt_type dt;
  using _origin_type =
    geometry_msgs::msg::Point_<ContainerAllocator>;
  _origin_type origin;
  using _theta_type =
    float;
  _theta_type theta;
  using _data_type =
    std::vector<uint8_t, typename ContainerAllocator::template rebind<uint8_t>::other>;
  _data_type data;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__height(
    const uint32_t & _arg)
  {
    this->height = _arg;
    return *this;
  }
  Type & set__width(
    const uint32_t & _arg)
  {
    this->width = _arg;
    return *this;
  }
  Type & set__depth(
    const uint32_t & _arg)
  {
    this->depth = _arg;
    return *this;
  }
  Type & set__dl(
    const float & _arg)
  {
    this->dl = _arg;
    return *this;
  }
  Type & set__dt(
    const float & _arg)
  {
    this->dt = _arg;
    return *this;
  }
  Type & set__origin(
    const geometry_msgs::msg::Point_<ContainerAllocator> & _arg)
  {
    this->origin = _arg;
    return *this;
  }
  Type & set__theta(
    const float & _arg)
  {
    this->theta = _arg;
    return *this;
  }
  Type & set__data(
    const std::vector<uint8_t, typename ContainerAllocator::template rebind<uint8_t>::other> & _arg)
  {
    this->data = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    msg_interfaces::msg::VoxGrid_<ContainerAllocator> *;
  using ConstRawPtr =
    const msg_interfaces::msg::VoxGrid_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<msg_interfaces::msg::VoxGrid_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<msg_interfaces::msg::VoxGrid_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      msg_interfaces::msg::VoxGrid_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<msg_interfaces::msg::VoxGrid_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      msg_interfaces::msg::VoxGrid_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<msg_interfaces::msg::VoxGrid_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<msg_interfaces::msg::VoxGrid_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<msg_interfaces::msg::VoxGrid_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__msg_interfaces__msg__VoxGrid
    std::shared_ptr<msg_interfaces::msg::VoxGrid_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__msg_interfaces__msg__VoxGrid
    std::shared_ptr<msg_interfaces::msg::VoxGrid_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const VoxGrid_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->height != other.height) {
      return false;
    }
    if (this->width != other.width) {
      return false;
    }
    if (this->depth != other.depth) {
      return false;
    }
    if (this->dl != other.dl) {
      return false;
    }
    if (this->dt != other.dt) {
      return false;
    }
    if (this->origin != other.origin) {
      return false;
    }
    if (this->theta != other.theta) {
      return false;
    }
    if (this->data != other.data) {
      return false;
    }
    return true;
  }
  bool operator!=(const VoxGrid_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct VoxGrid_

// alias to use template instance with default allocator
using VoxGrid =
  msg_interfaces::msg::VoxGrid_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace msg_interfaces

#endif  // MSG_INTERFACES__MSG__VOX_GRID__STRUCT_HPP_
