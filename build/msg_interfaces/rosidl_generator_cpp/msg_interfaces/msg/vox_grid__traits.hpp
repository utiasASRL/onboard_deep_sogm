// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from msg_interfaces:msg/VoxGrid.idl
// generated code does not contain a copyright notice

#ifndef MSG_INTERFACES__MSG__VOX_GRID__TRAITS_HPP_
#define MSG_INTERFACES__MSG__VOX_GRID__TRAITS_HPP_

#include "msg_interfaces/msg/vox_grid__struct.hpp"
#include <rosidl_generator_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/header__traits.hpp"
// Member 'origin'
#include "geometry_msgs/msg/point__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<msg_interfaces::msg::VoxGrid>()
{
  return "msg_interfaces::msg::VoxGrid";
}

template<>
struct has_fixed_size<msg_interfaces::msg::VoxGrid>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<msg_interfaces::msg::VoxGrid>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<msg_interfaces::msg::VoxGrid>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MSG_INTERFACES__MSG__VOX_GRID__TRAITS_HPP_
