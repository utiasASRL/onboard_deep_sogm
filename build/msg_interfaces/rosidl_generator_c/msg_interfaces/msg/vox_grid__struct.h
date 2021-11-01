// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from msg_interfaces:msg/VoxGrid.idl
// generated code does not contain a copyright notice

#ifndef MSG_INTERFACES__MSG__VOX_GRID__STRUCT_H_
#define MSG_INTERFACES__MSG__VOX_GRID__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/header__struct.h"
// Member 'origin'
#include "geometry_msgs/msg/point__struct.h"
// Member 'data'
#include "rosidl_generator_c/primitives_sequence.h"

// Struct defined in msg/VoxGrid in the package msg_interfaces.
typedef struct msg_interfaces__msg__VoxGrid
{
  std_msgs__msg__Header header;
  uint32_t height;
  uint32_t width;
  uint32_t depth;
  float dl;
  float dt;
  geometry_msgs__msg__Point origin;
  float theta;
  rosidl_generator_c__uint8__Sequence data;
} msg_interfaces__msg__VoxGrid;

// Struct for a sequence of msg_interfaces__msg__VoxGrid.
typedef struct msg_interfaces__msg__VoxGrid__Sequence
{
  msg_interfaces__msg__VoxGrid * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} msg_interfaces__msg__VoxGrid__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MSG_INTERFACES__MSG__VOX_GRID__STRUCT_H_
