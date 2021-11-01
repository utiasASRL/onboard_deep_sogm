// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from msg_interfaces:msg/VoxGrid.idl
// generated code does not contain a copyright notice
#include "msg_interfaces/msg/vox_grid__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header__functions.h"
// Member `origin`
#include "geometry_msgs/msg/point__functions.h"
// Member `data`
#include "rosidl_generator_c/primitives_sequence_functions.h"

bool
msg_interfaces__msg__VoxGrid__init(msg_interfaces__msg__VoxGrid * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    msg_interfaces__msg__VoxGrid__fini(msg);
    return false;
  }
  // height
  // width
  // depth
  // dl
  // dt
  // origin
  if (!geometry_msgs__msg__Point__init(&msg->origin)) {
    msg_interfaces__msg__VoxGrid__fini(msg);
    return false;
  }
  // theta
  // data
  if (!rosidl_generator_c__uint8__Sequence__init(&msg->data, 0)) {
    msg_interfaces__msg__VoxGrid__fini(msg);
    return false;
  }
  return true;
}

void
msg_interfaces__msg__VoxGrid__fini(msg_interfaces__msg__VoxGrid * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // height
  // width
  // depth
  // dl
  // dt
  // origin
  geometry_msgs__msg__Point__fini(&msg->origin);
  // theta
  // data
  rosidl_generator_c__uint8__Sequence__fini(&msg->data);
}

msg_interfaces__msg__VoxGrid *
msg_interfaces__msg__VoxGrid__create()
{
  msg_interfaces__msg__VoxGrid * msg = (msg_interfaces__msg__VoxGrid *)malloc(sizeof(msg_interfaces__msg__VoxGrid));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(msg_interfaces__msg__VoxGrid));
  bool success = msg_interfaces__msg__VoxGrid__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
msg_interfaces__msg__VoxGrid__destroy(msg_interfaces__msg__VoxGrid * msg)
{
  if (msg) {
    msg_interfaces__msg__VoxGrid__fini(msg);
  }
  free(msg);
}


bool
msg_interfaces__msg__VoxGrid__Sequence__init(msg_interfaces__msg__VoxGrid__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  msg_interfaces__msg__VoxGrid * data = NULL;
  if (size) {
    data = (msg_interfaces__msg__VoxGrid *)calloc(size, sizeof(msg_interfaces__msg__VoxGrid));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = msg_interfaces__msg__VoxGrid__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        msg_interfaces__msg__VoxGrid__fini(&data[i - 1]);
      }
      free(data);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
msg_interfaces__msg__VoxGrid__Sequence__fini(msg_interfaces__msg__VoxGrid__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      msg_interfaces__msg__VoxGrid__fini(&array->data[i]);
    }
    free(array->data);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

msg_interfaces__msg__VoxGrid__Sequence *
msg_interfaces__msg__VoxGrid__Sequence__create(size_t size)
{
  msg_interfaces__msg__VoxGrid__Sequence * array = (msg_interfaces__msg__VoxGrid__Sequence *)malloc(sizeof(msg_interfaces__msg__VoxGrid__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = msg_interfaces__msg__VoxGrid__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
msg_interfaces__msg__VoxGrid__Sequence__destroy(msg_interfaces__msg__VoxGrid__Sequence * array)
{
  if (array) {
    msg_interfaces__msg__VoxGrid__Sequence__fini(array);
  }
  free(array);
}
