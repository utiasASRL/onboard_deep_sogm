// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from msg_interfaces:msg/VoxGrid.idl
// generated code does not contain a copyright notice

#ifndef MSG_INTERFACES__MSG__VOX_GRID__FUNCTIONS_H_
#define MSG_INTERFACES__MSG__VOX_GRID__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_generator_c/visibility_control.h"
#include "msg_interfaces/msg/rosidl_generator_c__visibility_control.h"

#include "msg_interfaces/msg/vox_grid__struct.h"

/// Initialize msg/VoxGrid message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * msg_interfaces__msg__VoxGrid
 * )) before or use
 * msg_interfaces__msg__VoxGrid__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_msg_interfaces
bool
msg_interfaces__msg__VoxGrid__init(msg_interfaces__msg__VoxGrid * msg);

/// Finalize msg/VoxGrid message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_msg_interfaces
void
msg_interfaces__msg__VoxGrid__fini(msg_interfaces__msg__VoxGrid * msg);

/// Create msg/VoxGrid message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * msg_interfaces__msg__VoxGrid__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_msg_interfaces
msg_interfaces__msg__VoxGrid *
msg_interfaces__msg__VoxGrid__create();

/// Destroy msg/VoxGrid message.
/**
 * It calls
 * msg_interfaces__msg__VoxGrid__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_msg_interfaces
void
msg_interfaces__msg__VoxGrid__destroy(msg_interfaces__msg__VoxGrid * msg);


/// Initialize array of msg/VoxGrid messages.
/**
 * It allocates the memory for the number of elements and calls
 * msg_interfaces__msg__VoxGrid__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_msg_interfaces
bool
msg_interfaces__msg__VoxGrid__Sequence__init(msg_interfaces__msg__VoxGrid__Sequence * array, size_t size);

/// Finalize array of msg/VoxGrid messages.
/**
 * It calls
 * msg_interfaces__msg__VoxGrid__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_msg_interfaces
void
msg_interfaces__msg__VoxGrid__Sequence__fini(msg_interfaces__msg__VoxGrid__Sequence * array);

/// Create array of msg/VoxGrid messages.
/**
 * It allocates the memory for the array and calls
 * msg_interfaces__msg__VoxGrid__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_msg_interfaces
msg_interfaces__msg__VoxGrid__Sequence *
msg_interfaces__msg__VoxGrid__Sequence__create(size_t size);

/// Destroy array of msg/VoxGrid messages.
/**
 * It calls
 * msg_interfaces__msg__VoxGrid__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_msg_interfaces
void
msg_interfaces__msg__VoxGrid__Sequence__destroy(msg_interfaces__msg__VoxGrid__Sequence * array);

#ifdef __cplusplus
}
#endif

#endif  // MSG_INTERFACES__MSG__VOX_GRID__FUNCTIONS_H_
