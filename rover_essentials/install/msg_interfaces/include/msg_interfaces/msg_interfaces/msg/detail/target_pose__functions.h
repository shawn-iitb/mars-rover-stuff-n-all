// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from msg_interfaces:msg/TargetPose.idl
// generated code does not contain a copyright notice

#ifndef MSG_INTERFACES__MSG__DETAIL__TARGET_POSE__FUNCTIONS_H_
#define MSG_INTERFACES__MSG__DETAIL__TARGET_POSE__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "msg_interfaces/msg/rosidl_generator_c__visibility_control.h"

#include "msg_interfaces/msg/detail/target_pose__struct.h"

/// Initialize msg/TargetPose message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * msg_interfaces__msg__TargetPose
 * )) before or use
 * msg_interfaces__msg__TargetPose__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_msg_interfaces
bool
msg_interfaces__msg__TargetPose__init(msg_interfaces__msg__TargetPose * msg);

/// Finalize msg/TargetPose message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_msg_interfaces
void
msg_interfaces__msg__TargetPose__fini(msg_interfaces__msg__TargetPose * msg);

/// Create msg/TargetPose message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * msg_interfaces__msg__TargetPose__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_msg_interfaces
msg_interfaces__msg__TargetPose *
msg_interfaces__msg__TargetPose__create();

/// Destroy msg/TargetPose message.
/**
 * It calls
 * msg_interfaces__msg__TargetPose__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_msg_interfaces
void
msg_interfaces__msg__TargetPose__destroy(msg_interfaces__msg__TargetPose * msg);

/// Check for msg/TargetPose message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_msg_interfaces
bool
msg_interfaces__msg__TargetPose__are_equal(const msg_interfaces__msg__TargetPose * lhs, const msg_interfaces__msg__TargetPose * rhs);

/// Copy a msg/TargetPose message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_msg_interfaces
bool
msg_interfaces__msg__TargetPose__copy(
  const msg_interfaces__msg__TargetPose * input,
  msg_interfaces__msg__TargetPose * output);

/// Initialize array of msg/TargetPose messages.
/**
 * It allocates the memory for the number of elements and calls
 * msg_interfaces__msg__TargetPose__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_msg_interfaces
bool
msg_interfaces__msg__TargetPose__Sequence__init(msg_interfaces__msg__TargetPose__Sequence * array, size_t size);

/// Finalize array of msg/TargetPose messages.
/**
 * It calls
 * msg_interfaces__msg__TargetPose__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_msg_interfaces
void
msg_interfaces__msg__TargetPose__Sequence__fini(msg_interfaces__msg__TargetPose__Sequence * array);

/// Create array of msg/TargetPose messages.
/**
 * It allocates the memory for the array and calls
 * msg_interfaces__msg__TargetPose__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_msg_interfaces
msg_interfaces__msg__TargetPose__Sequence *
msg_interfaces__msg__TargetPose__Sequence__create(size_t size);

/// Destroy array of msg/TargetPose messages.
/**
 * It calls
 * msg_interfaces__msg__TargetPose__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_msg_interfaces
void
msg_interfaces__msg__TargetPose__Sequence__destroy(msg_interfaces__msg__TargetPose__Sequence * array);

/// Check for msg/TargetPose message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_msg_interfaces
bool
msg_interfaces__msg__TargetPose__Sequence__are_equal(const msg_interfaces__msg__TargetPose__Sequence * lhs, const msg_interfaces__msg__TargetPose__Sequence * rhs);

/// Copy an array of msg/TargetPose messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_msg_interfaces
bool
msg_interfaces__msg__TargetPose__Sequence__copy(
  const msg_interfaces__msg__TargetPose__Sequence * input,
  msg_interfaces__msg__TargetPose__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // MSG_INTERFACES__MSG__DETAIL__TARGET_POSE__FUNCTIONS_H_
