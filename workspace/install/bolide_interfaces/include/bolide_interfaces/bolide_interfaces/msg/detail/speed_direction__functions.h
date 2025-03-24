// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from bolide_interfaces:msg/SpeedDirection.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "bolide_interfaces/msg/speed_direction.h"


#ifndef BOLIDE_INTERFACES__MSG__DETAIL__SPEED_DIRECTION__FUNCTIONS_H_
#define BOLIDE_INTERFACES__MSG__DETAIL__SPEED_DIRECTION__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/action_type_support_struct.h"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_runtime_c/service_type_support_struct.h"
#include "rosidl_runtime_c/type_description/type_description__struct.h"
#include "rosidl_runtime_c/type_description/type_source__struct.h"
#include "rosidl_runtime_c/type_hash.h"
#include "rosidl_runtime_c/visibility_control.h"
#include "bolide_interfaces/msg/rosidl_generator_c__visibility_control.h"

#include "bolide_interfaces/msg/detail/speed_direction__struct.h"

/// Initialize msg/SpeedDirection message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * bolide_interfaces__msg__SpeedDirection
 * )) before or use
 * bolide_interfaces__msg__SpeedDirection__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_bolide_interfaces
bool
bolide_interfaces__msg__SpeedDirection__init(bolide_interfaces__msg__SpeedDirection * msg);

/// Finalize msg/SpeedDirection message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_bolide_interfaces
void
bolide_interfaces__msg__SpeedDirection__fini(bolide_interfaces__msg__SpeedDirection * msg);

/// Create msg/SpeedDirection message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * bolide_interfaces__msg__SpeedDirection__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_bolide_interfaces
bolide_interfaces__msg__SpeedDirection *
bolide_interfaces__msg__SpeedDirection__create(void);

/// Destroy msg/SpeedDirection message.
/**
 * It calls
 * bolide_interfaces__msg__SpeedDirection__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_bolide_interfaces
void
bolide_interfaces__msg__SpeedDirection__destroy(bolide_interfaces__msg__SpeedDirection * msg);

/// Check for msg/SpeedDirection message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_bolide_interfaces
bool
bolide_interfaces__msg__SpeedDirection__are_equal(const bolide_interfaces__msg__SpeedDirection * lhs, const bolide_interfaces__msg__SpeedDirection * rhs);

/// Copy a msg/SpeedDirection message.
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
ROSIDL_GENERATOR_C_PUBLIC_bolide_interfaces
bool
bolide_interfaces__msg__SpeedDirection__copy(
  const bolide_interfaces__msg__SpeedDirection * input,
  bolide_interfaces__msg__SpeedDirection * output);

/// Retrieve pointer to the hash of the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_bolide_interfaces
const rosidl_type_hash_t *
bolide_interfaces__msg__SpeedDirection__get_type_hash(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_bolide_interfaces
const rosidl_runtime_c__type_description__TypeDescription *
bolide_interfaces__msg__SpeedDirection__get_type_description(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the single raw source text that defined this type.
ROSIDL_GENERATOR_C_PUBLIC_bolide_interfaces
const rosidl_runtime_c__type_description__TypeSource *
bolide_interfaces__msg__SpeedDirection__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the recursive raw sources that defined the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_bolide_interfaces
const rosidl_runtime_c__type_description__TypeSource__Sequence *
bolide_interfaces__msg__SpeedDirection__get_type_description_sources(
  const rosidl_message_type_support_t * type_support);

/// Initialize array of msg/SpeedDirection messages.
/**
 * It allocates the memory for the number of elements and calls
 * bolide_interfaces__msg__SpeedDirection__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_bolide_interfaces
bool
bolide_interfaces__msg__SpeedDirection__Sequence__init(bolide_interfaces__msg__SpeedDirection__Sequence * array, size_t size);

/// Finalize array of msg/SpeedDirection messages.
/**
 * It calls
 * bolide_interfaces__msg__SpeedDirection__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_bolide_interfaces
void
bolide_interfaces__msg__SpeedDirection__Sequence__fini(bolide_interfaces__msg__SpeedDirection__Sequence * array);

/// Create array of msg/SpeedDirection messages.
/**
 * It allocates the memory for the array and calls
 * bolide_interfaces__msg__SpeedDirection__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_bolide_interfaces
bolide_interfaces__msg__SpeedDirection__Sequence *
bolide_interfaces__msg__SpeedDirection__Sequence__create(size_t size);

/// Destroy array of msg/SpeedDirection messages.
/**
 * It calls
 * bolide_interfaces__msg__SpeedDirection__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_bolide_interfaces
void
bolide_interfaces__msg__SpeedDirection__Sequence__destroy(bolide_interfaces__msg__SpeedDirection__Sequence * array);

/// Check for msg/SpeedDirection message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_bolide_interfaces
bool
bolide_interfaces__msg__SpeedDirection__Sequence__are_equal(const bolide_interfaces__msg__SpeedDirection__Sequence * lhs, const bolide_interfaces__msg__SpeedDirection__Sequence * rhs);

/// Copy an array of msg/SpeedDirection messages.
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
ROSIDL_GENERATOR_C_PUBLIC_bolide_interfaces
bool
bolide_interfaces__msg__SpeedDirection__Sequence__copy(
  const bolide_interfaces__msg__SpeedDirection__Sequence * input,
  bolide_interfaces__msg__SpeedDirection__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // BOLIDE_INTERFACES__MSG__DETAIL__SPEED_DIRECTION__FUNCTIONS_H_
