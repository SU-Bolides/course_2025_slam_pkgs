// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from bolide_interfaces:msg/MultipleRange.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "bolide_interfaces/msg/multiple_range.h"


#ifndef BOLIDE_INTERFACES__MSG__DETAIL__MULTIPLE_RANGE__FUNCTIONS_H_
#define BOLIDE_INTERFACES__MSG__DETAIL__MULTIPLE_RANGE__FUNCTIONS_H_

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

#include "bolide_interfaces/msg/detail/multiple_range__struct.h"

/// Initialize msg/MultipleRange message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * bolide_interfaces__msg__MultipleRange
 * )) before or use
 * bolide_interfaces__msg__MultipleRange__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_bolide_interfaces
bool
bolide_interfaces__msg__MultipleRange__init(bolide_interfaces__msg__MultipleRange * msg);

/// Finalize msg/MultipleRange message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_bolide_interfaces
void
bolide_interfaces__msg__MultipleRange__fini(bolide_interfaces__msg__MultipleRange * msg);

/// Create msg/MultipleRange message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * bolide_interfaces__msg__MultipleRange__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_bolide_interfaces
bolide_interfaces__msg__MultipleRange *
bolide_interfaces__msg__MultipleRange__create(void);

/// Destroy msg/MultipleRange message.
/**
 * It calls
 * bolide_interfaces__msg__MultipleRange__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_bolide_interfaces
void
bolide_interfaces__msg__MultipleRange__destroy(bolide_interfaces__msg__MultipleRange * msg);

/// Check for msg/MultipleRange message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_bolide_interfaces
bool
bolide_interfaces__msg__MultipleRange__are_equal(const bolide_interfaces__msg__MultipleRange * lhs, const bolide_interfaces__msg__MultipleRange * rhs);

/// Copy a msg/MultipleRange message.
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
bolide_interfaces__msg__MultipleRange__copy(
  const bolide_interfaces__msg__MultipleRange * input,
  bolide_interfaces__msg__MultipleRange * output);

/// Retrieve pointer to the hash of the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_bolide_interfaces
const rosidl_type_hash_t *
bolide_interfaces__msg__MultipleRange__get_type_hash(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_bolide_interfaces
const rosidl_runtime_c__type_description__TypeDescription *
bolide_interfaces__msg__MultipleRange__get_type_description(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the single raw source text that defined this type.
ROSIDL_GENERATOR_C_PUBLIC_bolide_interfaces
const rosidl_runtime_c__type_description__TypeSource *
bolide_interfaces__msg__MultipleRange__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the recursive raw sources that defined the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_bolide_interfaces
const rosidl_runtime_c__type_description__TypeSource__Sequence *
bolide_interfaces__msg__MultipleRange__get_type_description_sources(
  const rosidl_message_type_support_t * type_support);

/// Initialize array of msg/MultipleRange messages.
/**
 * It allocates the memory for the number of elements and calls
 * bolide_interfaces__msg__MultipleRange__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_bolide_interfaces
bool
bolide_interfaces__msg__MultipleRange__Sequence__init(bolide_interfaces__msg__MultipleRange__Sequence * array, size_t size);

/// Finalize array of msg/MultipleRange messages.
/**
 * It calls
 * bolide_interfaces__msg__MultipleRange__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_bolide_interfaces
void
bolide_interfaces__msg__MultipleRange__Sequence__fini(bolide_interfaces__msg__MultipleRange__Sequence * array);

/// Create array of msg/MultipleRange messages.
/**
 * It allocates the memory for the array and calls
 * bolide_interfaces__msg__MultipleRange__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_bolide_interfaces
bolide_interfaces__msg__MultipleRange__Sequence *
bolide_interfaces__msg__MultipleRange__Sequence__create(size_t size);

/// Destroy array of msg/MultipleRange messages.
/**
 * It calls
 * bolide_interfaces__msg__MultipleRange__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_bolide_interfaces
void
bolide_interfaces__msg__MultipleRange__Sequence__destroy(bolide_interfaces__msg__MultipleRange__Sequence * array);

/// Check for msg/MultipleRange message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_bolide_interfaces
bool
bolide_interfaces__msg__MultipleRange__Sequence__are_equal(const bolide_interfaces__msg__MultipleRange__Sequence * lhs, const bolide_interfaces__msg__MultipleRange__Sequence * rhs);

/// Copy an array of msg/MultipleRange messages.
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
bolide_interfaces__msg__MultipleRange__Sequence__copy(
  const bolide_interfaces__msg__MultipleRange__Sequence * input,
  bolide_interfaces__msg__MultipleRange__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // BOLIDE_INTERFACES__MSG__DETAIL__MULTIPLE_RANGE__FUNCTIONS_H_
