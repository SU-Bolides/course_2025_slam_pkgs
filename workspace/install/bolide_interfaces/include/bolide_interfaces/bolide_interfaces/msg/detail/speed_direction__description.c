// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from bolide_interfaces:msg/SpeedDirection.idl
// generated code does not contain a copyright notice

#include "bolide_interfaces/msg/detail/speed_direction__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_bolide_interfaces
const rosidl_type_hash_t *
bolide_interfaces__msg__SpeedDirection__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xe8, 0xde, 0x3c, 0x45, 0x34, 0x82, 0x8f, 0x89,
      0x6e, 0xfb, 0xbe, 0x1b, 0x2c, 0xae, 0xcf, 0xf2,
      0xf2, 0x2b, 0xc9, 0x0a, 0x88, 0xc4, 0x38, 0x47,
      0x30, 0xe6, 0x5f, 0xf5, 0x89, 0xe6, 0x3f, 0x32,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char bolide_interfaces__msg__SpeedDirection__TYPE_NAME[] = "bolide_interfaces/msg/SpeedDirection";

// Define type names, field names, and default values
static char bolide_interfaces__msg__SpeedDirection__FIELD_NAME__speed[] = "speed";
static char bolide_interfaces__msg__SpeedDirection__FIELD_NAME__direction[] = "direction";

static rosidl_runtime_c__type_description__Field bolide_interfaces__msg__SpeedDirection__FIELDS[] = {
  {
    {bolide_interfaces__msg__SpeedDirection__FIELD_NAME__speed, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {bolide_interfaces__msg__SpeedDirection__FIELD_NAME__direction, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
bolide_interfaces__msg__SpeedDirection__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {bolide_interfaces__msg__SpeedDirection__TYPE_NAME, 36, 36},
      {bolide_interfaces__msg__SpeedDirection__FIELDS, 2, 2},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "float64 speed\n"
  "float64 direction";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
bolide_interfaces__msg__SpeedDirection__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {bolide_interfaces__msg__SpeedDirection__TYPE_NAME, 36, 36},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 31, 31},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
bolide_interfaces__msg__SpeedDirection__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *bolide_interfaces__msg__SpeedDirection__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
