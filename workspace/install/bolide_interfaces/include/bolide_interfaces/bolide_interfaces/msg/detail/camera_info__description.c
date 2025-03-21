// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from bolide_interfaces:msg/CameraInfo.idl
// generated code does not contain a copyright notice

#include "bolide_interfaces/msg/detail/camera_info__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_bolide_interfaces
const rosidl_type_hash_t *
bolide_interfaces__msg__CameraInfo__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x85, 0xee, 0x70, 0x6b, 0x04, 0xf2, 0x8c, 0x16,
      0x22, 0xfa, 0xe1, 0x5a, 0xef, 0xf2, 0x51, 0x03,
      0x9a, 0xb5, 0xd5, 0x66, 0x32, 0xce, 0xfa, 0x91,
      0x7c, 0x50, 0xdf, 0xfb, 0xa9, 0xbb, 0x7a, 0x9f,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char bolide_interfaces__msg__CameraInfo__TYPE_NAME[] = "bolide_interfaces/msg/CameraInfo";

// Define type names, field names, and default values
static char bolide_interfaces__msg__CameraInfo__FIELD_NAME__wrong_way[] = "wrong_way";
static char bolide_interfaces__msg__CameraInfo__FIELD_NAME__front_color[] = "front_color";
static char bolide_interfaces__msg__CameraInfo__FIELD_NAME__left_color[] = "left_color";
static char bolide_interfaces__msg__CameraInfo__FIELD_NAME__right_color[] = "right_color";

static rosidl_runtime_c__type_description__Field bolide_interfaces__msg__CameraInfo__FIELDS[] = {
  {
    {bolide_interfaces__msg__CameraInfo__FIELD_NAME__wrong_way, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {bolide_interfaces__msg__CameraInfo__FIELD_NAME__front_color, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {bolide_interfaces__msg__CameraInfo__FIELD_NAME__left_color, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {bolide_interfaces__msg__CameraInfo__FIELD_NAME__right_color, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
bolide_interfaces__msg__CameraInfo__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {bolide_interfaces__msg__CameraInfo__TYPE_NAME, 32, 32},
      {bolide_interfaces__msg__CameraInfo__FIELDS, 4, 4},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "bool wrong_way\n"
  "string front_color\n"
  "string left_color\n"
  "string right_color";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
bolide_interfaces__msg__CameraInfo__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {bolide_interfaces__msg__CameraInfo__TYPE_NAME, 32, 32},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 70, 70},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
bolide_interfaces__msg__CameraInfo__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *bolide_interfaces__msg__CameraInfo__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
