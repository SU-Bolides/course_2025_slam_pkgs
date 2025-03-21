// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from bolide_interfaces:msg/MultipleRange.idl
// generated code does not contain a copyright notice

#include "bolide_interfaces/msg/detail/multiple_range__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_bolide_interfaces
const rosidl_type_hash_t *
bolide_interfaces__msg__MultipleRange__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x11, 0xa4, 0xc7, 0xa7, 0x78, 0x47, 0xc9, 0xa9,
      0x22, 0xc3, 0x0e, 0xcf, 0xdc, 0xa7, 0xc3, 0x24,
      0x80, 0x57, 0x10, 0xaa, 0x7c, 0xab, 0xab, 0x4b,
      0x57, 0x20, 0x13, 0xb3, 0xd0, 0x16, 0x90, 0x22,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "std_msgs/msg/detail/header__functions.h"
#include "builtin_interfaces/msg/detail/time__functions.h"
#include "sensor_msgs/msg/detail/range__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t builtin_interfaces__msg__Time__EXPECTED_HASH = {1, {
    0xb1, 0x06, 0x23, 0x5e, 0x25, 0xa4, 0xc5, 0xed,
    0x35, 0x09, 0x8a, 0xa0, 0xa6, 0x1a, 0x3e, 0xe9,
    0xc9, 0xb1, 0x8d, 0x19, 0x7f, 0x39, 0x8b, 0x0e,
    0x42, 0x06, 0xce, 0xa9, 0xac, 0xf9, 0xc1, 0x97,
  }};
static const rosidl_type_hash_t sensor_msgs__msg__Range__EXPECTED_HASH = {1, {
    0xb4, 0x2b, 0x62, 0x56, 0x2e, 0x93, 0xcb, 0xfe,
    0x9d, 0x42, 0xb8, 0x2f, 0xe5, 0x99, 0x4d, 0xfa,
    0x3d, 0x63, 0xd7, 0xd5, 0xc9, 0x0a, 0x31, 0x79,
    0x81, 0x70, 0x3f, 0x73, 0x88, 0xad, 0xff, 0x3a,
  }};
static const rosidl_type_hash_t std_msgs__msg__Header__EXPECTED_HASH = {1, {
    0xf4, 0x9f, 0xb3, 0xae, 0x2c, 0xf0, 0x70, 0xf7,
    0x93, 0x64, 0x5f, 0xf7, 0x49, 0x68, 0x3a, 0xc6,
    0xb0, 0x62, 0x03, 0xe4, 0x1c, 0x89, 0x1e, 0x17,
    0x70, 0x1b, 0x1c, 0xb5, 0x97, 0xce, 0x6a, 0x01,
  }};
#endif

static char bolide_interfaces__msg__MultipleRange__TYPE_NAME[] = "bolide_interfaces/msg/MultipleRange";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char sensor_msgs__msg__Range__TYPE_NAME[] = "sensor_msgs/msg/Range";
static char std_msgs__msg__Header__TYPE_NAME[] = "std_msgs/msg/Header";

// Define type names, field names, and default values
static char bolide_interfaces__msg__MultipleRange__FIELD_NAME__ir_rear_left[] = "ir_rear_left";
static char bolide_interfaces__msg__MultipleRange__FIELD_NAME__ir_rear_right[] = "ir_rear_right";
static char bolide_interfaces__msg__MultipleRange__FIELD_NAME__sonar_rear[] = "sonar_rear";

static rosidl_runtime_c__type_description__Field bolide_interfaces__msg__MultipleRange__FIELDS[] = {
  {
    {bolide_interfaces__msg__MultipleRange__FIELD_NAME__ir_rear_left, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {sensor_msgs__msg__Range__TYPE_NAME, 21, 21},
    },
    {NULL, 0, 0},
  },
  {
    {bolide_interfaces__msg__MultipleRange__FIELD_NAME__ir_rear_right, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {sensor_msgs__msg__Range__TYPE_NAME, 21, 21},
    },
    {NULL, 0, 0},
  },
  {
    {bolide_interfaces__msg__MultipleRange__FIELD_NAME__sonar_rear, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {sensor_msgs__msg__Range__TYPE_NAME, 21, 21},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription bolide_interfaces__msg__MultipleRange__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {sensor_msgs__msg__Range__TYPE_NAME, 21, 21},
    {NULL, 0, 0},
  },
  {
    {std_msgs__msg__Header__TYPE_NAME, 19, 19},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
bolide_interfaces__msg__MultipleRange__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {bolide_interfaces__msg__MultipleRange__TYPE_NAME, 35, 35},
      {bolide_interfaces__msg__MultipleRange__FIELDS, 3, 3},
    },
    {bolide_interfaces__msg__MultipleRange__REFERENCED_TYPE_DESCRIPTIONS, 3, 3},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&sensor_msgs__msg__Range__EXPECTED_HASH, sensor_msgs__msg__Range__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = sensor_msgs__msg__Range__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&std_msgs__msg__Header__EXPECTED_HASH, std_msgs__msg__Header__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[2].fields = std_msgs__msg__Header__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "sensor_msgs/Range ir_rear_left\n"
  "sensor_msgs/Range ir_rear_right\n"
  "sensor_msgs/Range sonar_rear";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
bolide_interfaces__msg__MultipleRange__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {bolide_interfaces__msg__MultipleRange__TYPE_NAME, 35, 35},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 91, 91},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
bolide_interfaces__msg__MultipleRange__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[4];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 4, 4};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *bolide_interfaces__msg__MultipleRange__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *sensor_msgs__msg__Range__get_individual_type_description_source(NULL);
    sources[3] = *std_msgs__msg__Header__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
