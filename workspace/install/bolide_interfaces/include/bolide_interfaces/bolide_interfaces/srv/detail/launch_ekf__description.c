// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from bolide_interfaces:srv/LaunchEkf.idl
// generated code does not contain a copyright notice

#include "bolide_interfaces/srv/detail/launch_ekf__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_bolide_interfaces
const rosidl_type_hash_t *
bolide_interfaces__srv__LaunchEkf__get_type_hash(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x6f, 0x49, 0xd3, 0x9a, 0x82, 0xd7, 0xa7, 0x69,
      0xc6, 0xb3, 0x3d, 0x29, 0x10, 0x8b, 0x58, 0x46,
      0x2e, 0x7c, 0x0a, 0xaf, 0xa9, 0x27, 0x9c, 0x89,
      0x09, 0xa7, 0xbf, 0x5c, 0x10, 0x93, 0xf2, 0xb4,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_bolide_interfaces
const rosidl_type_hash_t *
bolide_interfaces__srv__LaunchEkf_Request__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x9e, 0xd7, 0x05, 0x95, 0x0b, 0xcf, 0xb7, 0x25,
      0xf8, 0x6d, 0x82, 0x26, 0xab, 0x61, 0x18, 0x21,
      0xdc, 0xd3, 0xe9, 0xf4, 0x8f, 0xf7, 0xba, 0xc0,
      0x5c, 0x1e, 0x23, 0xde, 0xce, 0x67, 0x6e, 0x19,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_bolide_interfaces
const rosidl_type_hash_t *
bolide_interfaces__srv__LaunchEkf_Response__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xa9, 0x20, 0x3a, 0x4d, 0x7b, 0x96, 0x71, 0xa9,
      0x2f, 0x78, 0xc7, 0x9c, 0x9b, 0x37, 0x7b, 0xb3,
      0xd0, 0x42, 0x81, 0x74, 0x40, 0x94, 0x6c, 0x68,
      0x10, 0x48, 0x5a, 0x04, 0x11, 0xdc, 0x37, 0x67,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_bolide_interfaces
const rosidl_type_hash_t *
bolide_interfaces__srv__LaunchEkf_Event__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xb6, 0x80, 0x7b, 0xe6, 0x5f, 0x97, 0x9d, 0xb8,
      0x61, 0xf2, 0x69, 0xba, 0x46, 0x5b, 0x05, 0xef,
      0xe3, 0xb3, 0x45, 0xbf, 0x77, 0xfb, 0x6f, 0x3f,
      0x64, 0x16, 0xf7, 0xf4, 0x7f, 0x0b, 0xfb, 0xcd,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "service_msgs/msg/detail/service_event_info__functions.h"
#include "builtin_interfaces/msg/detail/time__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t builtin_interfaces__msg__Time__EXPECTED_HASH = {1, {
    0xb1, 0x06, 0x23, 0x5e, 0x25, 0xa4, 0xc5, 0xed,
    0x35, 0x09, 0x8a, 0xa0, 0xa6, 0x1a, 0x3e, 0xe9,
    0xc9, 0xb1, 0x8d, 0x19, 0x7f, 0x39, 0x8b, 0x0e,
    0x42, 0x06, 0xce, 0xa9, 0xac, 0xf9, 0xc1, 0x97,
  }};
static const rosidl_type_hash_t service_msgs__msg__ServiceEventInfo__EXPECTED_HASH = {1, {
    0x41, 0xbc, 0xbb, 0xe0, 0x7a, 0x75, 0xc9, 0xb5,
    0x2b, 0xc9, 0x6b, 0xfd, 0x5c, 0x24, 0xd7, 0xf0,
    0xfc, 0x0a, 0x08, 0xc0, 0xcb, 0x79, 0x21, 0xb3,
    0x37, 0x3c, 0x57, 0x32, 0x34, 0x5a, 0x6f, 0x45,
  }};
#endif

static char bolide_interfaces__srv__LaunchEkf__TYPE_NAME[] = "bolide_interfaces/srv/LaunchEkf";
static char bolide_interfaces__srv__LaunchEkf_Event__TYPE_NAME[] = "bolide_interfaces/srv/LaunchEkf_Event";
static char bolide_interfaces__srv__LaunchEkf_Request__TYPE_NAME[] = "bolide_interfaces/srv/LaunchEkf_Request";
static char bolide_interfaces__srv__LaunchEkf_Response__TYPE_NAME[] = "bolide_interfaces/srv/LaunchEkf_Response";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char service_msgs__msg__ServiceEventInfo__TYPE_NAME[] = "service_msgs/msg/ServiceEventInfo";

// Define type names, field names, and default values
static char bolide_interfaces__srv__LaunchEkf__FIELD_NAME__request_message[] = "request_message";
static char bolide_interfaces__srv__LaunchEkf__FIELD_NAME__response_message[] = "response_message";
static char bolide_interfaces__srv__LaunchEkf__FIELD_NAME__event_message[] = "event_message";

static rosidl_runtime_c__type_description__Field bolide_interfaces__srv__LaunchEkf__FIELDS[] = {
  {
    {bolide_interfaces__srv__LaunchEkf__FIELD_NAME__request_message, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {bolide_interfaces__srv__LaunchEkf_Request__TYPE_NAME, 39, 39},
    },
    {NULL, 0, 0},
  },
  {
    {bolide_interfaces__srv__LaunchEkf__FIELD_NAME__response_message, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {bolide_interfaces__srv__LaunchEkf_Response__TYPE_NAME, 40, 40},
    },
    {NULL, 0, 0},
  },
  {
    {bolide_interfaces__srv__LaunchEkf__FIELD_NAME__event_message, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {bolide_interfaces__srv__LaunchEkf_Event__TYPE_NAME, 37, 37},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription bolide_interfaces__srv__LaunchEkf__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {bolide_interfaces__srv__LaunchEkf_Event__TYPE_NAME, 37, 37},
    {NULL, 0, 0},
  },
  {
    {bolide_interfaces__srv__LaunchEkf_Request__TYPE_NAME, 39, 39},
    {NULL, 0, 0},
  },
  {
    {bolide_interfaces__srv__LaunchEkf_Response__TYPE_NAME, 40, 40},
    {NULL, 0, 0},
  },
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
bolide_interfaces__srv__LaunchEkf__get_type_description(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {bolide_interfaces__srv__LaunchEkf__TYPE_NAME, 31, 31},
      {bolide_interfaces__srv__LaunchEkf__FIELDS, 3, 3},
    },
    {bolide_interfaces__srv__LaunchEkf__REFERENCED_TYPE_DESCRIPTIONS, 5, 5},
  };
  if (!constructed) {
    description.referenced_type_descriptions.data[0].fields = bolide_interfaces__srv__LaunchEkf_Event__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[1].fields = bolide_interfaces__srv__LaunchEkf_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = bolide_interfaces__srv__LaunchEkf_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[3].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[4].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char bolide_interfaces__srv__LaunchEkf_Request__FIELD_NAME__start[] = "start";

static rosidl_runtime_c__type_description__Field bolide_interfaces__srv__LaunchEkf_Request__FIELDS[] = {
  {
    {bolide_interfaces__srv__LaunchEkf_Request__FIELD_NAME__start, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
bolide_interfaces__srv__LaunchEkf_Request__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {bolide_interfaces__srv__LaunchEkf_Request__TYPE_NAME, 39, 39},
      {bolide_interfaces__srv__LaunchEkf_Request__FIELDS, 1, 1},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char bolide_interfaces__srv__LaunchEkf_Response__FIELD_NAME__gotcha[] = "gotcha";

static rosidl_runtime_c__type_description__Field bolide_interfaces__srv__LaunchEkf_Response__FIELDS[] = {
  {
    {bolide_interfaces__srv__LaunchEkf_Response__FIELD_NAME__gotcha, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
bolide_interfaces__srv__LaunchEkf_Response__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {bolide_interfaces__srv__LaunchEkf_Response__TYPE_NAME, 40, 40},
      {bolide_interfaces__srv__LaunchEkf_Response__FIELDS, 1, 1},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char bolide_interfaces__srv__LaunchEkf_Event__FIELD_NAME__info[] = "info";
static char bolide_interfaces__srv__LaunchEkf_Event__FIELD_NAME__request[] = "request";
static char bolide_interfaces__srv__LaunchEkf_Event__FIELD_NAME__response[] = "response";

static rosidl_runtime_c__type_description__Field bolide_interfaces__srv__LaunchEkf_Event__FIELDS[] = {
  {
    {bolide_interfaces__srv__LaunchEkf_Event__FIELD_NAME__info, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    },
    {NULL, 0, 0},
  },
  {
    {bolide_interfaces__srv__LaunchEkf_Event__FIELD_NAME__request, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {bolide_interfaces__srv__LaunchEkf_Request__TYPE_NAME, 39, 39},
    },
    {NULL, 0, 0},
  },
  {
    {bolide_interfaces__srv__LaunchEkf_Event__FIELD_NAME__response, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {bolide_interfaces__srv__LaunchEkf_Response__TYPE_NAME, 40, 40},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription bolide_interfaces__srv__LaunchEkf_Event__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {bolide_interfaces__srv__LaunchEkf_Request__TYPE_NAME, 39, 39},
    {NULL, 0, 0},
  },
  {
    {bolide_interfaces__srv__LaunchEkf_Response__TYPE_NAME, 40, 40},
    {NULL, 0, 0},
  },
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
bolide_interfaces__srv__LaunchEkf_Event__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {bolide_interfaces__srv__LaunchEkf_Event__TYPE_NAME, 37, 37},
      {bolide_interfaces__srv__LaunchEkf_Event__FIELDS, 3, 3},
    },
    {bolide_interfaces__srv__LaunchEkf_Event__REFERENCED_TYPE_DESCRIPTIONS, 4, 4},
  };
  if (!constructed) {
    description.referenced_type_descriptions.data[0].fields = bolide_interfaces__srv__LaunchEkf_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[1].fields = bolide_interfaces__srv__LaunchEkf_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[2].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[3].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "bool start\n"
  "---\n"
  "bool gotcha";

static char srv_encoding[] = "srv";
static char implicit_encoding[] = "implicit";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
bolide_interfaces__srv__LaunchEkf__get_individual_type_description_source(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {bolide_interfaces__srv__LaunchEkf__TYPE_NAME, 31, 31},
    {srv_encoding, 3, 3},
    {toplevel_type_raw_source, 26, 26},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
bolide_interfaces__srv__LaunchEkf_Request__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {bolide_interfaces__srv__LaunchEkf_Request__TYPE_NAME, 39, 39},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
bolide_interfaces__srv__LaunchEkf_Response__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {bolide_interfaces__srv__LaunchEkf_Response__TYPE_NAME, 40, 40},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
bolide_interfaces__srv__LaunchEkf_Event__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {bolide_interfaces__srv__LaunchEkf_Event__TYPE_NAME, 37, 37},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
bolide_interfaces__srv__LaunchEkf__get_type_description_sources(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[6];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 6, 6};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *bolide_interfaces__srv__LaunchEkf__get_individual_type_description_source(NULL),
    sources[1] = *bolide_interfaces__srv__LaunchEkf_Event__get_individual_type_description_source(NULL);
    sources[2] = *bolide_interfaces__srv__LaunchEkf_Request__get_individual_type_description_source(NULL);
    sources[3] = *bolide_interfaces__srv__LaunchEkf_Response__get_individual_type_description_source(NULL);
    sources[4] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[5] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
bolide_interfaces__srv__LaunchEkf_Request__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *bolide_interfaces__srv__LaunchEkf_Request__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
bolide_interfaces__srv__LaunchEkf_Response__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *bolide_interfaces__srv__LaunchEkf_Response__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
bolide_interfaces__srv__LaunchEkf_Event__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[5];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 5, 5};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *bolide_interfaces__srv__LaunchEkf_Event__get_individual_type_description_source(NULL),
    sources[1] = *bolide_interfaces__srv__LaunchEkf_Request__get_individual_type_description_source(NULL);
    sources[2] = *bolide_interfaces__srv__LaunchEkf_Response__get_individual_type_description_source(NULL);
    sources[3] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[4] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
