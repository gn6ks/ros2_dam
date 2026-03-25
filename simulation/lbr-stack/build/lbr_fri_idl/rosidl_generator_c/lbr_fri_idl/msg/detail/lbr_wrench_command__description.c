// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from lbr_fri_idl:msg/LBRWrenchCommand.idl
// generated code does not contain a copyright notice

#include "lbr_fri_idl/msg/detail/lbr_wrench_command__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_lbr_fri_idl
const rosidl_type_hash_t *
lbr_fri_idl__msg__LBRWrenchCommand__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x66, 0xa2, 0xc7, 0xac, 0x87, 0xe7, 0x16, 0x02,
      0x6f, 0x26, 0x7c, 0xd2, 0xef, 0x42, 0xec, 0x2c,
      0x47, 0x99, 0x56, 0x3d, 0xf0, 0xd0, 0x24, 0x01,
      0x88, 0x85, 0x59, 0x3b, 0x7f, 0xd8, 0x75, 0x37,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char lbr_fri_idl__msg__LBRWrenchCommand__TYPE_NAME[] = "lbr_fri_idl/msg/LBRWrenchCommand";

// Define type names, field names, and default values
static char lbr_fri_idl__msg__LBRWrenchCommand__FIELD_NAME__joint_position[] = "joint_position";
static char lbr_fri_idl__msg__LBRWrenchCommand__FIELD_NAME__wrench[] = "wrench";

static rosidl_runtime_c__type_description__Field lbr_fri_idl__msg__LBRWrenchCommand__FIELDS[] = {
  {
    {lbr_fri_idl__msg__LBRWrenchCommand__FIELD_NAME__joint_position, 14, 14},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE_ARRAY,
      7,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {lbr_fri_idl__msg__LBRWrenchCommand__FIELD_NAME__wrench, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE_ARRAY,
      6,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
lbr_fri_idl__msg__LBRWrenchCommand__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {lbr_fri_idl__msg__LBRWrenchCommand__TYPE_NAME, 32, 32},
      {lbr_fri_idl__msg__LBRWrenchCommand__FIELDS, 2, 2},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# Wrench command for the Fast Robot Interface (FRI).\n"
  "# Requires joint position and wrench overlay.\n"
  "# Refer to KUKA::FRI::LBRCommand in friLBRCommand.h for documentation.\n"
  "#\n"
  "#  * joint_position [rad]\n"
  "#  * wrench [N/Nm]\n"
  "\n"
  "float64[7] joint_position\n"
  "float64[6] wrench";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
lbr_fri_idl__msg__LBRWrenchCommand__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {lbr_fri_idl__msg__LBRWrenchCommand__TYPE_NAME, 32, 32},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 262, 262},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
lbr_fri_idl__msg__LBRWrenchCommand__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *lbr_fri_idl__msg__LBRWrenchCommand__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
