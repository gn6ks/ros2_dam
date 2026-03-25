// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from lbr_fri_idl:msg/LBRCommand.idl
// generated code does not contain a copyright notice

#include "lbr_fri_idl/msg/detail/lbr_command__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_lbr_fri_idl
const rosidl_type_hash_t *
lbr_fri_idl__msg__LBRCommand__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x2b, 0xf0, 0x4f, 0x40, 0x08, 0x37, 0x3e, 0x5d,
      0xa8, 0xa2, 0x6d, 0x45, 0xe9, 0x42, 0x4f, 0xa5,
      0xd1, 0x68, 0x7b, 0x1e, 0x6f, 0x3f, 0xe6, 0x99,
      0xed, 0x54, 0x56, 0x20, 0x1c, 0x0b, 0x0f, 0x61,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char lbr_fri_idl__msg__LBRCommand__TYPE_NAME[] = "lbr_fri_idl/msg/LBRCommand";

// Define type names, field names, and default values
static char lbr_fri_idl__msg__LBRCommand__FIELD_NAME__joint_position[] = "joint_position";
static char lbr_fri_idl__msg__LBRCommand__FIELD_NAME__torque[] = "torque";
static char lbr_fri_idl__msg__LBRCommand__FIELD_NAME__wrench[] = "wrench";

static rosidl_runtime_c__type_description__Field lbr_fri_idl__msg__LBRCommand__FIELDS[] = {
  {
    {lbr_fri_idl__msg__LBRCommand__FIELD_NAME__joint_position, 14, 14},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE_ARRAY,
      7,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {lbr_fri_idl__msg__LBRCommand__FIELD_NAME__torque, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE_ARRAY,
      7,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {lbr_fri_idl__msg__LBRCommand__FIELD_NAME__wrench, 6, 6},
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
lbr_fri_idl__msg__LBRCommand__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {lbr_fri_idl__msg__LBRCommand__TYPE_NAME, 26, 26},
      {lbr_fri_idl__msg__LBRCommand__FIELDS, 3, 3},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# Message that holds commands that are exposed through the\n"
  "# Fast Robot Interface (FRI).\n"
  "# Following commands are exposed (refer to KUKA::FRI::LBRCommand in friLBRCommand.h for documentation):\n"
  "#\n"
  "#  * joint_position [rad]\n"
  "#  * torque [Nm]\n"
  "#  * wrench [N/Nm]\n"
  "\n"
  "float64[7] joint_position\n"
  "float64[7] torque\n"
  "float64[6] wrench";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
lbr_fri_idl__msg__LBRCommand__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {lbr_fri_idl__msg__LBRCommand__TYPE_NAME, 26, 26},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 320, 320},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
lbr_fri_idl__msg__LBRCommand__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *lbr_fri_idl__msg__LBRCommand__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
