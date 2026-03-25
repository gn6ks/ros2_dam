// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from lbr_fri_idl:msg/LBRTorqueCommand.idl
// generated code does not contain a copyright notice

#include "lbr_fri_idl/msg/detail/lbr_torque_command__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_lbr_fri_idl
const rosidl_type_hash_t *
lbr_fri_idl__msg__LBRTorqueCommand__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x7c, 0xc0, 0x62, 0xfe, 0x08, 0xee, 0xf6, 0x6b,
      0xd9, 0xbc, 0xc3, 0x85, 0xee, 0x6c, 0x30, 0x9d,
      0xfd, 0x71, 0x09, 0x29, 0x3b, 0x2f, 0x99, 0x5a,
      0x9c, 0xce, 0x9c, 0x66, 0x2d, 0xc5, 0x2b, 0x64,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char lbr_fri_idl__msg__LBRTorqueCommand__TYPE_NAME[] = "lbr_fri_idl/msg/LBRTorqueCommand";

// Define type names, field names, and default values
static char lbr_fri_idl__msg__LBRTorqueCommand__FIELD_NAME__joint_position[] = "joint_position";
static char lbr_fri_idl__msg__LBRTorqueCommand__FIELD_NAME__torque[] = "torque";

static rosidl_runtime_c__type_description__Field lbr_fri_idl__msg__LBRTorqueCommand__FIELDS[] = {
  {
    {lbr_fri_idl__msg__LBRTorqueCommand__FIELD_NAME__joint_position, 14, 14},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE_ARRAY,
      7,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {lbr_fri_idl__msg__LBRTorqueCommand__FIELD_NAME__torque, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE_ARRAY,
      7,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
lbr_fri_idl__msg__LBRTorqueCommand__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {lbr_fri_idl__msg__LBRTorqueCommand__TYPE_NAME, 32, 32},
      {lbr_fri_idl__msg__LBRTorqueCommand__FIELDS, 2, 2},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# Torque command for the Fast Robot Interface (FRI).\n"
  "# Requires joint position and torque overlay.\n"
  "# Refer to KUKA::FRI::LBRCommand in friLBRCommand.h for documentation.\n"
  "#\n"
  "#  * joint_position [rad]\n"
  "#  * torque [Nm]\n"
  "\n"
  "float64[7] joint_position\n"
  "float64[7] torque";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
lbr_fri_idl__msg__LBRTorqueCommand__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {lbr_fri_idl__msg__LBRTorqueCommand__TYPE_NAME, 32, 32},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 260, 260},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
lbr_fri_idl__msg__LBRTorqueCommand__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *lbr_fri_idl__msg__LBRTorqueCommand__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
