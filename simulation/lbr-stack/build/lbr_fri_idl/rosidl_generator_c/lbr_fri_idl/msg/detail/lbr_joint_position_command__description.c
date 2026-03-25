// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from lbr_fri_idl:msg/LBRJointPositionCommand.idl
// generated code does not contain a copyright notice

#include "lbr_fri_idl/msg/detail/lbr_joint_position_command__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_lbr_fri_idl
const rosidl_type_hash_t *
lbr_fri_idl__msg__LBRJointPositionCommand__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xe2, 0x91, 0xf8, 0xc5, 0x8a, 0x82, 0x14, 0xc8,
      0x0a, 0xe5, 0x76, 0x03, 0x97, 0xb4, 0xe8, 0xd0,
      0xd3, 0xc0, 0x44, 0x1f, 0xac, 0x36, 0xab, 0xf9,
      0xac, 0x4a, 0x7d, 0x34, 0xd2, 0x15, 0x46, 0x2a,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char lbr_fri_idl__msg__LBRJointPositionCommand__TYPE_NAME[] = "lbr_fri_idl/msg/LBRJointPositionCommand";

// Define type names, field names, and default values
static char lbr_fri_idl__msg__LBRJointPositionCommand__FIELD_NAME__joint_position[] = "joint_position";

static rosidl_runtime_c__type_description__Field lbr_fri_idl__msg__LBRJointPositionCommand__FIELDS[] = {
  {
    {lbr_fri_idl__msg__LBRJointPositionCommand__FIELD_NAME__joint_position, 14, 14},
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
lbr_fri_idl__msg__LBRJointPositionCommand__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {lbr_fri_idl__msg__LBRJointPositionCommand__TYPE_NAME, 39, 39},
      {lbr_fri_idl__msg__LBRJointPositionCommand__FIELDS, 1, 1},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# Position command for the Fast Robot Interface (FRI).\n"
  "# Refer to KUKA::FRI::LBRCommand in friLBRCommand.h for documentation.\n"
  "#\n"
  "#  * joint_position [rad]\n"
  "\n"
  "float64[7] joint_position";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
lbr_fri_idl__msg__LBRJointPositionCommand__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {lbr_fri_idl__msg__LBRJointPositionCommand__TYPE_NAME, 39, 39},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 181, 181},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
lbr_fri_idl__msg__LBRJointPositionCommand__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *lbr_fri_idl__msg__LBRJointPositionCommand__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
