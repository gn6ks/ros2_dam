// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from lbr_fri_idl:msg/LBRState.idl
// generated code does not contain a copyright notice

#include "lbr_fri_idl/msg/detail/lbr_state__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_lbr_fri_idl
const rosidl_type_hash_t *
lbr_fri_idl__msg__LBRState__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x0b, 0xd5, 0x06, 0xfb, 0x2d, 0xee, 0xcf, 0x20,
      0x21, 0x0a, 0x7b, 0xcd, 0x33, 0x36, 0x73, 0x7c,
      0xf6, 0x56, 0x5e, 0x14, 0xf8, 0x63, 0x77, 0x51,
      0x08, 0x2e, 0x54, 0x06, 0x7c, 0x9a, 0xe9, 0x9d,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char lbr_fri_idl__msg__LBRState__TYPE_NAME[] = "lbr_fri_idl/msg/LBRState";

// Define type names, field names, and default values
static char lbr_fri_idl__msg__LBRState__FIELD_NAME__sample_time[] = "sample_time";
static char lbr_fri_idl__msg__LBRState__FIELD_NAME__session_state[] = "session_state";
static char lbr_fri_idl__msg__LBRState__FIELD_NAME__connection_quality[] = "connection_quality";
static char lbr_fri_idl__msg__LBRState__FIELD_NAME__safety_state[] = "safety_state";
static char lbr_fri_idl__msg__LBRState__FIELD_NAME__operation_mode[] = "operation_mode";
static char lbr_fri_idl__msg__LBRState__FIELD_NAME__drive_state[] = "drive_state";
static char lbr_fri_idl__msg__LBRState__FIELD_NAME__client_command_mode[] = "client_command_mode";
static char lbr_fri_idl__msg__LBRState__FIELD_NAME__overlay_type[] = "overlay_type";
static char lbr_fri_idl__msg__LBRState__FIELD_NAME__control_mode[] = "control_mode";
static char lbr_fri_idl__msg__LBRState__FIELD_NAME__time_stamp_sec[] = "time_stamp_sec";
static char lbr_fri_idl__msg__LBRState__FIELD_NAME__time_stamp_nano_sec[] = "time_stamp_nano_sec";
static char lbr_fri_idl__msg__LBRState__FIELD_NAME__measured_joint_position[] = "measured_joint_position";
static char lbr_fri_idl__msg__LBRState__FIELD_NAME__commanded_joint_position[] = "commanded_joint_position";
static char lbr_fri_idl__msg__LBRState__FIELD_NAME__measured_torque[] = "measured_torque";
static char lbr_fri_idl__msg__LBRState__FIELD_NAME__commanded_torque[] = "commanded_torque";
static char lbr_fri_idl__msg__LBRState__FIELD_NAME__external_torque[] = "external_torque";
static char lbr_fri_idl__msg__LBRState__FIELD_NAME__ipo_joint_position[] = "ipo_joint_position";
static char lbr_fri_idl__msg__LBRState__FIELD_NAME__tracking_performance[] = "tracking_performance";

static rosidl_runtime_c__type_description__Field lbr_fri_idl__msg__LBRState__FIELDS[] = {
  {
    {lbr_fri_idl__msg__LBRState__FIELD_NAME__sample_time, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {lbr_fri_idl__msg__LBRState__FIELD_NAME__session_state, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {lbr_fri_idl__msg__LBRState__FIELD_NAME__connection_quality, 18, 18},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {lbr_fri_idl__msg__LBRState__FIELD_NAME__safety_state, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {lbr_fri_idl__msg__LBRState__FIELD_NAME__operation_mode, 14, 14},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {lbr_fri_idl__msg__LBRState__FIELD_NAME__drive_state, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {lbr_fri_idl__msg__LBRState__FIELD_NAME__client_command_mode, 19, 19},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {lbr_fri_idl__msg__LBRState__FIELD_NAME__overlay_type, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {lbr_fri_idl__msg__LBRState__FIELD_NAME__control_mode, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {lbr_fri_idl__msg__LBRState__FIELD_NAME__time_stamp_sec, 14, 14},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {lbr_fri_idl__msg__LBRState__FIELD_NAME__time_stamp_nano_sec, 19, 19},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {lbr_fri_idl__msg__LBRState__FIELD_NAME__measured_joint_position, 23, 23},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE_ARRAY,
      7,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {lbr_fri_idl__msg__LBRState__FIELD_NAME__commanded_joint_position, 24, 24},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE_ARRAY,
      7,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {lbr_fri_idl__msg__LBRState__FIELD_NAME__measured_torque, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE_ARRAY,
      7,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {lbr_fri_idl__msg__LBRState__FIELD_NAME__commanded_torque, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE_ARRAY,
      7,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {lbr_fri_idl__msg__LBRState__FIELD_NAME__external_torque, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE_ARRAY,
      7,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {lbr_fri_idl__msg__LBRState__FIELD_NAME__ipo_joint_position, 18, 18},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE_ARRAY,
      7,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {lbr_fri_idl__msg__LBRState__FIELD_NAME__tracking_performance, 20, 20},
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
lbr_fri_idl__msg__LBRState__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {lbr_fri_idl__msg__LBRState__TYPE_NAME, 24, 24},
      {lbr_fri_idl__msg__LBRState__FIELDS, 18, 18},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# Message that holds states that are exposed through the\n"
  "# Fast Robot Interface (FRI).\n"
  "# Following states are exposed (refer to KUKA::FRI::LBRState in friLBRState.h for documentation):\n"
  "#\n"
  "#  * sample_time [s]\n"
  "#  * session_state [#]\n"
  "#  * connection_quality [#]\n"
  "#  * safety_state [#]\n"
  "#  * operation_mode [#]\n"
  "#  * drive_state [#]: \n"
  "#  * client_command_mode [#]\n"
  "#  * overlay_type [#]\n"
  "#  * control_mode [#]\n"
  "#\n"
  "#  * time_stamp_sec [s]\n"
  "#  * time_stamp_nano_sec [ns]\n"
  "#\n"
  "#  * measured_joint_position [rad]\n"
  "#  * commanded_joint_position [rad]\n"
  "#  * measured_torque [Nm]\n"
  "#  * commanded_torque [Nm]\n"
  "#  * external_torque [Nm]\n"
  "#  * ipo_joint_position [rad]\n"
  "#  * tracking_performance [a.u.]\n"
  "\n"
  "float64 sample_time\n"
  "int8 session_state\n"
  "int8 connection_quality\n"
  "int8 safety_state\n"
  "int8 operation_mode\n"
  "int8 drive_state\n"
  "int8 client_command_mode\n"
  "int8 overlay_type\n"
  "int8 control_mode\n"
  "\n"
  "uint32 time_stamp_sec\n"
  "uint32 time_stamp_nano_sec\n"
  "\n"
  "float64[7] measured_joint_position\n"
  "float64[7] commanded_joint_position\n"
  "float64[7] measured_torque\n"
  "float64[7] commanded_torque\n"
  "float64[7] external_torque\n"
  "float64[7] ipo_joint_position\n"
  "float64 tracking_performance";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
lbr_fri_idl__msg__LBRState__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {lbr_fri_idl__msg__LBRState__TYPE_NAME, 24, 24},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 1115, 1115},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
lbr_fri_idl__msg__LBRState__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *lbr_fri_idl__msg__LBRState__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
