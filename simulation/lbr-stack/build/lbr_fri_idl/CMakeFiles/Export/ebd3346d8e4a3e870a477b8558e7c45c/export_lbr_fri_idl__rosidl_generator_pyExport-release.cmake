#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "lbr_fri_idl::lbr_fri_idl__rosidl_generator_py" for configuration "Release"
set_property(TARGET lbr_fri_idl::lbr_fri_idl__rosidl_generator_py APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(lbr_fri_idl::lbr_fri_idl__rosidl_generator_py PROPERTIES
  IMPORTED_LINK_DEPENDENT_LIBRARIES_RELEASE "lbr_fri_idl::lbr_fri_idl__rosidl_generator_c;Python3::Python;lbr_fri_idl::lbr_fri_idl__rosidl_typesupport_c;builtin_interfaces::builtin_interfaces__rosidl_generator_c;builtin_interfaces::builtin_interfaces__rosidl_typesupport_fastrtps_c;builtin_interfaces::builtin_interfaces__rosidl_typesupport_introspection_c;builtin_interfaces::builtin_interfaces__rosidl_typesupport_c;builtin_interfaces::builtin_interfaces__rosidl_typesupport_fastrtps_cpp;builtin_interfaces::builtin_interfaces__rosidl_typesupport_introspection_cpp;builtin_interfaces::builtin_interfaces__rosidl_typesupport_cpp;builtin_interfaces::builtin_interfaces__rosidl_generator_py"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/liblbr_fri_idl__rosidl_generator_py.so"
  IMPORTED_SONAME_RELEASE "liblbr_fri_idl__rosidl_generator_py.so"
  )

list(APPEND _cmake_import_check_targets lbr_fri_idl::lbr_fri_idl__rosidl_generator_py )
list(APPEND _cmake_import_check_files_for_lbr_fri_idl::lbr_fri_idl__rosidl_generator_py "${_IMPORT_PREFIX}/lib/liblbr_fri_idl__rosidl_generator_py.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
