#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "libkernel" for configuration ""
set_property(TARGET libkernel APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(libkernel PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_NOCONFIG "ASM"
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/kernel/liblibkernel.a"
  )

list(APPEND _IMPORT_CHECK_TARGETS libkernel )
list(APPEND _IMPORT_CHECK_FILES_FOR_libkernel "${_IMPORT_PREFIX}/kernel/liblibkernel.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
