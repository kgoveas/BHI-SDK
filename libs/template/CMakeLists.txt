#--comment: use *.c source files to generate library
#get_filename_component( proj ${CMAKE_CURRENT_LIST_DIR} NAME)

#project(${proj} C)

#set(SOURCES
#   lib_template.c
#)
#include_directories(
#     ../../libs/template/includes/
#)

#ADD_ARC_LIBRARY(${proj} ${SOURCES})

#EXPORT_ARC_LIBRARY(${proj})

#--comment: use *.a to integrate
file(GLOB CONFIG_FILES "${CMAKE_CURRENT_LIST_DIR}/*.sdk.cmake")
foreach(f ${CONFIG_FILES})
  include(${f})
endforeach()

