# - Try to find LibYAML
# Once done this will define
#
#  LIBYAML_FOUND - system has LibYAML
#  LIBYAML_INCLUDE_DIRS - the LibYAML include directory
#  LIBYAML_LIBRARIES - Link these to use LibYAML
#  LIBYAML_DEFINITIONS - Compiler switches required for using LibYAML
#
#  Copyright (c) 2009  <>
#
#  Redistribution and use is allowed according to the terms of the New
#  BSD license.
#  For details see the accompanying COPYING-CMAKE-SCRIPTS file.
#


if (LIBYAML_LIBRARIES AND LIBYAML_INCLUDE_DIRS)
  # in cache already
  set(LIBYAML_FOUND TRUE)
else (LIBYAML_LIBRARIES AND LIBYAML_INCLUDE_DIRS)
  find_path(LIBYAML_INCLUDE_DIR
    NAMES
      yaml.h
    PATHS
      /usr/include
      /usr/local/include
      /opt/local/include
      /sw/include
  )

  find_library(YAML_LIBRARY
    NAMES
      yaml
    PATHS
      /usr/lib
      /usr/local/lib
      /opt/local/lib
      /sw/lib
  )

  set(LIBYAML_INCLUDE_DIRS
    ${LIBYAML_INCLUDE_DIR}
  )
  set(LIBYAML_LIBRARIES
    ${YAML_LIBRARY}
)

  if (LIBYAML_INCLUDE_DIRS AND LIBYAML_LIBRARIES)
     set(LIBYAML_FOUND TRUE)
  endif (LIBYAML_INCLUDE_DIRS AND LIBYAML_LIBRARIES)

  if (LIBYAML_FOUND)
    if (NOT LibYAML_FIND_QUIETLY)
      message(STATUS "Found LibYAML: ${LIBYAML_LIBRARIES}")
    endif (NOT LibYAML_FIND_QUIETLY)
  else (LIBYAML_FOUND)
    if (LibYAML_FIND_REQUIRED)
      message(FATAL_ERROR "Could not find LibYAML")
    endif (LibYAML_FIND_REQUIRED)
  endif (LIBYAML_FOUND)

  # show the LIBYAML_INCLUDE_DIRS and LIBYAML_LIBRARIES variables only in the advanced view
  mark_as_advanced(LIBYAML_INCLUDE_DIRS LIBYAML_LIBRARIES)

endif (LIBYAML_LIBRARIES AND LIBYAML_INCLUDE_DIRS)

