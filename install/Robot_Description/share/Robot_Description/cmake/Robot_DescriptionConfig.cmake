# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_Robot_Description_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED Robot_Description_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(Robot_Description_FOUND FALSE)
  elseif(NOT Robot_Description_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(Robot_Description_FOUND FALSE)
  endif()
  return()
endif()
set(_Robot_Description_CONFIG_INCLUDED TRUE)

# output package information
if(NOT Robot_Description_FIND_QUIETLY)
  message(STATUS "Found Robot_Description: 0.0.0 (${Robot_Description_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'Robot_Description' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${Robot_Description_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(Robot_Description_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${Robot_Description_DIR}/${_extra}")
endforeach()
