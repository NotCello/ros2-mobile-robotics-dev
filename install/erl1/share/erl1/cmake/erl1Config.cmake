# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_erl1_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED erl1_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(erl1_FOUND FALSE)
  elseif(NOT erl1_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(erl1_FOUND FALSE)
  endif()
  return()
endif()
set(_erl1_CONFIG_INCLUDED TRUE)

# output package information
if(NOT erl1_FIND_QUIETLY)
  message(STATUS "Found erl1: 0.0.0 (${erl1_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'erl1' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT erl1_DEPRECATED_QUIET)
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(erl1_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${erl1_DIR}/${_extra}")
endforeach()
