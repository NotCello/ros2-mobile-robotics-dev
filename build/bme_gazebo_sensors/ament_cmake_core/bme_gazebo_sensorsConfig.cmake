# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_bme_gazebo_sensors_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED bme_gazebo_sensors_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(bme_gazebo_sensors_FOUND FALSE)
  elseif(NOT bme_gazebo_sensors_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(bme_gazebo_sensors_FOUND FALSE)
  endif()
  return()
endif()
set(_bme_gazebo_sensors_CONFIG_INCLUDED TRUE)

# output package information
if(NOT bme_gazebo_sensors_FIND_QUIETLY)
  message(STATUS "Found bme_gazebo_sensors: 1.0.0 (${bme_gazebo_sensors_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'bme_gazebo_sensors' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT bme_gazebo_sensors_DEPRECATED_QUIET)
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(bme_gazebo_sensors_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${bme_gazebo_sensors_DIR}/${_extra}")
endforeach()
