# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_hamerschlag_haul_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED hamerschlag_haul_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(hamerschlag_haul_FOUND FALSE)
  elseif(NOT hamerschlag_haul_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(hamerschlag_haul_FOUND FALSE)
  endif()
  return()
endif()
set(_hamerschlag_haul_CONFIG_INCLUDED TRUE)

# output package information
if(NOT hamerschlag_haul_FIND_QUIETLY)
  message(STATUS "Found hamerschlag_haul:  (${hamerschlag_haul_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'hamerschlag_haul' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT hamerschlag_haul_DEPRECATED_QUIET)
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(hamerschlag_haul_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${hamerschlag_haul_DIR}/${_extra}")
endforeach()
