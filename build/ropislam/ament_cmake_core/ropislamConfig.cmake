# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_ropislam_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED ropislam_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(ropislam_FOUND FALSE)
  elseif(NOT ropislam_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(ropislam_FOUND FALSE)
  endif()
  return()
endif()
set(_ropislam_CONFIG_INCLUDED TRUE)

# output package information
if(NOT ropislam_FIND_QUIETLY)
  message(STATUS "Found ropislam: 0.0.0 (${ropislam_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'ropislam' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${ropislam_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(ropislam_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${ropislam_DIR}/${_extra}")
endforeach()
