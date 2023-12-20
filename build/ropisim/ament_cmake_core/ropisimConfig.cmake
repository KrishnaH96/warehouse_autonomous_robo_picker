# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_ropisim_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED ropisim_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(ropisim_FOUND FALSE)
  elseif(NOT ropisim_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(ropisim_FOUND FALSE)
  endif()
  return()
endif()
set(_ropisim_CONFIG_INCLUDED TRUE)

# output package information
if(NOT ropisim_FIND_QUIETLY)
  message(STATUS "Found ropisim: 0.0.0 (${ropisim_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'ropisim' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${ropisim_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(ropisim_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${ropisim_DIR}/${_extra}")
endforeach()
