# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_fullscan_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED fullscan_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(fullscan_FOUND FALSE)
  elseif(NOT fullscan_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(fullscan_FOUND FALSE)
  endif()
  return()
endif()
set(_fullscan_CONFIG_INCLUDED TRUE)

# output package information
if(NOT fullscan_FIND_QUIETLY)
  message(STATUS "Found fullscan: 0.0.0 (${fullscan_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'fullscan' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${fullscan_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(fullscan_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${fullscan_DIR}/${_extra}")
endforeach()
