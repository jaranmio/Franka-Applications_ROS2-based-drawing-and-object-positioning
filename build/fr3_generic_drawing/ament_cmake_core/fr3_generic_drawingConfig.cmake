# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_fr3_generic_drawing_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED fr3_generic_drawing_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(fr3_generic_drawing_FOUND FALSE)
  elseif(NOT fr3_generic_drawing_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(fr3_generic_drawing_FOUND FALSE)
  endif()
  return()
endif()
set(_fr3_generic_drawing_CONFIG_INCLUDED TRUE)

# output package information
if(NOT fr3_generic_drawing_FIND_QUIETLY)
  message(STATUS "Found fr3_generic_drawing: 0.0.0 (${fr3_generic_drawing_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'fr3_generic_drawing' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${fr3_generic_drawing_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(fr3_generic_drawing_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${fr3_generic_drawing_DIR}/${_extra}")
endforeach()
