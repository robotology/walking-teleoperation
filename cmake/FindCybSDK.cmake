# Copyright 2018 Istituto Italiano di Tecnologia (IIT)
# @author Mohamed Babiker Mohamed Elobaid <mohamed.elobaid@iit.it>

# Finds the Cyb SDK
#
# This will define the following variables::
#
#   CybSDK_FOUND    - True if the system has the CybSDK 
#   CybSDK_VERSION  - The version of the CybSDK  which was found
#
#

##### Utility #####

# Check Directory Macro
macro(CHECK_DIR _DIR)
  if(NOT EXISTS "${${_DIR}}")
    message(WARNING "Directory \"${${_DIR}}\" not found.")
    set(CybSDK_FOUND FALSE)
    unset(_DIR)
  endif()
endmacro()

# Check Files Macro
macro(CHECK_FILES _FILES _DIR)
  set(_MISSING_FILES)
  foreach(_FILE ${${_FILES}})
    if(NOT EXISTS "${_FILE}")
      get_filename_component(_FILE ${_FILE} NAME)
      set(_MISSING_FILES "${_MISSING_FILES}${_FILE}, ")
    endif()
  endforeach()
  if(_MISSING_FILES)
    message(WARNING "In directory \"${${_DIR}}\" not found files: ${_MISSING_FILES}")
    set(CybSDK_FOUND FALSE)
    unset(_FILES)
  endif()
endmacro()

# Target Platform
set(TARGET_PLATFORM)
if(NOT CMAKE_CL_64)
  set(TARGET_PLATFORM x86)
else()
  set(TARGET_PLATFORM x64)
endif()

##### Find CybSDK #####

# Found
set(CybSDK_FOUND TRUE)
if(MSVC_VERSION LESS 1700)
  message(WARNING "CybSDK supported Visual Studio 2012 or later.")
  set(CybSDK_FOUND FALSE)
endif()

# Root Directoty
set(CybSDK_DIR)
if(CybSDK_FOUND)
  set(CybSDK_DIR $ENV{CybSDK_DIR} CACHE PATH "CybSDK Path." FORCE)
  check_dir(CybSDK_DIR)
endif()

# Include Directories
set(CybSDK_INCLUDE_DIRS)
if(CybSDK_FOUND)
  set(CybSDK_INCLUDE_DIRS ${CybSDK_DIR}/include)
  check_dir(CybSDK_INCLUDE_DIRS)
endif()

# Library Directories
set(CybSDK_LIBRARY_DIRS)
if(CybSDK_FOUND)
  set(CybSDK_LIBRARY_DIRS ${CybSDK_DIR}/Plugins/${TARGET_PLATFORM})
  check_dir(CybSDK_LIBRARY_DIRS)
endif()

# Dependencies
set(CybSDK_LIBRARIES)
if(CybSDK_FOUND)
  set(CybSDK_LIBRARIES ${CybSDK_LIBRARY_DIRS}/CybSDK.dll)
  check_files(CybSDK_LIBRARIES CybSDK_LIBRARY_DIRS)
endif()

message(STATUS "CybSDK_FOUND : ${CybSDK_LIBRARIES}")

