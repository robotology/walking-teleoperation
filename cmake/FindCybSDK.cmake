# Copyright 2020 Istituto Italiano di Tecnologia (IIT)
# @author Kourosh Darvish <kourosh.darvish@iit.it>

# Finds the Sense Glove SDK
#
# This will define the following variables::
#
#   CybSDK_FOUND         - True if the system has the Sense Glove SDK
#   CybSDK_INCLUDE_DIR  - The include directory of the SDK
#   CybSDK_LIBRARIES     - The name of the libraries to link against.
###############################

# Check Directory of the CybSDK_DIR
if(NOT DEFINED ENV{CybSDK_DIR})
  message( FATAL_ERROR "Environment variable {CybSDK_DIR} is not defined: $ENV{CybSDK_DIR}" )
else()
  message("Environment variable {CybSDK_DIR}: $ENV{CybSDK_DIR}" )
endif()

if(NOT DEFINED ENV{CybSDK_INCLUDE_DIRS})
  message( FATAL_ERROR "Environment variable {CybSDK_INCLUDE_DIRS} is not defined: $ENV{CybSDK_INCLUDE_DIRS}" )
else()
  message("Environment variable {CybSDK_INCLUDE_DIRS}: $ENV{CybSDK_INCLUDE_DIRS}" )
endif()
##### Find CybSDK #####

find_path(CybSDK_INCLUDE_DIR NAMES CVirt.h CVirtDevice.h HINTS $ENV{CybSDK_INCLUDE_DIRS})

if(WIN32)
  find_library(CybSDK_LIBRARY NAMES CybSDK HINTS ${CybSDK_DIR})
else()
  message(FATAL_ERROR  "The Sense Glove SDK is not supported for this operating system.")
endif()

find_package_handle_standard_args(CybSDK_LIBRARY DEFAULT_MSG CybSDK_LIBRARIES
                                                      CybSDK_INCLUDE_DIR)


if(CybSDK_INCLUDE_DIR AND CybSDK_LIBRARY)
	set(CybSDK_FOUND TRUE)
else()
	set(CybSDK_FOUND FALSE)
endif()

set(CybSDK_LIBRARIES ${CybSDK_LIBRARY})

message(STATUS "CybSDK_FOUND: ${CybSDK_FOUND}")
message(STATUS "CybSDK_LIBRARIES: ${CybSDK_LIBRARIES}")
message(STATUS "CybSDK_INCLUDE_DIR: ${CybSDK_INCLUDE_DIR}")

#set(CybSDK_FOUND TRUE)
#if(NOT CybSDK )
#  message(FATAL_ERROR "Sense Glove not found.")
#  set(CybSDK_FOUND FALSE)
#endif()

# Include Directories
#set(CybSDK_INCLUDE_DIRS)
#if(CybSDK_FOUND)
#  set(CybSDK_INCLUDE_DIRS $ENV{CybSDK_DIR}/Core/SGCoreCpp/incl)
#endif()

#if(NOT (EXISTS CybSDK_INCLUDE_DIRS))
#  message(FATAL_ERROR "CybSDK_INCLUDE_DIRS not exists: ${CybSDK_INCLUDE_DIRS}")
#endif()


