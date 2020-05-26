# Copyright 2020 Istituto Italiano di Tecnologia (IIT)
# @author Kourosh Darvish <kourosh.darvish@iit.it>

# Finds the Sense Glove SDK
#
# This will define the following variables::
#
#   SenseGlove_FOUND         - True if the system has the Sense Glove SDK
#   SenseGlove_INCLUDE_DIR  - The include directory of the SDK
#   SenseGlove_LIBRARIES     - The name of the libraries to link against.
###############################

# Check Directory of the SenseGlove_DIR
if(NOT DEFINED ENV{SenseGlove_DIR})
  message( FATAL_ERROR "Environment variable {SenseGlove_DIR} is not defined: $ENV{SenseGlove_DIR}" )
else()
  message("Environment variable {SenseGlove_DIR}: $ENV{SenseGlove_DIR}" )
endif()

if(NOT DEFINED ENV{SenseGlove_INCLUDE_DIRS})
  message( FATAL_ERROR "Environment variable {SenseGlove_INCLUDE_DIRS} is not defined: $ENV{SenseGlove_INCLUDE_DIRS}" )
else()
  message("Environment variable {SenseGlove_DIR}: $ENV{SenseGlove_INCLUDE_DIRS}" )
endif()
##### Find SenseGlove #####

find_path(SenseGlove_INCLUDE_DIR NAMES DeviceList.h SenseGlove.h HINTS $ENV{SenseGlove_INCLUDE_DIRS})

if(WIN32)
  find_library(SenseGlove_LIBRARY NAMES SGCore HINTS ${SenseGlove_DIR} NO_DEFAULT_PATH)
elseif(UNIX)
  find_library(SenseGlove_LIBRARY  NAMES SGCore.so PATHS ${SenseGlove_DIR} NO_DEFAULT_PATH)
else()
  message(FATAL_ERROR  "The Sense Glove SDK is not supported for this operating system.")
endif()


if(SenseGlove_INCLUDE_DIR AND SenseGlove_LIBRARY)
	set(SenseGlove_FOUND TRUE)
else()
	set(SenseGlove_FOUND FALSE)
endif()

set(SenseGlove_LIBRARIES ${SenseGlove_LIBRARY})

message(STATUS "SenseGlove_FOUND: ${SenseGlove_FOUND}")
message(STATUS "SenseGlove_LIBRARIES: ${SenseGlove_LIBRARIES}")
message(STATUS "SenseGlove_INCLUDE_DIR: ${SenseGlove_INCLUDE_DIR}")

#set(SENSEGLOVE_FOUND TRUE)
#if(NOT SENSEGLOVE )
#  message(FATAL_ERROR "Sense Glove not found.")
#  set(SENSEGLOVE_FOUND FALSE)
#endif()

# Include Directories
#set(SENSEGLOVE_INCLUDE_DIRS)
#if(SENSEGLOVE_FOUND)
#  set(SENSEGLOVE_INCLUDE_DIRS $ENV{SenseGlove_DIR}/Core/SGCoreCpp/incl)
#endif()

#if(NOT (EXISTS SENSEGLOVE_INCLUDE_DIRS))
#  message(FATAL_ERROR "SENSEGLOVE_INCLUDE_DIRS not exists: ${SENSEGLOVE_INCLUDE_DIRS}")
#endif()


