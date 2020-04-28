# Copyright 2020 Istituto Italiano di Tecnologia (IIT)
# @author Kourosh Darvish <kourosh.darvish@iit.it>

# Finds the Sense Glove SDK
#
# This will define the following variables::
#
#   SENSEGLOVE_FOUND         - True if the system has the Sense Glove SDK
#   SENSEGLOVE_INCLUDE_DIRS  - The include directory of the SDK
#   SENSEGLOVE_LIBRARIES     - The name of the libraries to link against.


message("<FindSENSEGLOVE.cmake>")

# Target Platform
set(TARGET_PLATFORM)
if(NOT CMAKE_CL_64)
  set(TARGET_PLATFORM x86)
  message( SEND_ERROR  "The Sense Glove SDK is not supported for this platform: ${TARGET_PLATFORM}")
else()
  set(TARGET_PLATFORM x64)
endif()

# Check Directory of the SenseGlove_DIR

if(NOT DEFINED ENV{SenseGlove_DIR})
  message( SEND_ERROR "Environment variable {SenseGlove_DIR} is not defined: $ENV{SenseGlove_DIR}" )
else()
  message("Environment variable {SenseGlove_DIR}: $ENV{SenseGlove_DIR}" )
endif()

##### Find SenseGlove #####
if(WIN32)
  find_library(SENSEGLOVE SGCore HINTS $ENV{SenseGlove_DIR}/Core/SGCoreCpp/lib/Win/${TARGET_PLATFORM}/${CMAKE_BUILD_TYPE} )
elseif(UNIX)
  find_library(SENSEGLOVE SGCore HINTS $ENV{SenseGlove_DIR}/Core/SGCoreCpp/lib/Linux )
else()
  message(FATAL_ERROR  "The Sense Glove SDK is not supported for this operating system.")
endif()

# Found
set(SENSEGLOVE_FOUND TRUE)
if(NOT SENSEGLOVE )
  message(FATAL_ERROR "Sense Glove not found.")
  set(SENSEGLOVE_FOUND FALSE)
endif()

# Include Directories
set(SENSEGLOVE_INCLUDE_DIRS)
if(SENSEGLOVE_FOUND)
  set(SENSEGLOVE_INCLUDE_DIRS $ENV{SenseGlove_DIR}/Core/SGCoreCpp/incl)
endif()

#if(NOT (EXISTS SENSEGLOVE_INCLUDE_DIRS))
#  message(FATAL_ERROR "SENSEGLOVE_INCLUDE_DIRS not exists: ${SENSEGLOVE_INCLUDE_DIRS}")
#endif()


message(STATUS "SENSEGLOVE_FOUND : ${SENSEGLOVE_LIBRARIES}")

message("</FindSENSEGLOVE.cmake>")