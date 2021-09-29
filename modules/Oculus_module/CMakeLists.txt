# Copyright (C) 2021 Fondazione Istituto Italiano di Tecnologia (IIT)
# All Rights Reserved.
# Authors: Giulio Romualdi <giulio.romualdi@iit.it>

# set target name
set(EXE_TARGET_NAME OculusRetargetingModule)

option(ENABLE_RPATH "Enable RPATH for this library" ON)
option(ENABLE_LOGGER "Enable logger using matlogger2" OFF)

mark_as_advanced(ENABLE_RPATH)
include(AddInstallRPATHSupport)
add_install_rpath_support(BIN_DIRS "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_BINDIR}"
  LIB_DIRS "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}"
  INSTALL_NAME_DIR "${CMAKE_INSTALL_PREFIX}"
  DEPENDS ENABLE_RPATH
  USE_LINK_PATH)

# Find required package
if(ENABLE_LOGGER)
  add_definitions(-DENABLE_LOGGER)
  find_package(matlogger2 REQUIRED)
endif(ENABLE_LOGGER)
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})
include(FindPackageHandleStandardArgs)

# set cpp files
set(${EXE_TARGET_NAME}_SRC
  src/main.cpp
  src/OculusModule.cpp
  )

# set hpp files
set(${EXE_TARGET_NAME}_HDR
  include/OculusModule.hpp
  )

set(${EXE_TARGET_NAME}_THRIFT_HDR
  thrifts/TeleoperationCommands.thrift
)

# add include directories to the build.
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Application target calls
yarp_add_idl(${EXE_TARGET_NAME}_THRIFT_GEN_FILES ${${EXE_TARGET_NAME}_THRIFT_HDR})

# add an executable to the project using the specified source files.
add_executable(${EXE_TARGET_NAME} ${${EXE_TARGET_NAME}_SRC} ${${EXE_TARGET_NAME}_HDR}
    ${${EXE_TARGET_NAME}_THRIFT_GEN_FILES})

if(ENABLE_LOGGER)
  target_link_libraries(${EXE_TARGET_NAME} PRIVATE
    ${YARP_LIBRARIES}
    ${iDynTree_LIBRARIES}
    ctrlLib
    CommonLibrary
    matlogger2::matlogger2)
else(ENABLE_LOGGER)
  target_link_libraries(${EXE_TARGET_NAME} PRIVATE
    ${YARP_LIBRARIES}
    ${iDynTree_LIBRARIES}
    ctrlLib
    UtilityLibrary
    CommonLibrary
)
endif()

install(TARGETS ${EXE_TARGET_NAME} DESTINATION bin)
