# Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)
# All Rights Reserved.
# Authors: Kourosh Darvish <kourosh.darvish@iit.it>

# set target name
set(EXE_TARGET_NAME HapticGloveModule)

option(ENABLE_RPATH "Enable RPATH for this library" ON)
option(ENABLE_LOGGER "Enable logger using matlogger2" OFF)
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

mark_as_advanced(ENABLE_RPATH)
include(AddInstallRPATHSupport)
add_install_rpath_support(BIN_DIRS "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_BINDIR}"
  LIB_DIRS "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}"
  INSTALL_NAME_DIR "${CMAKE_INSTALL_PREFIX}"
  DEPENDS ENABLE_RPATH
  USE_LINK_PATH)

# Find required package
find_package(ICUB REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(iDynTree REQUIRED)
if(ENABLE_LOGGER)
  add_definitions(-DENABLE_LOGGER)
  find_package(matlogger2 REQUIRED)
endif(ENABLE_LOGGER)
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})
include(FindPackageHandleStandardArgs)
if(WIN32)
  # MSVC standard library doesn't include things like M_PI_2 without this
  add_definitions(-D_USE_MATH_DEFINES)
endif()

find_library(SENSEGLOVE SGCore HINTS $ENV{SenseGlove_DIR}/Core/SGCoreCpp/lib/Win/x64/Release )
set(SENSEGLOVE_INCLUDE_DIRS $ENV{SenseGlove_DIR}/Core/SGCoreCpp/incl)
#find_package(SENSEGLOVE REQUIRED)
#if(SENSEGLOVE_FOUND)
#  include_directories($SENSEGLOVE_INCLUDE_DIR)
#endif(SENSEGLOVE_FOUND)



# set cpp files
set(${EXE_TARGET_NAME}_SRC
  src/main.cpp
  src/HapticGloveFingersRetargeting.cpp
  src/HapticGloveRobotControlHelper.cpp
  src/HapticGloveRetargetingController.cpp
  src/HapticGloveModule.cpp
  src/GloveControlHelper.cpp
  )

# set hpp files
set(${EXE_TARGET_NAME}_HDR
  include/HapticGloveFingersRetargeting.hpp
  include/HapticGloveRobotControlHelper.hpp
  include/HapticGloveRetargetingController.hpp
  include/HapticGloveModule.hpp
  include/GloveControlHelper.hpp
  )

# add include directories to the build.
message("{SENSEGLOVE_INCLUDE_DIRS}: ${SENSEGLOVE_INCLUDE_DIRS}")

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${SENSEGLOVE_INCLUDE_DIRS}
)

# add an executable to the project using the specified source files.
add_executable(${EXE_TARGET_NAME} ${${EXE_TARGET_NAME}_SRC} ${${EXE_TARGET_NAME}_HDR})

message("YARP_LIBRARIES: ${YARP_LIBRARIES}")
message("iDynTree_LIBRARIES: ${iDynTree_LIBRARIES}")
message("SENSEGLOVE_LIBRARIES: ${SENSEGLOVE_LIBRARIES}")
message("ctrlLib: ${ctrlLib}")
message("UtilityLibrary: ${UtilityLibrary}")

if(ENABLE_LOGGER)
  target_link_libraries(${EXE_TARGET_NAME} LINK_PUBLIC
    ${YARP_LIBRARIES}
    ${iDynTree_LIBRARIES}
    ${SENSEGLOVE}
    ctrlLib
    UtilityLibrary
    matlogger2::matlogger2
    )
else(ENABLE_LOGGER)
  target_link_libraries(${EXE_TARGET_NAME} LINK_PUBLIC
    ${YARP_LIBRARIES}
    ${iDynTree_LIBRARIES}
    ${SENSEGLOVE}
    ctrlLib
    UtilityLibrary
)
endif()

message("BIN: ${bin}")
message(" {CMAKE_BUILD_TYPE}: ${CMAKE_BUILD_TYPE}")
install(TARGETS ${EXE_TARGET_NAME} DESTINATION bin)
if(WIN32)
  install(FILES $ENV{SenseGlove_DIR}/Core/SGCoreCpp/lib/Win/x64/Release/SGCore.dll DESTINATION bin
  )
endif()
