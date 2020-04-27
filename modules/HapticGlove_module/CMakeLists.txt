# Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)
# All Rights Reserved.
# Authors: Kourosh Darvish <kourosh.darvish@iit.it>

# set target name
set(EXE_TARGET_NAME HapticGloveModule)

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

# set cpp files
set(${EXE_TARGET_NAME}_SRC
  src/main.cpp
  src/HapticGloveFingersRetargeting.cpp
  src/HapticGloveRobotControlHelper.cpp
  src/HapticGloveRetargetingController.cpp
  src/HapticGloveModule.cpp
  )

# set hpp files
set(${EXE_TARGET_NAME}_HDR
  include/HapticGloveFingersRetargeting.hpp
  include/HapticGloveRobotControlHelper.hpp
  include/HapticGloveRetargetingController.hpp
  include/HapticGloveModule.hpp
  )

# add include directories to the build.
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# add an executable to the project using the specified source files.
add_executable(${EXE_TARGET_NAME} ${${EXE_TARGET_NAME}_SRC} ${${EXE_TARGET_NAME}_HDR})

if(ENABLE_LOGGER)
  target_link_libraries(${EXE_TARGET_NAME} LINK_PUBLIC
    ${YARP_LIBRARIES}
    ${iDynTree_LIBRARIES}
    ctrlLib
    UtilityLibrary
    matlogger2::matlogger2)
else(ENABLE_LOGGER)
  target_link_libraries(${EXE_TARGET_NAME} LINK_PUBLIC
    ${YARP_LIBRARIES}
    ${iDynTree_LIBRARIES}
    ctrlLib
    UtilityLibrary
)
endif()

install(TARGETS ${EXE_TARGET_NAME} DESTINATION bin)
