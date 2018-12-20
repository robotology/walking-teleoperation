# Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)
# All Rights Reserved.
# Authors: Giulio Romualdi <giulio.romualdi@iit.it>

# set target name
set(EXE_TARGET_NAME OculusRetargetingModule)

option(ENABLE_RPATH "Enable RPATH for this library" ON)
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
find_package(OpenCV REQUIRED)
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})
include(FindPackageHandleStandardArgs)

# set cpp files
set(${EXE_TARGET_NAME}_SRC
  src/main.cpp
  src/FingersRetargeting.cpp
  src/HandRetargeting.cpp
  src/HeadRetargeting.cpp
  src/RobotControlHelper.cpp
  src/RetargetingHelper.cpp
  src/OculusModule.cpp)

# set hpp files
set(${EXE_TARGET_NAME}_HDR
  include/FingersRetargeting.hpp
  include/HandRetargeting.hpp
  include/HeadRetargeting.hpp
  include/RobotControlHelper.hpp
  include/RetargetingHelper.hpp
  include/OculusModule.hpp)

# add include directories to the build.
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# add an executable to the project using the specified source files.
add_executable(${EXE_TARGET_NAME} ${${EXE_TARGET_NAME}_SRC} ${${EXE_TARGET_NAME}_HDR})

target_link_libraries(${EXE_TARGET_NAME} LINK_PUBLIC
  ${YARP_LIBRARIES}
  ${iDynTree_LIBRARIES}
  ${OpenCV_LIBS}
  ctrlLib
  UtilityLibrary)

install(TARGETS ${EXE_TARGET_NAME} DESTINATION bin)
