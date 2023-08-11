# SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-3-Clause

# set target name
set(EXE_TARGET_NAME XsensRetargetingModule)

option(ENABLE_RPATH "Enable RPATH for this library" ON)
mark_as_advanced(ENABLE_RPATH)
include(AddInstallRPATHSupport)
add_install_rpath_support(BIN_DIRS "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_BINDIR}"
  LIB_DIRS "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}"
  INSTALL_NAME_DIR "${CMAKE_INSTALL_PREFIX}"
  DEPENDS ENABLE_RPATH
  USE_LINK_PATH)

# Find required package
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})
include(FindPackageHandleStandardArgs)

# set cpp files
set(${EXE_TARGET_NAME}_SRC
  src/main.cpp
  src/XsensRetargeting.cpp)

# set hpp files
set(${EXE_TARGET_NAME}_HDR
  include/XsensRetargeting.hpp)

# add include directories to the build.
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# add an executable to the project using the specified source files.
add_executable(${EXE_TARGET_NAME} ${${EXE_TARGET_NAME}_SRC} ${${EXE_TARGET_NAME}_HDR} )

target_link_libraries(${EXE_TARGET_NAME} LINK_PUBLIC
  ${YARP_LIBRARIES}
  ${iDynTree_LIBRARIES}
  ctrlLib
  HumanDynamicsEstimation::HumanStateMsg
  UtilityLibrary)

install(TARGETS ${EXE_TARGET_NAME} DESTINATION bin)
