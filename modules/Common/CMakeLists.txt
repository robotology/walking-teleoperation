# Copyright (C) 2021 Fondazione Istituto Italiano di Tecnologia (IIT)
# All Rights Reserved.
# Authors: Giulio Romualdi <giulio.romualdi@iit.it>

# set target name
set(LIBRARY_NAME CommonLibrary)

option(ENABLE_RPATH "Enable RPATH for this library" ON)
mark_as_advanced(ENABLE_RPATH)
include(AddInstallRPATHSupport)
add_install_rpath_support(BIN_DIRS "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_BINDIR}"
  LIB_DIRS "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}"
  INSTALL_NAME_DIR "${CMAKE_INSTALL_PREFIX}"
  DEPENDS ENABLE_RPATH
  USE_LINK_PATH)

# set cpp files
set(${LIBRARY_NAME}_SRC
  src/FingersRetargeting.cpp
  src/HandRetargeting.cpp
  src/HeadRetargeting.cpp
  src/RobotControlHelper.cpp
  src/RetargetingController.cpp
  )

# set hpp files
set(${LIBRARY_NAME}_HDR
  include/FingersRetargeting.hpp
  include/HandRetargeting.hpp
  include/HeadRetargeting.hpp
  include/RobotControlHelper.hpp
  include/RetargetingController.hpp
  )

# add an executable to the project using the specified source files.
add_library(${LIBRARY_NAME} ${${LIBRARY_NAME}_SRC} ${${LIBRARY_NAME}_HDR})

# add include directories to the build.
target_include_directories(${LIBRARY_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_link_libraries(${LIBRARY_NAME}
  ${YARP_LIBRARIES}
  ${iDynTree_LIBRARIES}
  ctrlLib
  UtilityLibrary
  Eigen3::Eigen)
