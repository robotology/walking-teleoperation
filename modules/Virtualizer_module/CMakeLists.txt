# Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)
# All Rights Reserved.
# Authors: Mohamed Babiker Mohamed Elobaid <mohamed.elobaid@iit.it>
#          Giulio Romualdi <giulio.romualdi@iit.it>
#
# set target name

find_package(CybSDK REQUIRED)
set(EXE_TARGET_NAME VirtualizerModule)

option(ENABLE_RPATH "Enable RPATH for this library" ON)
mark_as_advanced(ENABLE_RPATH)
include(AddInstallRPATHSupport)
add_install_rpath_support(BIN_DIRS "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_BINDIR}"
  LIB_DIRS "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}"
  INSTALL_NAME_DIR "${CMAKE_INSTALL_PREFIX}"
  DEPENDS ENABLE_RPATH
  USE_LINK_PATH)

# find virtualizer SDK
file(GLOB CybSDK_SOURCE_H "${CMAKE_CURRENT_SOURCE_DIR}/CybSDK/include/*")
file(GLOB CybSDK_SOURCE_C "${CMAKE_CURRENT_SOURCE_DIR}/CybSDK/src/*")

add_definitions(-DUNICODE)

add_library(CybSDK ${CybSDK_SOURCE_C} ${CybSDK_SOURCE_H})

# set cpp files
set(${EXE_TARGET_NAME}_SRC
  src/main.cpp
  src/VirtualizerModule.cpp
  )

# set hpp files
set(${EXE_TARGET_NAME}_HDR
  include/VirtualizerModule.hpp
  )

# add include directories to the build.
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/CybSDK/include)

# add an executable to the project using the specified source files.
add_executable(${EXE_TARGET_NAME} ${${EXE_TARGET_NAME}_SRC} ${${EXE_TARGET_NAME}_HDR})

target_link_libraries(${EXE_TARGET_NAME}
  ${YARP_LIBRARIES}
  CybSDK
  UtilityLibrary)

install(TARGETS ${EXE_TARGET_NAME} DESTINATION bin)
install(FILES ${CybSDK_LIBRARIES} DESTINATION bin)
