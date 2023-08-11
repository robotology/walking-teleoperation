# SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-3-Clause

# set target name
set(EXE_TARGET_NAME OculusRetargetingModule)

# set cpp files
set(${EXE_TARGET_NAME}_SRC
  src/main.cpp
  src/FingersRetargeting.cpp
  src/HandRetargeting.cpp
  src/HeadRetargeting.cpp
  src/RobotControlHelper.cpp
  src/RetargetingController.cpp
  src/OculusModule.cpp
  )

# set hpp files
set(${EXE_TARGET_NAME}_HDR
  include/FingersRetargeting.hpp
  include/HandRetargeting.hpp
  include/HeadRetargeting.hpp
  include/RobotControlHelper.hpp
  include/RetargetingController.hpp
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
  target_link_libraries(${EXE_TARGET_NAME} LINK_PUBLIC
    ${YARP_LIBRARIES}
    ${iDynTree_LIBRARIES}
    ctrlLib
    UtilityLibrary
    Eigen3::Eigen
    matlogger2::matlogger2)
else(ENABLE_LOGGER)
  target_link_libraries(${EXE_TARGET_NAME} LINK_PUBLIC
    ${YARP_LIBRARIES}
    ${iDynTree_LIBRARIES}
    ctrlLib
    UtilityLibrary
    Eigen3::Eigen
)
endif()

install(TARGETS ${EXE_TARGET_NAME} DESTINATION bin)
