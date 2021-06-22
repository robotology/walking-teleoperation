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
#######
#if(WIN32)
#	if(DEFINED $ENV{SenseGlove_DIR})
#		SET(SENSEGLOVE_DIR "$ENV{SenseGlove_DIR}/Core/SGCoreCpp/lib/Win/x64/Release")
#	endif()
#else()
#	if(DEFINED $ENV{SenseGlove_DIR})
#		SET(SENSEGLOVE_DIR "$ENV{SenseGlove_DIR}/Core/SGCoreCpp/lib/Linux")
#	endif()
#endif()

#find_path(SENSEGLOVE_INCLUDE_DIR NAMES Core/SGCoreCpp/incl/DeviceList.h Core/SGCoreCpp/incl/SenseGlove.h PATHS ${SENSEGLOVE_DIR})
#find_library(SENSEGLOVE_LIBRARY NAMES SGCore PATHS ${SENSEGLOVE_DIR} )

#find_package_handle_standard_args(SENSEGLOVE DEFAULT_MSG 
#	SENSEGLOVE_INCLUDE_DIR
#	SENSEGLOVE_LIBRARY )

#if(SENSEGLOVE_FOUND)
#	set(SENSEGLOVE_LIBRARIES ${SENSEGLOVE_LIBRARY})
#	set(SENSEGLOVE_INCLUDE_DIRS ${SENSEGLOVE_DIR})
#	mark_as_advanced( SENSEGLOVE_LIBRARY
#										SENSEGLOVE_INCLUDE_DIR
#										SenseGlove_DIR )
	
#	message("SENSEGLOVE FOUND!")
#else()
#	set(SenseGlove_DIR "" CACHE STRING 
#			"an optional hint to a directory for finding `SenseGlove`" )
#	message("SENSEGLOVE not FOUND!")
#endif()
##########
#find_package(SenseGlove REQUIRED)
if(SENSEGLOVE_FOUND)
#  include_directories($SENSEGLOVE_INCLUDE_DIR)
   message(WARNING "${EXE_TARGET_NAME} SenseGlove is not found")
endif(SENSEGLOVE_FOUND)



# set cpp files
set(${EXE_TARGET_NAME}_SRC
  src/main.cpp
  src/RobotController_hapticGlove.cpp
  src/RobotControlInterface_hapticGlove.cpp
  src/RobotControlHelper_hapticGlove.cpp
  src/HapticGloveModule.cpp
  src/GloveControlHelper.cpp
  src/KalmanFilter.cpp
  src/MotorEstimation.cpp
  src/RobotMotorsEstimation.cpp
  src/Retargeting.cpp
  src/LinearRegression.cpp
  src/GloveWearable.cpp
  )

# set hpp files
set(${EXE_TARGET_NAME}_HDR
  include/RobotController_hapticGlove.hpp
  include/RobotControlInterface_hapticGlove.hpp
  include/RobotControlHelper_hapticGlove.hpp
  include/HapticGloveModule.hpp
  include/GloveControlHelper.hpp
  include/KalmanFilter.hpp
  include/MotorEstimation.hpp
  include/RobotMotorsEstimation.hpp
  include/Retargeting.hpp
  include/LinearRegression.hpp
  include/GloveWearable.hpp
  )

# add include directories to the build.

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
#  ${SenseGlove_INCLUDE_DIR}
)

# add an executable to the project using the specified source files.
add_executable(${EXE_TARGET_NAME} ${${EXE_TARGET_NAME}_SRC} ${${EXE_TARGET_NAME}_HDR})

message("YARP_LIBRARIES: ${YARP_LIBRARIES}")
message("iDynTree_LIBRARIES: ${iDynTree_LIBRARIES}")
#message("SenseGlove_LIBRARIES: ${SenseGlove_LIBRARIES}")
message("ctrlLib: ${ctrlLib}")
message("UtilityLibrary: ${UtilityLibrary}")

if(ENABLE_LOGGER)
  target_link_libraries(${EXE_TARGET_NAME} LINK_PUBLIC
    ${YARP_LIBRARIES}
    ${iDynTree_LIBRARIES}
#    ${SenseGlove_LIBRARIES}
    SenseGlove
    ctrlLib
    UtilityLibrary
    matlogger2::matlogger2
    walking-teleoperation::ML
    IWear::IWear
    WearableActuators::WearableActuators

    )
else(ENABLE_LOGGER)
  target_link_libraries(${EXE_TARGET_NAME} LINK_PUBLIC
    ${YARP_LIBRARIES}
    ${iDynTree_LIBRARIES}
#    ${SenseGlove_LIBRARIES}
    SenseGlove
    ctrlLib
    UtilityLibrary
    walking-teleoperation::ML
    IWear::IWear
    WearableActuators::WearableActuators

)
endif()

message("BIN: ${bin}")
message(" {CMAKE_BUILD_TYPE}: ${CMAKE_BUILD_TYPE}")
install(TARGETS ${EXE_TARGET_NAME} DESTINATION bin)

#install(FILES ${SenseGlove_LIBRARIES} DESTINATION lib RENAME libSGCore.so)
if(WIN32)
	install(FILES $ENV{SenseGlove_DIR}/SGCore.dll DESTINATION bin )
else()
        install(FILES ${SenseGlove} DESTINATION lib RENAME libSGCoreCpp.so)
endif() 
