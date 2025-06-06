# SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-3-Clause

# set target name
set(EXE_TARGET_NAME HapticGloveModule)


OPTION(EIGEN_RUNTIME_NO_MALLOC "If ON, it can assert in case of runtime allocation" OFF)

IF(EIGEN_RUNTIME_NO_MALLOC)
  MESSAGE(STATUS "Option EIGEN_RUNTIME_NO_MALLOC on.")
  ADD_DEFINITIONS(-DEIGEN_RUNTIME_NO_MALLOC)
ENDIF(EIGEN_RUNTIME_NO_MALLOC)

yarp_add_idl(HAPTIC_GLOVE_SERVICES thrift/RobotSkinService.thrift
                                   thrift/HapticGloveTeleoperationService.thrift
                                   thrift/HapticGloveService.thrift )

# set cpp files
set(${EXE_TARGET_NAME}_SRC
  src/main.cpp
  src/RobotController.cpp
  src/RobotInterface.cpp
  src/HapticGloveModule.cpp
  src/GloveControlHelper.cpp
  src/KalmanFilter.cpp
  src/MotorEstimation.cpp
  src/RobotMotorsEstimation.cpp
  src/Retargeting.cpp
  src/LinearRegression.cpp
  src/GloveWearable.cpp
  src/Logger.cpp
  src/Teleoperation.cpp
  src/ControlHelper.cpp
  src/RobotSkin.cpp
  ${HAPTIC_GLOVE_SERVICES}
  )

# set hpp files
set(${EXE_TARGET_NAME}_HDR
  include/RobotController.hpp
  include/RobotInterface.hpp
  include/HapticGloveModule.hpp
  include/GloveControlHelper.hpp
  include/KalmanFilter.hpp
  include/MotorEstimation.hpp
  include/RobotMotorsEstimation.hpp
  include/Retargeting.hpp
  include/LinearRegression.hpp
  include/GloveWearable.hpp
  include/Logger.hpp
  include/Teleoperation.hpp
  include/ControlHelper.hpp
  include/RobotSkin.hpp
  )


# add an executable to the project using the specified source files.
add_executable(${EXE_TARGET_NAME} ${${EXE_TARGET_NAME}_SRC} ${${EXE_TARGET_NAME}_HDR})

# add include directories to the build.
target_include_directories(${EXE_TARGET_NAME} PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include)

set(${EXE_TARGET_NAME}_LINKED_LIBS
    ${YARP_LIBRARIES}
    ${iDynTree_LIBRARIES}
    Eigen3::Eigen
    ctrlLib
    UtilityLibrary
    IWear::IWear
    WearableActuators::WearableActuators
    BipedalLocomotion::VectorsCollection
    BipedalLocomotion::ParametersHandlerYarpImplementation
    )

target_link_libraries(${EXE_TARGET_NAME} LINK_PUBLIC ${${EXE_TARGET_NAME}_LINKED_LIBS} )

install(TARGETS ${EXE_TARGET_NAME} DESTINATION bin)
