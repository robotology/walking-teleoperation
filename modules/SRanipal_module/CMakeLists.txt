# Copyright (C) 2021 Fondazione Istituto Italiano di Tecnologia (IIT)
# All Rights Reserved.

if (NOT WIN32)
#    message(FATAL_ERROR "The SRanipalModule is available only on Windows")
endif()

set(EXE_TARGET_NAME SRanipalModule)

set (${EXE_TARGET_NAME}_SRC src/main.cpp
                            src/SRanipalModule.cpp
                            src/SRanipalInterface.cpp
                            src/EyelidsRetargeting.cpp
                            src/FaceExpressionsRetargeting.cpp
                            src/GazeRetargeting.cpp)

set (${EXE_TARGET_NAME}_HDR include/SRanipalModule.hpp
                            include/SRanipalInterface.hpp
                            include/EyelidsRetargeting.hpp
                            include/FaceExpressionsRetargeting.hpp
                            include/GazeRetargeting.hpp)

add_executable(${EXE_TARGET_NAME} ${${EXE_TARGET_NAME}_SRC} ${${EXE_TARGET_NAME}_HDR})
target_link_libraries(${EXE_TARGET_NAME} PUBLIC ${YARP_LIBRARIES} Eigen3::Eigen iDynTree::idyntree-core SRanipalSDK::SRanipalSDK)
target_include_directories(${EXE_TARGET_NAME} PUBLIC include)

install(TARGETS ${EXE_TARGET_NAME} DESTINATION bin)
