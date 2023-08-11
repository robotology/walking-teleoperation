# SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-3-Clause

set(EXE_TARGET_NAME FaceExpressionsRetargetingModule)

set (${EXE_TARGET_NAME}_SRC src/main.cpp src/FaceExpressionsRetargeting.cpp)

set (${EXE_TARGET_NAME}_HDR include/FaceExpressionsRetargeting.hpp)

add_executable(${EXE_TARGET_NAME} ${${EXE_TARGET_NAME}_SRC} ${${EXE_TARGET_NAME}_HDR})
target_link_libraries(${EXE_TARGET_NAME} PRIVATE ${YARP_LIBRARIES} PkgConfig::libfvad)
target_include_directories(${EXE_TARGET_NAME} PUBLIC include)

install(TARGETS ${EXE_TARGET_NAME} DESTINATION bin)
