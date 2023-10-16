# Copyright (C) 2023 Fondazione Istituto Italiano di Tecnologia (IIT)
# All Rights Reserved.

# set target name
set(EXE_TARGET_NAME OpenXRJoypadModule)

# set cpp files
set(${EXE_TARGET_NAME}_SRC
  src/main.cpp
  src/OpenXRJoypadModule.cpp
  )

# set hpp files
set(${EXE_TARGET_NAME}_HDR
  include/OpenXRJoypadModule.hpp
  )

# add an executable to the project using the specified source files.
add_executable(${EXE_TARGET_NAME} ${${EXE_TARGET_NAME}_SRC} ${${EXE_TARGET_NAME}_HDR})

# add include directories to the build.
target_include_directories( ${EXE_TARGET_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)


target_link_libraries(${EXE_TARGET_NAME} PRIVATE
${YARP_LIBRARIES}
${iDynTree_LIBRARIES}
ctrlLib
UtilityLibrary
Eigen3::Eigen
)

install(TARGETS ${EXE_TARGET_NAME} DESTINATION bin)
