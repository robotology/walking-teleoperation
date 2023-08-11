# SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-3-Clause

cmake_minimum_required(VERSION 3.5)
set(CMAKE_CXX_STANDARD 14)

project(walking-teleoperation
  VERSION 1.3.3)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
include(WalkingTeleoperationFindDependencies)

include(FindPackageHandleStandardArgs)
if(WIN32)
  # MSVC standard library doesn't include things like M_PI_2 without this
  add_definitions(-D_USE_MATH_DEFINES)
endif()



add_subdirectory(modules)
add_subdirectory(app)

 # Include clang-format target
include(AddClangFormatTarget)

include(AddUninstallTarget)
