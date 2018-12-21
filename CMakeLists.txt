# Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)
# All Rights Reserved.
# Authors: Giulio Romualdi <giulio.romualdi@iit.it>

cmake_minimum_required(VERSION 3.5)
set(CMAKE_CXX_STANDARD 14)

project(retargeting)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

find_package(YARP REQUIRED)
find_package(YCM REQUIRED)

add_subdirectory(modules)
add_subdirectory(app)

 # Include clang-format target
include(AddClangFormatTarget)

include(AddUninstallTarget)
