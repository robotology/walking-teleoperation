# SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-3-Clause

# Finds the Cyberith SDK
#
set(CybSDK_FOUND FALSE)
set(CybSDK_DIR $ENV{CybSDK_DIR} CACHE STRING "The directiry containing the Cyberith library")
if(NOT EXISTS ${CybSDK_DIR})
  message( WARNING "variable {CybSDK_DIR} is not defined: ${CybSDK_DIR}" )
  return()
endif()

option (BUILD_SHARED_CybSDK_LIB "Identify if the library type is SHARED or STATIC" FALSE)


##
if (BUILD_SHARED_CybSDK_LIB)
    set(EXTENSION ${CMAKE_SHARED_LIBRARY_SUFFIX})
    set(TYPE "SHARED")
else()
    set(EXTENSION ${CMAKE_STATIC_LIBRARY_SUFFIX})
    set(TYPE "STATIC")
endif()

if(${CMAKE_SIZEOF_VOID_P} EQUAL 8)
    # 64 bits
    set(FOLDER "x64")
elseif(${CMAKE_SIZEOF_VOID_P} EQUAL 4)
    # 32 bits
    set(FOLDER "x86")
endif()
##### Find CybSDK #####

add_library(CybSDK ${TYPE} IMPORTED GLOBAL ${CybSDK_DIR}/${FOLDER}/CybSDK${EXTENSION})
set_target_properties(CybSDK PROPERTIES IMPORTED_LOCATION ${CybSDK_DIR}/${FOLDER}/CybSDK${EXTENSION})
target_include_directories(CybSDK INTERFACE ${CybSDK_DIR}/Include)

set(CybSDK_FOUND TRUE)
