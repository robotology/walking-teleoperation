# Copyright 2020 Istituto Italiano di Tecnologia (IIT)
# @author Kourosh Darvish <kourosh.darvish@iit.it>

# Finds the Cyberith SDK
#
set(CybSDK_DIR $ENV{CybSDK_DIR} CACHE STRING "The directiry containing the Cyberith library")
if(NOT DEFINED CybSDK_DIR)
  message( FATAL_ERROR "variable {CybSDK_DIR} is not defined: ${CybSDK_DIR}" )
endif()

option (BUILD_SHARED_LIBS "Please identify the library type SHARED or STATIC" FALSE)


##
if (BUILD_SHARED_LIBS)
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
