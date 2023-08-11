# SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-3-Clause

set(SRanipal_SDK_DIR $ENV{SRanipal_SDK_DIR} CACHE PATH "Path to the SRanipal C SDK (i.e. the 01_C folder).")

find_path(SRanipalSDK_INCLUDE_DIR SRanipal.h HINTS ${SRanipal_SDK_DIR}/include)
find_library(SRanipalSDK_LIBRARY NAMES SRanipal HINTS ${SRanipal_SDK_DIR}/lib)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(SRanipalSDK DEFAULT_MSG
  SRanipalSDK_LIBRARY SRanipalSDK_INCLUDE_DIR)

if(SRanipalSDK_FOUND AND NOT TARGET SRanipalSDK::SRanipalSDK)
  set(SRanipalSDK_LIBRARIES ${SRanipalSDK_LIBRARY})
  set(SRanipalSDK_INCLUDE_DIRS ${SRanipalSDK_INCLUDE_DIR})

  add_library(SRanipalSDK::SRanipalSDK UNKNOWN IMPORTED)
  set_target_properties(SRanipalSDK::SRanipalSDK PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES ${SRanipalSDK_INCLUDE_DIR}
    IMPORTED_LOCATION ${SRanipalSDK_LIBRARY})

    mark_as_advanced(SRanipalSDK_INCLUDE_DIR SRanipalSDK_LIBRARY)
endif()
