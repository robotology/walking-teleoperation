# SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-3-Clause

# This module checks if all the dependencies are installed and if the
# dependencies to build some parts of WALKING_TELEOPERATION are satisfied.
# For every dependency, it creates the following variables:
#
# WALKING_TELEOPERATION_USE_${Package}: Can be disabled by the user if he doesn't want to use that
#                      dependency.
# WALKING_TELEOPERATION_HAS_${Package}: Internal flag. It should be used to check if a part of
#                      WALKING_TELEOPERATION should be built. It is on if WALKING_TELEOPERATION_USE_${Package} is
#                      on and either the package was found or will be built.
# WALKING_TELEOPERATION_BUILD_${Package}: Internal flag. Used to check if WALKING_TELEOPERATION has to build an
#                        external package.
# WALKING_TELEOPERATION_BUILD_DEPS_${Package}: Internal flag. Used to check if dependencies
#                             required to build the package are available.
# WALKING_TELEOPERATION_HAS_SYSTEM_${Package}: Internal flag. Used to check if the package is
#                             available on the system.
# WALKING_TELEOPERATION_USE_SYSTEM_${Package}: This flag is shown only for packages in the
#                             extern folder that were also found on the system
#                             (TRUE by default). If this flag is enabled, the
#                             system installed library will be used instead of
#                             the version shipped with WALKING_CONTROLLERS.


include(CMakeDependentOption)

# Check if a package is installed and set some cmake variables
macro(checkandset_dependency package)

  string(TOUPPER ${package} PKG)

  # WALKING_TELEOPERATION_HAS_SYSTEM_${package}
  if(${package}_FOUND OR ${PKG}_FOUND)
    set(WALKING_TELEOPERATION_HAS_SYSTEM_${package} TRUE)
  else()
    set(WALKING_TELEOPERATION_HAS_SYSTEM_${package} FALSE)
  endif()

  # WALKING_TELEOPERATION_USE_${package}
  cmake_dependent_option(WALKING_TELEOPERATION_USE_${package} "Use package ${package}" TRUE
                         WALKING_TELEOPERATION_HAS_SYSTEM_${package} FALSE)
  mark_as_advanced(WALKING_TELEOPERATION_USE_${package})

  # WALKING_TELEOPERATION_USE_SYSTEM_${package}
  set(WALKING_TELEOPERATION_USE_SYSTEM_${package} ${WALKING_TELEOPERATION_USE_${package}} CACHE INTERNAL "Use system-installed ${package}, rather than a private copy (recommended)" FORCE)
  if(NOT "${package}" STREQUAL "${PKG}")
    unset(WALKING_TELEOPERATION_USE_SYSTEM_${PKG} CACHE) # Deprecated since WALKING_TELEOPERATION 3.2
  endif()

  # WALKING_TELEOPERATION_HAS_${package}
  if(${WALKING_TELEOPERATION_HAS_SYSTEM_${package}})
    set(WALKING_TELEOPERATION_HAS_${package} ${WALKING_TELEOPERATION_USE_${package}})
  else()
    set(WALKING_TELEOPERATION_HAS_${package} FALSE)
  endif()

endmacro()

macro(WALKING_TELEOPERATION_DEPENDENT_OPTION _option _doc _default _deps _force)

  if(DEFINED ${_option})
    get_property(_option_strings_set CACHE ${_option} PROPERTY STRINGS SET)
    if(_option_strings_set)
      # If the user thinks he is smarter than the machine, he deserves an error
      get_property(_option_strings CACHE ${_option} PROPERTY STRINGS)
      list(GET _option_strings 0 _option_strings_first)
      string(REGEX REPLACE ".+\"(.+)\".+" "\\1" _option_strings_first "${_option_strings_first}")
      list(LENGTH _option_strings _option_strings_length)
      math(EXPR _option_strings_last_index "${_option_strings_length} - 1")
      list(GET _option_strings ${_option_strings_last_index} _option_strings_last)
      if("${${_option}}" STREQUAL "${_option_strings_last}")
        message(SEND_ERROR "That was a trick, you cannot outsmart me! I will never let you win! ${_option} stays OFF until I say so! \"${_option_strings_first}\" is needed to enable ${_option}. Now stop bothering me, and install your dependencies, if you really want to enable this option.")
      endif()
      unset(${_option} CACHE)
    endif()
  endif()

  cmake_dependent_option(${_option} "${_doc}" ${_default} "${_deps}" ${_force})

  unset(_missing_deps)
  foreach(_dep ${_deps})
    string(REGEX REPLACE " +" ";" _depx "${_dep}")
    if(NOT (${_depx}))
      list(APPEND _missing_deps "${_dep}")
    endif()
  endforeach()

  if(DEFINED _missing_deps)
    set(${_option}_disable_reason " (dependencies unsatisfied: \"${_missing_deps}\")")
    # Set a value that can be visualized on ccmake and on cmake-gui, but
    # still evaluates to false
    set(${_option} "OFF - Dependencies unsatisfied: '${_missing_deps}' - ${_option}-NOTFOUND" CACHE STRING "${_option_doc}" FORCE)
    string(REPLACE ";" "\;" _missing_deps "${_missing_deps}")
    set_property(CACHE ${_option}
                PROPERTY STRINGS "OFF - Dependencies unsatisfied: '${_missing_deps}' - ${_option}-NOTFOUND"
                                 "OFF - You can try as much as you want, but '${_missing_deps}' is needed to enable ${_option} - ${_option}-NOTFOUND"
                                 "OFF - Are you crazy or what? '${_missing_deps}' is needed to enable ${_option} - ${_option}-NOTFOUND"
                                 "OFF - Didn't I already tell you that '${_missing_deps}' is needed to enable ${_option}? - ${_option}-NOTFOUND"
                                 "OFF - Stop it! - ${_option}-NOTFOUND"
                                 "OFF - This is insane! Leave me alone! - ${_option}-NOTFOUND"
                                 "ON - All right, you win. The option is enabled. Are you happy now? You just broke the build.")
    # Set non-cache variable that will override the value in current scope
    # For parent scopes, the "-NOTFOUND ensures that the variable still
    # evaluates to false
    set(${_option} ${_force})
  endif()

endmacro()



################################################################################
# Find all packages

find_package(YARP REQUIRED)
find_package(YCM REQUIRED)
find_package(ICUB REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(iDynTree REQUIRED)

# Enable RPATH
option(ENABLE_RPATH "Enable RPATH for this library" ON)

mark_as_advanced(ENABLE_RPATH)
include(AddInstallRPATHSupport)
add_install_rpath_support(BIN_DIRS "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_BINDIR}"
  LIB_DIRS "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}"
  INSTALL_NAME_DIR "${CMAKE_INSTALL_PREFIX}"
  DEPENDS ENABLE_RPATH
  USE_LINK_PATH)

# Enable logger
option(ENABLE_LOGGER "Enable logger using matlogger2" OFF)
if(ENABLE_LOGGER)
  add_definitions(-DENABLE_LOGGER)
  find_package(matlogger2 REQUIRED)
endif(ENABLE_LOGGER)


find_package(PkgConfig QUIET)
if (PkgConfig_FOUND)
    pkg_check_modules(libfvad QUIET IMPORTED_TARGET libfvad)
endif()
checkandset_dependency(libfvad)

find_package(HumanDynamicsEstimation QUIET)
checkandset_dependency(HumanDynamicsEstimation)

find_package(CybSDK QUIET)
checkandset_dependency(CybSDK)

find_package(SRanipalSDK QUIET)
checkandset_dependency(SRanipalSDK)

find_package(IWear QUIET)
checkandset_dependency(IWear)

find_package(WearableActuators QUIET)
checkandset_dependency(WearableActuators)


WALKING_TELEOPERATION_dependent_option(WALKING_TELEOPERATION_COMPILE_XsensModule "Compile Xsens Module?" ON WALKING_TELEOPERATION_HAS_HumanDynamicsEstimation OFF)
WALKING_TELEOPERATION_dependent_option(WALKING_TELEOPERATION_COMPILE_VirtualizerModule "Compile Virtualizer Module?" ON WALKING_TELEOPERATION_HAS_CybSDK OFF)
WALKING_TELEOPERATION_dependent_option(WALKING_TELEOPERATION_COMPILE_FaceExpressionsRetargetingModule "Compile Face Expressions Module?" ON WALKING_TELEOPERATION_USE_libfvad OFF)
WALKING_TELEOPERATION_dependent_option(WALKING_TELEOPERATION_COMPILE_SRanipalModule "Compile SRanipal Module?" ON WALKING_TELEOPERATION_USE_SRanipalSDK OFF)
WALKING_TELEOPERATION_dependent_option(WALKING_TELEOPERATION_COMPILE_HapticGloveModule "Compile Haptic Glove Module?" ON "WALKING_TELEOPERATION_USE_IWear;WALKING_TELEOPERATION_USE_WearableActuators" OFF)
