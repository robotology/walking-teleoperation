# Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)
# All Rights Reserved.
# Authors: Mohamed Babiker Mohamed Elobaid <mohamed.elobaid@iit.it>
#          Giulio Romualdi <giulio.romualdi@iit.it>

# List the subdirectory
# http://stackoverflow.com/questions/7787823/cmake-how-to-get-the-name-of-all-subdirectories-of-a-directory
macro(SUBDIRLIST result curdir)
  file(GLOB children RELATIVE ${curdir} ${curdir}/*)
  set(dirlist "")
  foreach(child ${children})
    if(IS_DIRECTORY ${curdir}/${child})
      list(APPEND dirlist ${child})
    endif()
  endforeach()
  set(${result} ${dirlist})
endmacro()

# Get list of models
subdirlist(subdirs ${CMAKE_CURRENT_SOURCE_DIR}/robots/)

# Install each model
foreach (dir ${subdirs})
  file(GLOB scripts robots/${dir}/*.ini)
  yarp_install(FILES ${scripts} DESTINATION ${YARP_ROBOTS_INSTALL_DIR}/${dir})
endforeach ()

file(GLOB scripts scripts/*.xml)
yarp_install(FILES ${scripts} DESTINATION ${YARP_APPLICATIONS_INSTALL_DIR})

if (MSVC)
  yarp_install(DIRECTORY virtualizerRetargeting DESTINATION ${YARP_CONTEXTS_INSTALL_DIR})
endif (MSVC)
