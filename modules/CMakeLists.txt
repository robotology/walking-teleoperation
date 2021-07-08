# Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)
# All Rights Reserved.
# Authors: Giulio Romualdi <giulio.romualdi@iit.it>

add_subdirectory(Utils)
add_subdirectory(Oculus_module)

if(WALKING_TELEOPERATION_COMPILE_XsensModule)
  add_subdirectory(Xsens_module)
endif (WALKING_TELEOPERATION_COMPILE_XsensModule)

if(WALKING_TELEOPERATION_COMPILE_VirtualizerModule)
  add_subdirectory(Virtualizer_module)
endif()

if(WALKING_TELEOPERATION_COMPILE_FaceExpressionsRetargetingModule)
    add_subdirectory(FaceExpressions_module)
endif()

if(WALKING_TELEOPERATION_COMPILE_SRanipalModule)
    add_subdirectory(SRanipal_module)
endif()

