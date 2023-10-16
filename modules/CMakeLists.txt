# SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-3-Clause

add_subdirectory(Utils)
add_subdirectory(Oculus_module)
add_subdirectory(OpenXRJoypad_module)

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

if(WALKING_TELEOPERATION_COMPILE_HapticGloveModule)
    add_subdirectory(HapticGlove_module)
endif()


