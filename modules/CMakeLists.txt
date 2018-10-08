# Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)
# All Rights Reserved.
# Authors: Giulio Romualdi <giulio.romualdi@iit.it>

add_subdirectory(Utils)
add_subdirectory(Oculus_module)
if (MSVC)
  add_subdirectory(Virtualizer_module)
endif (MSVC)
