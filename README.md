# walking-teleoperation
Software related to walking and teleoperation. 

The suite includes:

* **Oculus_module**: this is the module that implements retargeting of the upper body end_effectors.
* **Virtualizer_module**: this module allows using the cyberith virtualizer as a joypad interface for walking commands.
* **Utils_module**: an module that can be useful to implement some common functionality

The technical description of the suit is described [here](./docs/FrameDescriptions.md).

# Overview
 - [:orange_book: The general idea](#orange_book-some-theory-behind-the-code)
 - [:page_facing_up: Dependencies](#page_facing_up-dependencies)
 - [:hammer: Build the suite](#hammer-build-the-suite)
 - [:running: Using the software with iCub](#running-using-the-software-with-iCub)

# :orange_book: The general idea
This software allows teleoperation of a walking humanoid robot with a walking controller that expects positions on the plane as walking direction commands for the planner.
It implements the following architecture:
* Oculus module that captures the end effectors of the hands and head of the human operator and commands the respective movement;
* Virtualizer module [Optional] that graps the human walking teleoperation commands (orientation and Speed).


# :page_facing_up: Dependencies
The description of dependencies are located [here](./docs/Dependencies.md).

This guide is only for teleoperation dependencies on a Windows machine and it includes the guide to install Oculus module and Virtualizer module SDKs.

Besides, you need to have a linux machine for **Walking-controllers** module described [here](https://github.com/robotology/walking-controllers/tree/devel_hand_retargeting).

# :hammer: Build the suite
## Linux/macOs

```sh
git clone https://github.com/robotology/walking-teleoperation.git
cd walking-controllers
mkdir build && cd build
cmake ../
make
[sudo] make install
```
## Windows
Follow the same instructions from the Powershell. One can also opt to use the ``CMake`` gui application.

# :running: Using the software with iCub
Import the `DCM_WALKING_COORDINATOR_+_RETARGETING` to the `yarpmanager` applications.
The current set-up allows to run the module either on windows, or from a linux machine through `yarprun --server /name_of_server`. The preference is the following.
* Turn on the robot, through the linux machine.
* On the windows machine, use the same network.
* Do a `yarp namespace /the_robot_network_namespace`
* Do a `yarprun --server /icub-virtualizer`
* Calibrate the virtualizer and the oculus
* At this point, the operator should be in the virtualizer wearing the oculus and in the zero configuration, i.e. zero orientation in the virtualizer, facing the same direction as the robot and standing still.
* On the linux server, and from the `yarpmanager` run the application `DCM_WALKING_COORDINATOR_+_RETARGETING`
* On the same application window, connect all the ports.
* On the windows machine, adjust the image size and positioning (field of view) of the Oculus (to zoom out press ctrl+z, to move the right display use right ctrl+direction, to move the left display use left ctrl+direction ).
* On the linux machine to adjust the image quality, use the `frameGrapperGui` in the `calib_cams` application.

## :warning: Warning
Currently the supported robots are only:
- ``iCubGenova04``
- ``iCubGenova02``


