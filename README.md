# walking-teleoperation

This repository contains software related to walking and teleoperation. 

The whole-body teleoperation architecture is shown as follows.
The human user receives visual feedback from the robot environment by streaming the robot camera images through the _Oculus Headset_.
The robot hands are controlled via the _Joypads_.
The linear and the angular velocities of the human are obtained from the _Cyberith Virtualizer VR Treadmill_. 
The operator can wear a sensorized full body suit to obtain the kinematic information of various human links with respect to the inertial frame of reference. 


![teleoperationArchitecture](https://user-images.githubusercontent.com/17707730/75995333-8d15b100-5efc-11ea-8a40-cea64bf36bf8.jpg)


Software related to walking and teleoperation. 

The suite includes:

* **Oculus_module**: this is the module that implements retargeting of the upper body end_effectors.
* **Virtualizer_module**: this module allows using the Cyberith virtualizer as a joypad interface for walking commands.
* **Utils_module**: a module that can be useful to implement some common functionality
* **Xsens_module**: a module that gets joint values from [human state provider](https://github.com/robotology/human-dynamics-estimation/) and maps them to the [walking controller](https://github.com/robotology/walking-controllers) input
* **FaceExpressions_module**: a module that detects a human voice from a microphone and commands the robot face expressions to mimic the lips moving.
* **SRanipal_module**: a module that controls the robot face expressions directly from the operator's expressions thanks to the [VIVE Eye and Facial Tracking SDK](https://developer.vive.com/resources/vive-sense/sdk/vive-eye-and-facial-tracking-sdk/).

The technical description of the suit and the frame descriptions are documented [here](./docs/FrameDescriptions.md).

# Overview
 - [:orange_book: The general idea](#orange_book-some-theory-behind-the-code)
 - [:page_facing_up: Dependencies](#page_facing_up-dependencies)
 - [:hammer: Build the suite](#hammer-build-the-suite)
 - [:running: Using the software with iCub](#running-using-the-software-with-iCub)

# :orange_book: The general idea
This software allows teleoperation of a biped humanoid robot,e.g., iCub, with a walking controller that expects speed and orientation of the robot locomotion on a horizontal plane and commands for upper body control of the humanoid robot.
It implements the following architecture:
* Oculus module: that captures the end effectors of the hands and head of the human operator and commands the respective movement (if only using Oculus VR);
* Virtualizer module: [Optional]  grasps the human walking teleoperation commands (orientation and Speed).
* Xsens module: [optional] maps the robot's joints values to the controller

# :page_facing_up: Dependencies
The description of dependencies are located [here](./docs/Dependencies.md).

This guide is only for teleoperation dependencies on a Windows machine and it includes the guide to install Oculus module and Virtualizer module SDKs.

If you want to run the teleoperation scenario with Xsens MVN technologies, you need to enable the option [human-dynamics in robotology/superbuild](https://github.com/robotology/robotology-superbuild#human-dynamics) and install all the dependencies described there.

Besides, you need to have a Linux machine for **Walking-controllers** module described [here](https://github.com/robotology/walking-controllers/tree/devel_hand_retargeting).

# :hammer: Build the suite
## Linux/macOS

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
The current set-up allows running the module either on windows or from a Linux machine through `yarprun --server /name_of_server`. The preference is the following.
* Turn on the robot, through the Linux machine.
* On the windows machine, use the same network.
* Do a `yarp namespace /the_robot_network_namespace`
* Do a `yarprun --server /icub-virtualizer`
* Calibrate the virtualizer and the oculus
* At this point, the operator should be in the virtualizer wearing the oculus and in the zero configuration, i.e. zero orientation in the virtualizer, facing the same direction as the robot and standing still.
* On the Linux server, and from the `yarpmanager` run one of the applications `DCM_walking_retargeting`,`DCM_walking_retargeting (Virtualizer)` ,`DCM_walking_retargeting_(Xsens)`, or `DCM_walking_retargeting_(Virtualizer_Xsens)` depending on the experiment you want to perform.
* On the same application window, connect all the ports.
* On the windows machine, adjust the image size and positioning (field of view) of the Oculus (to zoom out press ctrl+z, to move the right display use right ctrl+direction, to move the left display use left ctrl+direction ).
* On the Linux machine to adjust the image quality, use the `frameGrapperGui` in the `calib_cams` application.


## :warning: Warning
Currently, the supported robots are only:
- ``iCubGenova04``
- ``iCubGenova02``

## :eyeglasses: Reference paper

You can read more about the work [here.](https://arxiv.org/pdf/1909.10080.pdf)
If you're going to use this package for your work, please quote it within any resulting publication:
```
@inproceedings{Whole-Body-2019,
author = {Darvish, Kourosh and Tirupachuri, Yeshasvi and Romualdi, Giulio and Rapetti, Lorenzo and Ferigo, Diego and Chavez, Francisco Javier Andrade and Pucci, Daniele},
 title = {Whole-Body Geometric Retargeting for Humanoid Robots},
booktitle = {Proceedings of the 2019 IEEE/RAS International Conference on Humanoid Robots (Humanoids)},
year = {2019},
month={October},
address = {Toronto, Canada},
}
```
## :dollar: License
The _walking-teleoperation_ repository is licensed under either the GNU Lesser General Public License v3.0 :

https://www.gnu.org/licenses/lgpl-3.0.html

or the GNU Lesser General Public License v2.1 :

https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html

at your option.


**To get the updated information about the dependencies branches and how to run, please check the wiki page of this repository.**

## Mantainers

* Stefano Dafarra ([@S-Dafarra](https://github.com/S-Dafarra))
* Giulio Romualdi ([@GiulioRomualdi](https://github.com/GiulioRomualdi))
