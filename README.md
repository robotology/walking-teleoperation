# walking-teleoperation
Software related to walking and teleoperation. 

The suite includes:

* **Oculus_module**: this is the module that implements retargeting of the upper body end_effectors.
* **Virtualizer_module**: this module allows using the cyberith virtualizer as a joypad interface for walking commands.
* **Utils_module**: an module that can be useful to implement some common functionality

# Overview
 - [:orange_book: The general idea](#orange_book-some-theory-behind-the-code)
 - [:page_facing_up: Dependencies](#page_facing_up-dependencies)
 - [:hammer: Build the suite](#hammer-build-the-suite)
 - [:computer: How to run the simulation](#computer-how-to-run-the-simulation)
 - [:running: How to test on iCub](#running-how-to-test-on-icub)

# :orange_book: The general idea
This software allows teleoperation of a walking humanoid robot with a walking controller that expects positions on the plane as walking direction commands for the planner.
It implements the following architecture:
* Oculus module that captures the end effectors of the hands and head of the human operator and commands the respective movement;
* Virtualizer module [Optional] that graps the human walking teleoperation commands (orientation and Speed).


# :page_facing_up: Dependencies
* [YARP](http://www.yarp.it/): to handle the comunication with the robot with both ovrheadset and SDLjoypad drivers;
* [Walking-controllers](https://github.com/robotology/walking-controllers) to handl the hand cartesian task and walking commands
* [Oculus SDK](https://developer.oculus.com/downloads/package/oculus-sdk-for-windows/): The native Windows SDK.
* [Cyberith SDK](https://www.cyberith.com/research-development/): To allow the virtualizer module to capture the operator data.

# :hammer: Build the suite
## Linux/macOs

```sh
git clone https://github.com/robotology/walking-controllers.git
cd walking-controllers
mkdir build && cd build
cmake ../
make
[sudo] make install
```
## Windows
Follow the same instructions from the Powershell. One can also opt to use the ``CMake`` gui application.

## :warning: Warning
Currently the supported robots are only:
- ``iCubGenova04``
- ``iCubGenova02``


