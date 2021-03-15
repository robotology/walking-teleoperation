# Dependencies
 1. **robotology superbuild [optional]:** Install the robotology superbuild on a Windows machine as described here [Windows installation notes](https://github.com/robotology/robotology-superbuild#windows):

    1. opencv: Currently there are open issues dicussed in [#145](https://github.com/robotology/robotology-superbuild/issues/145) related to the opencv package installation from robotlogy superbuild. Instead, install it from the [opencv website](https://opencv.org/releases.html), version 3.4.4;

   + if you don't want to use superbuild for installation, skip it and go directly to second dependency (YARP)
2. **YARP:** Using superbuild install the [YARP](http://www.yarp.it/): to handle the comunication with the robot with both ovrheadset and SDLjoypad drivers;

   + if you are not using superbuild, look at [this link](http://www.yarp.it/) for installing YARP.
   + Use YARP and icub repos using the devel branch instead of the master branch. The main reason is because of the `camCalibWithPose` application.
   + Enable the following options (be sure that this option is enabled in all the machines for the image communication):
   ```
   ENABLE_yarpcar_mjpeg
   MJPEG_AUTOCOMPRESS
   ```

3. **QT5 and Eigen3:** In the Super-build, enable the QT5 and Eigen3 to install (follow the instructions [here](https://github.com/robotology/robotology-superbuild#system-libraries)).

4. **Oculus SDK:** In order to install the oculus SDK, you need to install first the Oculus app on windows (oculus application), install the dependencies (GLEW, GLFW3), and finally install the sdk (LibOVR):

   1. `oculus application`: Download and install the oculus [setup application](https://www.oculus.com/setup/). It is used for both the Oculus Virtual Reality headset (ovrheadset) and joypads (touch controller).
          
   2. `GLEW`: Download the version 2.1.0 of the [glew library](http://glew.sourceforge.net/index.html) from this [link](https://sourceforge.net/projects/glew/files/glew/2.1.0/) (if you want to download from the [glew library](http://glew.sourceforge.net/index.html) select the source file)
     
      - extract and copy the glew library to your robot/code workspace (in our case in same path of robotology-superbuild)
      
      - the library has the makefiles, so go to the `glew\glew-2.1.0\glew-2.1.0\build\vc12` path and using the VS15 (VS 2017) build the library (release and debug!)
      
      - Add the following variables value in robotology/yarp using CMake GUI (search for glew!)
        ```
        GLEW_INCLUDE_DIR: <path to codes workspace>/glew/glew-2.1.0/include
               (inside this folder you should find the GL folder and inside that the header files)
          
        GLEW_SHARED_LIBRARY_DEBUG:   <path to codes workspace>/glew/glew-2.1.0/lib/Debug/x64/glew32d.lib
              
        GLEW_SHARED_LIBRARY_RELEASE: <path to codes workspace>/glew/glew-2.1.0/lib/Release/x64/glew32.lib
        
        GLEW_STATIC_LIBRARY_DEBUG:   <path to codes workspace>/glew/glew-2.1.0/lib/Debug/x64/glew32sd.lib
              
        GLEW_STATIC_LIBRARY_RELEASE: <path to codes workspace>/glew/glew-2.1.0/lib/Release/x64/glew32s.lib
              
        YARP_USE_GLEW: check the box
        ```
      - Append the following directories to the User environmental variable, for example using the [Rapid Environment Editor](https://www.rapidee.com):
        ```
        Path: (Expandable string)
                    <path to codes workspace>\glew\glew-2.1.0\bin\Release\x64
                    <path to codes workspace>\glew\glew-2.1.0\bin\Debug\x64
                    <path to codes workspace>\glew\glew-2.1.0\lib\Release
                    
        CMAKE_PREFIX_PATH: (Expandable string)
                    <path to codes workspace>\glew\glew-2.1.0\include
                    <path to codes workspace>\glew\glew-2.1.0\lib

        GLEW_DIR: (Expandable string)
                    <path to codes workspace>\glew\glew-2.1.0\
        ```  
  
      - Don't forget to configure, generate the cmake (cmake gui --> robotology yarp) and build release mode the yarp using vs15.  
      
      - if you have problems to install [git repo](https://github.com/nigels-com/glew) may help you. 
     
     
   3. `GLFW3`: To download this library go to [website1](https://www.glfw.org/) or [website2](https://www.glfw.org/download.html). Download the "Source package" among the available ones, the version of it is "3.2.1".
     

      - Place the package in your workspace (in our case in same path of robotology-superbuild)
          

      - Use CMake GUI to to configure and generate the project.
          

      - After that, using the VS15 (VS 2017) build the project.
      
      
      - Add the following value to the `CMAKE_INSTALL_PREFIX` of the glfw project
      ```
      CMAKE_INSTALL_PREFIX: <path to codes workspace>/glfw/glfw-3.2.1/build/install
      ```
      - After builing the project, install it as well.

      - Add the following paths to the variables of robotology/YARP (cmake gui --> robotology yarp); as the yarpdev which gets the data from the sdk of the ovr and publishes in yarp framework, has dependencies on this library:
      
      ```
      GLFW3_DIR: <path to codes workspace>/glfw/glfw-3.2.1/glfw-3.2.1/build/install/lib/cmake/glfw3 (inside this folder you should find the cmake files)
 
      GLFW3_GLFW_LIBRARY: <path to codes workspace>/glfw/glfw-3.2.1/glfw-3.2.1/build/install/lib/cmake/glfw3

      GLFW3_INCLUDE_DIR: <path to codes workspace>/glfw/glfw-3.2.1/glfw-3.2.1/build/install/include` (inside that you should find the GLFW folder, and inside it the header files)
              
      GLFW3_OPENGL_DIR: <path to codes workspace>/glfw/glfw-3.2.1/glfw-3.2.1/build/install/lib` (you should be able to find the glfw.lib file)
              
      YARP_USE_GLFW3: check the box
      ```
      - Add the following varibale to the User environmental variable:
      ```       
      GLFW3_DIR= <path to codes workspace>\glfw\glfw-3.2.1\glfw-3.2.1\build\install (variable type is String)
      ```    
      - Don't forget to configure, generate the cmake (cmake gui --> robotology yarp) and build release mode the yarp using vs15.  
             
      - if you have problems with compiling GLFW3, [this link](https://www.glfw.org/docs/latest/compile_guide.html#compile_generate) or [here](https://github.com/nigels-com/glew) may help.
          
          
    4. `LibOVR`: to dowload the SDK, you can go to [this website](https://developer.oculus.com/downloads/), choose "Native Windows", then "Core Package: OCULUS SDK for windows". Select the version 1.40.0 to download. Or easily follow this [link](https://developer.oculus.com/downloads/package/oculus-sdk-for-windows/1.40.0/) and choose version 1.40.0 to download.
    
      - Extract and place the package in your workspace (in our case in same path of robotology-superbuild), inside that there are two libraries which we need: LibOVR, and LibOVRKernel.
      
      - Build (both release and debug) the projects inside the sdk using VS15: 
        ```
        \LibOVR\Projects\Windows\VS2015\LibOVR.vcxproj
        
        \LibOVRKernel\Projects\Windows\VS2015\LibOVRKernel.vcxproj
        ```
        Before building the projects change the follwing options using VS:
        ```
        LibOVR or LibOVRKernel projects -> properties -> C/C++ -> Code Generation:
         
        Configuration: Debug --> Runtime Library: Multi-threaded Debug DLL(/MDd);
         
        Configuration: Release --> Runtime Library: Multi-threaded DLL(/MD)
        ```
        After these changes, build the projects.
        
      - Add the following paths to the variables of robotology/YARP (because the yarpdev which gets the data from the sdk of the ovr and publishes in yarp framework, has dependencies on this library)
          
        ```  
        LibOVR_LibOVRKernel_INCLUDE_DIR:<path to codes workspace>/OculusSDK/LibOVRKernel/Src
              
        LibOVR_LibOVRKernel_LIBRARY_DEBUG:<path to codes workspace>/OculusSDK/LibOVRKernel/Lib/Windows/x64/Debug/VS2015/LibOVRKernel.lib

        LibOVR_LibOVRKernel_LIBRARY_RELEASE: <path to codes workspace>/OculusSDK/LibOVRKernel/Lib/Windows/x64/Release/VS2015/LibOVRKernel.lib

        LibOVR_LibOVR_Extras_INCLUDE_DIR: <path to codes workspace>/OculusSDK/LibOVR/Include/Extras

        LibOVR_LibOVR_INCLUDE_DIR: <path to codes workspace>/OculusSDK/LibOVR/Include

        LibOVR_LibOVR_LIBRARY_DEBUG: <path to codes workspace>/OculusSDK/LibOVR/Lib/Windows/x64/Debug/VS2015/LibOVR.lib

        LibOVR_LibOVR_LIBRARY_RELEASE: <path to codes workspace>/OculusSDK/LibOVR/Lib/Windows/x64/Release/VS2015/LibOVR.lib

        YARP_USE_LOBOVR: check the box
        ENABLE_yarpmod_ovrheadset: check the box
        ```
        Before building the projects change the follwing options using VS:
        ```
        Yarp project -> Plugins -> Devices -> yarp_ovrheadset :: properties -> C/C++ -> Code Generation:
         
        Configuration: Debug --> Runtime Library: Multi-threaded Debug DLL(/MDd);
         
        Configuration: Release --> Runtime Library: Multi-threaded DLL(/MD)
        ```

      - Add and append the following variables to the User Environmental Variable:
        ```     
        OculusSDK_ROOT: <path to codes workspace>\OculusSDK (variable type is String)
        Path: <path to codes workspace>\OculusSDK\3rdParty\Windows Kits\8.1\Redist\D3D\x64
        ```      

      - Configure, generate the cmake (cmake gui --> robotology yarp) and build the yarp using vs15.

5. **Cyberith SDK:**   To allow the virtualizer module to capture the operator data, the virtualizer needs two modules to download: the CybSDK_app (used for the calibration) and the CybSDK:

   1. `CybSDK_app`: you can download the application from [this link](https://developer.cyberith.com/downloads). Go to `Tools` tab, download the `Virtualizer Control Panel`. When you start the virtualizer application, remember to calibrate it at the begining. After finishing the checks through the virtualizer application, please disconnect it, so that you can use the SDK in your teleoperation application.
     
   2. `CybSDK`: you can download the application from [this link](https://developer.cyberith.com/downloads). Go to `SDK` tab, download the `C++ SDK for Windows (Developed for: Visual Studio 2015 Community)`. After Downloading the SDK, append the following variables to the user environmental variable:
       ```
       CybSDK_DIR= <path to the root of the cyberith sdk directory>
       Path=Path; <path to the root of the cyberith sdk directory>/x64
       ```
       
6. **FaceExpressions_module**: To allow the FaceExpressions_module detecting the human voice, you need to install the ``libfvad`` library. On Linux, you can install it as follows:
   ```sh
   sudo apt install autoconf libtool pkg-config #libfvad dependencies
   git clone https://github.com/dpirch/libfvad
   cd libfvad
   autoreconf -i
   mkdir build
   cd build
   ../configure --prefix=/path/to/install 
   make
   [sudo] make install
   ```
   You can avoid setting ``--prefix=/path/to/install``. In this case, it will install it in the ``/usr/local`` folder and the command ``make install`` will need ``sudo``.
   In case you specified the prefix instead, in order to find this installation, you need to add the following to your ``.bashrc``:
   ```bash
   export PKG_CONFIG_PATH=$PKG_CONFIG_PATH:/path/to/install/lib/pkgconfig/
   ```
