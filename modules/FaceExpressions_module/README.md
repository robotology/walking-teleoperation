# **FaceExpressions_module**: 

This module makes opens and close the iCub mouth when it detects a voice in the input audio.



## Installation

The installation has been tested only on Linux. 

You need to install the ``libfvad`` library. On Linux, you can install it as follows:

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



## Example of Usage

- Start the iCub Face expressions ([here](https://github.com/robotology/robots-configuration/blob/0c4724cc530b4febd76c9eccef9b8d1e797a9fa7/iCubGenova09/extra/applications/faceExpressions.xml) an example of application).
- On one terminal launch the device to capture the operator voice

```
yarpdev --device AudioRecorderWrapper --subdevice portaudioRecorder --name /icub-virtualizer/microphone --min_samples_over_network 100 --max_samples_over_network 1000 --AUDIO_BASE::rate 16000 --AUDIO_BASE::samples 4000 --AUDIO_BASE::channels 1 --start
```

- Run the module with the command ``FaceExpressionsRetargetingModule``

- Connect the relevant ports

  ```
  yarp connect /icub-virtualizer/microphone/audio:o /face_expressions_from_mic/in
  yarp connect /face_expressions_from_mic/emotions:o /icub/face/emotions/in
  ```

  