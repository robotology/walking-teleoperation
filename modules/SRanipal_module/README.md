# SRanipal_module

The `SRanipal_module` commands the robot face expressions, eyelids, and gaze mimicking the operator's expressions. It exploits the (VIVE Eye and Facial Tracking SDK)[https://developer.vive.com/resources/vive-sense/sdk/vive-eye-and-facial-tracking-sdk/], called ``SRanipal``. This is available only on Windows.

## Additional dependencies
The `SRanipal_module`, besides the repo main dependencies, requires the `SRanipal_SDK` to be downloaded. Download it from the VIVE website at https://developer.vive.com/resources/vive-sense/sdk/vive-eye-and-facial-tracking-sdk/. It requires (free) registration. Once downloaded, simply uncompress it and set the environmental variable ``SRanipal_SDK_DIR`` to the ``01_C`` folder of the uncompressed archive, e.g. ``C:\pathWhereYouDownloadedSranipal\SDK\01_C``. You can also avoid setting this environmental variable, and specify the path from the CMake GUI in the ``SRanipal_SDK_DIR`` variable.

The module has been tested with the version 1.3.3.0 of the SDK.

The module needs to run where it is connected the HTC VIVE headset sporting the facial tracker. The headset related software should be properly installed and running. In particular, it is necessary that the ``SRanipalRuntime`` should be installed and running in background.
A small robot face should be present in the tray. Here the description of the icon: 
![image](https://user-images.githubusercontent.com/18591940/127194127-16dd1cf4-08ba-4f83-b34b-31c15ddd28bb.png)

![image](https://user-images.githubusercontent.com/18591940/127194179-d7ceb6be-9020-4751-91f4-dc1de079e4df.png)

The ``SRanipalRuntime`` can be installed with the ``VIVE_SRanipalInstaller`` installer downloadable from https://developer.vive.com/resources/vive-sense/sdk/vive-eye-and-facial-tracking-sdk/ as well.

## Running the module

The module attempts at commanding the robot face expressions communicating with the iCub [face expressions module](https://robotology.github.io/robotology-documentation/doc/html/group__icub__faceExpressions.html). In addition, it connects to the robot head motor control board to control the robot gaze.

It also commands the eyelids opening. In this case, the output is different in case the robot is sporting a RFE-based face (hence with a DC motor controlling the eyelids, like ``iCubGenova09``), or an older face using servomotors (like ``iCubGenova04``).
### RFE-based face
https://user-images.githubusercontent.com/18591940/122098153-c089ff80-ce10-11eb-9ad9-16d0f64d22e0.mp4

With an RFE face, the module can start without any additional parameter.

At startup, the module can request the user to calibrate the eye-tracking headset. Simply follow the instructions on the headset. Once the calibration is over, the module starts normally. 

The module tries to automatically connect to the eyelids joint in the ``face`` controlboard. If the connections fails, the module exits.

If the module started correctly, you should see a message like ``SRanipalModule started correctly``. 

#### Ports to connect
In order to control the robot face expressions, it is necessary to connect some ports after the module started
- ``/SRanipalModule/emotions:o`` to the ``emotionInterface`` input port (by default it should be ``/icub/face/emotions/in``). **Note**, the name of the port may change according to the ``name`` and ``emotionsOutputPortName`` input parameters.
- (Optional) It is possible to visualize the lip camera by connecting the port ``/SRanipalModule/lipImage:o`` to a ``yarpview`` input image. **Note**, the name of the port may change according to the ``name`` and ``lipImagePortName`` input parameters.

### Servomotors-based face

https://user-images.githubusercontent.com/18591940/123784082-5d4fa100-d8d7-11eb-979d-98d5f4858a53.mp4

Older versions of the iCub face sported a servomotor to control the eyelids opening, and it did not have an associated controlboard to be controlled. Hence, in order to control the eyelids opening, it is necessary to connect directly to the input port of the serial device communicating with the face expressions board.

Start the module appending ``--useRawEyelids`` to the command.

At startup, the module can request the user to calibrate the eye-tracking headset. Simply follow the instructions on the headset. Once the calibration is over, the module starts normally. 

If the module started correctly, you should see a message like ``SRanipalModule started correctly``. 
#### Ports to connect
In order to control the robot face expressions, it is necessary to connect some ports after the module started
- ``/SRanipalModule/emotions:o`` to the ``emotionInterface`` input port (by default it should be ``/icub/face/emotions/in``).  **Note**, the name of the port may change according to the ``name`` and ``emotionsOutputPortName`` input parameters.
- ``/SRanipalModule/face/raw:o`` to the input port of the serial device controlling the face expressions (by default it should be ``/icub/face/raw/in``). **Note**, the name of the port may change according to the ``name`` and ``rawEyelidsPortName`` input parameters.
- (Optional) It is possible to visualize the lip camera by connecting the port ``/SRanipalModule/lipImage:o`` to a ``yarpview`` input image. **Note**, the name of the port may change according to the ``name`` and ``lipImagePortName`` input parameters.

## Controlled face expressions
- The robot eyelids are finely and directly controlled by the operator's eyelids.
- When the operator opens the eyes wide open, the robot shows a surprised expression on the eyebrows.
- When the operator opens the mouth, the robot shows a mouth open expression.
- When the operator smiles (mouth in a U shape), the robot uses a smile expression.
- If the operator has a sad expression (mouth in a reversed U shape) or pout, the robot uses a sad face expression.

## Gaze control
https://user-images.githubusercontent.com/18591940/141832971-c6f1dd2c-d5a1-4fc3-b428-7130db2074c7.mp4

In order to control the robot gaze, the module needs to connect to the robot head motor control board. In addition, the module needs to connect to the VR application displaying the images coming from the robot. This is necessary for two reasons:

- the module needs to compute where the operator is looking at;
- the images in the VR have to move according to the robot eyes.

For these reasons, this functionality is available only when using the [``yarp-device-openxrheadset``](https://github.com/ami-iit/yarp-device-openxrheadset).

#### Ports to connect
- ``/SRanipalModule/VR/rpc:o`` (**note**, the name of the port may change according to the ``name`` and ``VRDeviceRPCOutputPortName`` input parameters) to ``/OpenXrHeadset/rpc`` (**note**, the name of the port may change according to the configuration of the ``yarp-device-openxrheadset`` device).
This port needs to be connected in order to use the ``AdvancedJoypad`` features.

## Optional parameters
The following optional parameters can be inserted after calling with module, each of them preceded by a ``--``, e.g. ``--parameterName value``.

### General settings
- ``name``,  prefix used to open the ports. Default ``SRanipalModule``.
- ``robot``, prefix used to connect to the robot in the RFE case. Default ``icub``.
- ``noEyebrows``, if specified without value (i.e. ``--noEyebrows``) or with value ``true`` (i.e. ``--noEyebrows true``), the module avoids to set the eyebrows expressions. By default it is ``false``.
- ``noEyelids``, if specified without value (i.e. ``--noEyelids``) or with value ``true`` (i.e. ``--noEyelids true``), the module avoids to control the eyelids. By default it is ``false``.
- ``noLip``, if specified without value (i.e. ``--noLip``) or with value ``true`` (i.e. ``--noLip true``), the module avoids to use the lip tracking device. By default it is ``false``.
- ``noGaze``, if specified without value (i.e. ``--noGaze``) or with value ``true`` (i.e. ``--noGaze true``), the module avoids to use the gaze tracking. By default it is ``false``.
- ``period``, the update period of the module in seconds. Default ``0.1`` (or ``0.01`` if the eyelids are controlled using velocity control, or the gaze control is active).
- ``skipEyeCalibration``, if set without value (i.e. ``--skipEyeCalibration``), or with value ``true`` (i.e. ``--skipEyeCalibration true``), it avoids starting the eye calibration procedure on the headset. Default ``false``.
- ``forceEyeCalibration``, if set without value (i.e. ``--forceEyeCalibration``), or with value ``true`` (i.e. ``--forceEyeCalibration true``), it forces the eye calibration procedure on the headset at startup. Default ``false``.

### Eyelids retargeting
- ``eyeOpenPrecision``, it determines the minimum variation of the measured eye openness of the operator (in the range [0, 1]), to command a variation in the robot eyelids. ``eyeOpenPrecision`` needs to be greater than 0. The minimum input value to trigger a motion is equal to 0.5 * ``eyeOpenPrecision``. Hence, if you set ``eyeOpenPrecision`` equal to 1.0, the eyelids will fully close as soon as the operator closes the eyes at half. This can be useful to make the robot eyelids close when the operator close the eyes more than a given threshold. Default ``0.1``.
- ``useEyelidsPositionControl``, if set without value (i.e. ``--useEyelidsPositionControl``), or with value ``true`` (i.e. ``--useEyelidsPositionControl true``), the module will control the eyelids using position control instead of velocity. This has an effect only if ``useRawEyelids`` is not set, or set to ``false``. Default ``false``.
- ``eyelidsMaxVelocity``, the maximum velocity used when controlling the eyelids. Default ``100`` (``75.0`` if using position control).
- ``eyelidsVelocityGain``, the gain used when controlling the eyelids velocity. Default ``10.0``.
- ``useRawEyelids``,  if set without value (i.e. ``--useRawEyelids``), or with value ``true`` (i.e. ``--useRawEyelids true``), it avoids connecting to the robot face remote control board to control the eyelids. This is needed in case the robot face uses servomotors to control the eyelids. Default ``false``.
- ``rawEyelidsCloseValue``, the raw value for which the eyelids are completely closed in the servomotor case. This value can be found by connecting to the ``/icub/face/raw/in`` port and sending commands like ``S50`` (note the capital S in front of the number). Default ``35``, found on ``iCubGenova04``.
- ``rawEyelidsOpenValue``, the raw value for which the eyelids are completely opened in the servomotor case. This value can be found by connecting to the ``/icub/face/raw/in`` port and sending commands like ``S50`` (note the capital S in front of the number). Default ``60``, found on ``iCubGenova04``.
- ``rawEyelidsPortName``, it sets the suffix for the port controlling the raw face expressions, in particular the eyelids.  Default ``/face/raw:o``.

### Face expression retargeting
- ``emotionsOutputPortName``, suffix used to open the port for communicating with the ``emotionInterface``. Default ``/emotions:o``).
- ``lipExpressionThreshold``, minimum value for a blend shape weight to trigger an expression on the robot. The ``SRanipal`` SDK provides a set of weights in the range [0, 1] classifying the current operator's expression. This parameters define the minimul value for which an expression is considered active. Default ``0.2``.
- ``eyeWideSurprisedThreshold``, similar to ``lipExpressionThreshold`` it defines the minimum value for the eyes wideness to command the surprised expression on the robot. Default ``0.2``.
- ``lipImagePortName``, it sets the suffix for the port streaming the lip camera image (stereo camera in grayscale).  Default ``/lipImage:o``.

### Gaze retargeting
- ``eyesVersionName``, the name of the version joint. Note, it enough for the actual name of the joint to cointain this parameter. In other words, this parameter is a substring of the name of the joint. Use "none" to avoid using the joint. Default ``eyes_vers``

- ``eyesVergenceName``, the name of the vergence joint. Note, it enough for the actual name of the joint to cointain this parameter. In other words, this parameter is a substring of the name of the joint. Use "none" to avoid using the joint. Default ``eyes_verg``.

- ``eyesTiltName``, the name of the tilt joint. Note, it enough for the actual name of the joint to cointain this parameter. In other words, this parameter is a substring of the name of the joint. Use "none" to avoid using the joint. Default ``eyes_tilt``.

- ``eyeMaxVelocity``, the maximum velocity used to move the eyes. Default ``20.0``.

- ``eyeMaxVergence``, the maximum vergence angle in degrees. The minimum is always zero. Default ``10.0``.

- ``eyeMaxVersion``, the maximum absolute value of the version angle in degrees. Default ``25.0``.

- ``eyeMaxTilt``, the maximum absolute value of the tilt angle in degrees. Default ``30.0``.

- ``eyeKinematicSaturationGain``, it is a numeric gain used for the heuristic that keeps the gaze angles within the specified values. Small values might slow down the gaze when far from the limits. High values might cause vibrations close to the limits. Default ``10.0``.

- ``gazeVelocityGain``, it is a gain determining the reactiveness of the gaze controller when the operator moves the eyes. Default ``2.0``.

- ``gazeDeadzone``, avoids the gaze controller to react to tiny eye motions or noises in the gaze measurement. It represents the radius of a circle around the gaze target point located in the image plane. Default ``0.02``.

- ``gazeDeadzoneActivationOffset``, it is added to ``gazeDeadzone`` to provide a two level thresholding to deactivate the gaze motion. Once the gaze is deactivated because the gaze error is below the ``gazeDeadzone`` , the threshold to reactivate the gaze is equal to  ``gazeDeadzone + gazeDeadzoneActivationOffset``. Default ``0.1``.

- ``gazeDeadzoneMinActivationTime``, it defines a time threshold in order to reactivate the gaze. The gaze needs to be outside the ``gazeDeadzone + gazeDeadzoneActivationOffset``radius for at least a time duration equal to ``gazeDeadzoneMinActivationTime`` in order to reactivate the gaze. This is because while blinking, the measurement of the operator's gaze might have some jump. Hence, we set a default value of ``0.5`` seconds in order to be slightly greater than the average blinking duration.

- ``gazeMovementAccuracyInDeg``, it defines the quantization to apply when moving the images in VR. This avoid tiny motions of the images in the headset if there is some small spike in the encoder reading, or in the input. Default ``0.1``.

- ``headControlBoardName``, the name of the robot head control board. This control board should contain the version, vergence and tilt joints. Default ``head``.

- ``VRDeviceRPCOutputPortName``, the suffix of the port used to connect to the ``yarp-device-openxrheadset`` RPC port. Default ``/VR/rpc:o``.

## Advanced joypad
- ``blinkToDisableGUIs``, the list of GUIs to disable when closing the eyes for a period longer than ``blinkDurationTrigger``. Example of usage ``--blinkToDisableGUIs "(0, 1)"``.

- ``blinkDurationTrigger``, minimum time to keep the eyes closed in order to trigger the disabling of GUIs. Default value: 1.0 (the value is in seconds).

- ``blinkTriggerValue``, value to consider the eyes closed. Default value: 0.1.

## Common issues
- The ``SRanipal_SDK`` sports an useful example to test the face detection system without running this module. In particular, you can run the ``FaceGym.exe`` application in the ``SDK/02_Unity/FaceGym`` folder.
- The eye tracking calibration can also be triggered from the head-mounted display (HMD) main menu. When running the calibration, if the message ``Initialization Failed`` appears, you can check https://forum.vive.com/topic/6482-vive-pro-eye-calibration-initialization-error-troubleshooting/. In particular, installing the latest version of the ``SRanipalRuntime`` from https://developer.vive.com/resources/vive-sense/sdk/vive-eye-and-facial-tracking-sdk/ might help.
- In case the operator is wearing thick glasses, the eyes tracking might be affected, causing unexpected behaviors, like the robot constantly closing the eyes.
