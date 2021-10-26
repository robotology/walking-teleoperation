# SRanipal_module

The `SRanipal_module` commands the robot face expressions mimicking the operator's expressions. It exploits the (VIVE Eye and Facial Tracking SDK)[https://developer.vive.com/resources/vive-sense/sdk/vive-eye-and-facial-tracking-sdk/], called ``SRanipal``. This is available only on Windows.

## Additional dependencies
The `SRanipal_module`, besides the repo main dependencies, requires the `SRanipal_SDK` to be downloaded. Download it from the VIVE website at https://developer.vive.com/resources/vive-sense/sdk/vive-eye-and-facial-tracking-sdk/. It requires (free) registration. Once downloaded, simply uncompress it and set the environmental variable ``SRanipal_SDK_DIR`` to the ``01_C`` folder of the uncompressed archive, e.g. ``C:\pathWhereYouDownloadedSranipal\SDK\01_C``. You can also avoid setting this environmental variable, and specify the path from the CMake GUI in the ``SRanipal_SDK_DIR`` variable.

The module has been tested with the version 1.3.3.0 of the SDK.

The module needs to run where it is connected the HTC VIVE headset sporting the facial tracker. The headset related software should be properly installed and running. In particular, it is necessary that the ``SRanipalRuntime`` should be installed and running in background.
A small robot face should be present in the tray. Here the description of the icon: 
![image](https://user-images.githubusercontent.com/18591940/127194127-16dd1cf4-08ba-4f83-b34b-31c15ddd28bb.png)

![image](https://user-images.githubusercontent.com/18591940/127194179-d7ceb6be-9020-4751-91f4-dc1de079e4df.png)

The ``SRanipalRuntime`` can be installed with the ``VIVE_SRanipalInstaller`` installer downloadable from https://developer.vive.com/resources/vive-sense/sdk/vive-eye-and-facial-tracking-sdk/ as well.

## Running the module

The module attempts at commanding the robot face expressions communicating with the iCub [face expressions module](https://robotology.github.io/robotology-documentation/doc/html/group__icub__faceExpressions.html). It also commands the eyelids opening. In this case, the output is different in case the robot is sporting a RFE-based face (hence with a DC motor controlling the eyelids, like ``iCubGenova09``), or an older face using servomotors (like ``iCubGenova04``).
### RFE-based face
https://user-images.githubusercontent.com/18591940/122098153-c089ff80-ce10-11eb-9ad9-16d0f64d22e0.mp4

With an RFE face, the module can start without any additional parameter.

At startup, the module can request the user to calibrate the eye-tracking headset. Simply follow the instructions on the headset. Once the calibration is over, the module starts normally. 

The module tries to automatically connect to the eyelids joint in the ``face`` controlboard. If the connections fails, the module exits.

If the module started correctly, you should see a message like ``SRanipalModule started correctly``. 

#### Ports to connect
In order to control the robot face expressions, it is necessary to connect some ports after the module started
- ``/SRanipalModule/emotions:o`` to the ``emotionInterface`` input port (by default it should be ``/icub/face/emotions/in``). **Note**, the name of the port may change according to the corresponding input parameters.
- (Optional) It is possible to visualize the lip camera by connecting the port ``/SRanipalModule/lipImage:o`` to a ``yarpview`` input image.

### Servomotors-based face

https://user-images.githubusercontent.com/18591940/123784082-5d4fa100-d8d7-11eb-979d-98d5f4858a53.mp4

Older versions of the iCub face sported a servomotor to control the eyelids opening, and it did not have an associated controlboard to be controlled. Hence, in order to control the eyelids opening, it is necessary to connect directly to the input port of the serial device communicating with the face expressions board.

Start the module appending ``--useRawEyelids`` to the command.

At startup, the module can request the user to calibrate the eye-tracking headset. Simply follow the instructions on the headset. Once the calibration is over, the module starts normally. 

If the module started correctly, you should see a message like ``SRanipalModule started correctly``. 
#### Ports to connect
In order to control the robot face expressions, it is necessary to connect some ports after the module started
- ``/SRanipalModule/emotions:o`` to the ``emotionInterface`` input port (by default it should be ``/icub/face/emotions/in``). **Note**, the name of the port may change according to the ``name`` input parameters.
- ``/SRanipalModule/face/raw:o`` to the input port of the serial device controlling the face expressions (by default it should be ``/icub/face/raw/in``).
- (Optional) It is possible to visualize the lip camera by connecting the port ``/SRanipalModule/lipImage:o`` to a ``yarpview`` input image.

## Controlled face expressions
- The robot eyelids are finely and directly controlled by the operator's eyelids.
- When the operator opens the eyes wide open, the robot shows a surprised expression on the eyebrows.
- When the operator opens the mouth, the robot shows a mouth open expression.
- When the operator smiles (mouth in a U shape), the robot uses a smile expression.
- If the operator has a sad expression (mouth in a reversed U shape) or pout, the robot uses a sad face expression.

## Optional parameters
The following optional parameters can be inserted after calling with module, each of them preceded by a ``--``, e.g. ``--parameterName value``.

- ``name``,  prefix used to open the ports. Default ``SRanipalModule``.
- ``robot``, prefix used to connect to the robot in the RFE case. Default ``icub``.
- ``emotionsOutputPortName``, suffix used to open the port for communicating with the ``emotionInterface``. Default ``/emotions:o``).
- ``noEye``, if specified without value (i.e. ``--noEye``) or with value ``true`` (i.e. ``--noEye true``), the module avoids to use the eye tracking device. By default it is ``false``.
- ``noEyelids``, if specified without value (i.e. ``--noEyelids``) or with value ``true`` (i.e. ``--noEyelids true``), the module avoids to control the eyelids. By default it is ``false``.
- ``noLip``, if specified without value (i.e. ``--noLip``) or with value ``true`` (i.e. ``--noLip true``), the module avoids to use the lip tracking device. By default it is ``false``.
- ``period``, the update period of the module in seconds. Default ``0.1``.
- ``lipExpressionThreshold``, minimum value for a blend shape weight to trigger an expression on the robot. The ``SRanipal`` SDK provides a set of weights in the range [0, 1] classifying the current operator's expression. This parameters define the minimul value for which an expression is considered active. Default ``0.2``.
- ``eyeWideSurprisedThreshold``, similar to ``lipExpressionThreshold`` it defines the minimum value for the eyes wideness to command the surprised expression on the robot. Default ``0.2``.
- ``eyeOpenPrecision``, it determines the minimum variation of the measured eye openness of the operator (in the range [0, 1]), to command a variation in the robot eye lids. Default ``0.1``.
- ``skipEyeCalibration``, if set without value (i.e. ``--skipEyeCalibration``), or with value ``true`` (i.e. ``--skipEyeCalibration true``), it avoids starting the eye calibration procedure on the headset. Default ``false``.
- ``forceEyeCalibration``, if set without value (i.e. ``--forceEyeCalibration``), or with value ``true`` (i.e. ``--forceEyeCalibration true``), it forces the eye calibration procedure on the headset at startup. Default ``false``.
- ``useEyelidsPositionControl``, if set without value (i.e. ``--useEyelidsPositionControl``), or with value ``true`` (i.e. ``--useEyelidsPositionControl true``), the module will control the eyelids using position control instead of velocity. This has an effect only if ``useRawEyelids`` is not set, or set to ``false``. Default ``false``.
- ``useRawEyelids``,  if set without value (i.e. ``--useRawEyelids``), or with value ``true`` (i.e. ``--useRawEyelids true``), it avoids connecting to the robot face remote control board to control the eyelids. This is needed in case the robot face uses servomotors to control the eyelids. Default ``false``.
- ``rawEyelidsCloseValue``, the raw value for which the eyelids are completely closed in the servomotor case. This value can be found by connecting to the ``/icub/face/raw/in`` port and sending commands like ``S50`` (note the capital S in front of the number). Default ``35``, found on ``iCubGenova04``.
- ``rawEyelidsOpenValue``, the raw value for which the eyelids are completely opened in the servomotor case. This value can be found by connecting to the ``/icub/face/raw/in`` port and sending commands like ``S50`` (note the capital S in front of the number). Default ``60``, found on ``iCubGenova04``.
- ``eyelidsMaxVelocity``, the maximum velocity used when controlling the eyelids. Default ``75.0``.
- ``eyelidsVelocityGain``, the gain used when controlling the eyelids velocity. Default ``10.0``.
- ``lipImagePortName``, it sets the suffix for the port streaming the lip camera image (stereo camera in grayscale).  Default ``/lipImage:o``.

## Common issues
- The ``SRanipal_SDK`` sports an useful example to test the face detection system without running this module. In particular, you can run the ``FaceGym.exe`` application in the ``SDK/02_Unity/FaceGym`` folder.
- The eye tracking calibration can also be triggered from the head-mounted display (HMD) main menu. When running the calibration, if the message ``Initialization Failed`` appears, you can check https://forum.vive.com/topic/6482-vive-pro-eye-calibration-initialization-error-troubleshooting/. In particular, installing the latest version of the ``SRanipalRuntime`` from https://developer.vive.com/resources/vive-sense/sdk/vive-eye-and-facial-tracking-sdk/ might help.
- In case the operator is wearing thick glasses, the eyes tracking might be affected, causing unexpected behaviors, like the robot constantly closing the eyes.
