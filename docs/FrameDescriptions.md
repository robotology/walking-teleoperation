Following are the descriptions of the Frames used by different devices in our framework:

## Inertial Frame

The inertial frame of the human in our Oculus and Cyberith Virtualizer treadmill teleoperation setup are as following:

<p align="center">
  <img width="300" src="./images/InertialFrame.jpg">
</p>

## Oculus Device
According to the [Oculus documentation](https://developer.oculus.com/documentation/native/pc/dg-sensor/), the frames attached to the Oculus Virtual Reality (OVR) headset are as following.
According to the Oculus documentation:
```
The x-z plane is aligned with the ground regardless of camera orientation.
As seen from the diagram, the coordinate system uses the following axis definitions:
- Y is positive in the up direction.
- X is positive to the right.
- Z is positive heading backwards.
Rotation is maintained as a unit quaternion, but can also be reported in yaw-pitch-roll form. Positive rotation is counter-clockwise.
```
However, the custom they have used for defining the euler angles and their composition order are different from ours.
[Following line of code](https://github.com/robotology/walking-teleoperation/blob/master/modules/Oculus_module/src/OculusModule.cpp#L386-L389
) provides us the rotation from [inertial frame](#xsens-suit) (attached to the human head at the begning) to the oculus headset:

```
iDynTree::toEigen(m_oculusRoot_T_headOculus).block(0, 0, 3, 3)
                = iDynTree::toEigen(iDynTree::Rotation::RPY(-desiredHeadOrientationVector(1),
                                                            desiredHeadOrientationVector(0),
                                                            desiredHeadOrientationVector(2)));
```                                                           

<p align="center">
  <img width="500" height="500" src="./images/Oculus.png">
</p>

## treadmill (Virtualizer Cyberith)

According to the measurement information we get from the cyberith treadmill, we identify the frame of Virtualizer as following:

<p align="center">
  <img width="300" src="./images/TeleoperationFrame.png">
</p>

Threfore the frame we get [in this part of the code:](https://github.com/robotology/walking-teleoperation/blob/60b449e6e8d5120a2a11ca2997521f46c51821c1/modules/Oculus_module/src/HeadRetargeting.cpp#L105)

```
m_oculusInertial_R_teleopFrame = iDynTree::Rotation::RotZ(-playerOrientation);
```
The rotation of the operator (called player here) inside the treadmill is defined with `playerOrientation`, and the rotation of it is expressed in prefered coordinates of the `I_T`, therefore in order to express the rotation in Inertial coordinates (in the code is mentioned by `oculusInertial`), we performam the following transformation
```
 rotz(- playerOrientation)= rotx(pi) * rotz(playerOrientation) * rotx(-pi)
```


# Hand Frames

The Frame with we 

## Frame Chains:


## Xsens Suit
Following are the frames attached to the human body links:


<p align="center">
  <img width="500" height="500" src="./images/humanXses.png">
</p> 

Following are the frames attached to the iCub body links:

<p align="center">
  <img width="500" height="500" src="./images/RobotFrames.png">
</p> 

When we do retargeting using Xsens, each of these frames of the human (where the data are coming) are transformed to the corresponding ones of the robot.

For more information regarding the correspondence of the frames you can look at [this configuration file](https://github.com/robotology/human-dynamics-estimation/blob/devel/conf/xml/RobotStateProvider_iCub.xml).

Regarding the transformation of the frames, you can find them [in this urdf file](), and the tranformation are done from the parent link (each link_name) to the child dummy link specified by `_fake`, e.g., [from `root_link` to `root_link_fake`](https://github.com/robotology/human-dynamics-estimation/blob/devel/conf/urdfs/teleoperation_iCub_model_V_2_5.urdf#L3341-L3353).


