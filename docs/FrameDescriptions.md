Following are the descriptions of the Frames used by different devices in our framework:

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

**Virtualizer (treadmill)**

<p align="center">
  <img width="600" height="300" src="./images/Virtualizer%2Bothers.jpg">
</p>

**Xsens Suit**

<p align="center">
  <img width="500" height="500" src="./images/humanXses.png">
</p> | <p align="center">
  <img width="500" height="500" src="./images/RobotFrames.png">
</p> 
