# Xsens BASE: Changing or Resetting the MTi reference co-ordinate systems
Source: https://base.xsens.com/s/article/Changing-or-Resetting-the-MTi-reference-co-ordinate-systems-1605869706643?language=en_US (rendered page text, accessed 2026-09-27)


Title
Changing or Resetting the MTi reference co-ordinate systems
Last Published Date
1/11/2022, 7:01 PM
URL Name
Changing-or-Resetting-the-MTi-reference-co-ordinate-systems-1605869706643
Answer

In some situations it may be that the MT sensor axes are not exactly aligned with the axes of the object of which the motion data has to be tracked. It may be desired to retrieve the 3D motion data in the object frame (O) instead of the default sensor-fixed frame (S) and it may be determine the orientation/velocity data with respect to a different earth-fixed local frame (L’) than the default earth-fixed local frame (L). The transformations are defined by the rotation matrices L’LR and SOR, in software referred to as RotLocal and RotSensor respectively, resulting in the following equations affecting the rotation matrix LSR, the SDI data (S∆q and S∆v), the calibrated data (Ss), and the sensor fusion algorithm output (Lx):

  

Five methods are available to facilitate in obtaining the output in the desired coordinate frames, which are:

Performing an inclination reset, which levels the sensor by defining the O frame. It computes O such that Roll and Pitch become 0 deg (applying SOR only);
Performing a heading reset, which defines the L’ frame by setting the x-axis of L’ frame while maintaining the z-axis along the vertical. It computes L’ such that Yaw becomes 0 deg (also known as "bore sighting")(applying L’LR only);
Performing a combined inclination/heading reset, referred to as alignment reset. It computes L’ and O such that Roll, Pitch and Yaw become 0 deg (applying both SOR and L’LR);
Setting an arbitrary alignment rotation matrix to rotate S to the chosen frame O (applying SOR only);
Setting an arbitrary alignment rotation matrix to rotate L to the chosen frame L’ (applying L’LR only).

 

Arbitrary alignment

If the measured kinematics are required in a frame O and/or a frame L’ with known orientations with respect to standard sensor coordinate frames S and L, the alignment matrices SOR and L’LR can be set directly. This can be useful if for mechanical or spatial reasons the MTi can only be mounted in some specific orientation. The alignment matrices can be set directly in MT Manager's Device Settings window, or through low-level communication using the SetAlignmentRotation command.

 

Common earth-fixed local frame: NWU and NED

The default earth-fixed local frame (L) of the MTi (in Normal output mode) is the East-North-Up (ENU) frame, as described in the MTi Family Reference Manual. The MTi can be easily set to use two other reference coordinate systems. These are NWU (North West Up) and NED (North East Down). 

In order to configure the MTi to provide data in the NWU local frame, apply the following (RotLocal) rotation matrix:

NED is a reference coordinate system often used in aerospace applications. In order to configure the MTi in NED, apply the following (RotLocal) rotation matrix:

 

Orientation resets

The orientation reset functions aim to facilitate in aligning the sensor object it is strapped to, by defining the L’ frame (heading reset) and the O frame (inclination reset) resulting in L’OR, defined in the equations above. The alignment reset is separated in an inclination reset (leveling) and a heading reset (bore sighting). After an alignment reset, the orientation of the L’ and O frames are equal, and the coordinate axes are defined by:

the L’ and O z-axes are the vertical (up, along gravity);
the L’ and O x-axes equal the S x-axis, but projected on the horizontal plane;
the L’ and O y-axes are chosen as to obtain a right handed coordinate frame.

Figure 1 visualizes the different orientation resets, showing a side and top view of each of the resets, as compared to the standard orientation output LSR.

The coordinate rotation matrices L’LR and SOR are calculated by:

Figure 1: The different alignments and resets define the O (S') and/or L’ frame.
(a): the default orientation output of S with respect to L: LSR
(b): after an inclination reset, method 3: LOR
(c): after a heading reset, method 4: L’SR
(d): after an alignment reset, method 5: L’OR.
All of these are also possible to set with arbitrary alignments (methods 4 and 5).

 

The desired orientation resets (methods 3, 4 and 5) can be performed in MT Manager using the drop-down menu shown below:

First, choose the desired orientation reset from the drop-down menu. Then, use the first icon from the left to apply the orientation reset. Note that in order to write the result of the orientation reset to non-volatile memory, a "Store" action has to be performed, using the icon on the right. Only when stored to non-volatile memory, the MTi will use the updated reference co-ordinate system the next time it is powered up. This can be verified by checking the RotSensor and RotLocal matrices in the Device Settings window of MT Manager. 

Orientation resets can also be performed through low-level communication or XDA. Refer to the LLCP manual and XDA library (found in your MT SDK Documentation folder) for more information. 

 

 

Important remarks

The Euler orientation output is computed according to the Z-Y'-X" sequence (also known as aerospace sequence, or yaw-pitch-roll representation). 
We do not recommend performing an alignment reset during the initialization phase of the filter.  If you perform an alignment reset you may expect the filter to take the new value as fixed "North". However, this only works properly if the estimated heading has stabilized. It is recommended to wait at least 5 minutes before you perform the alignment reset (may be longer for the MTw depending on the magnetic environment).
The inclination reset (SOR) will not work if the sensor x-axis is aligned along the z-axis of the object, since the sensor's x-axis is used to describe the direction of the object's x-axis.
After a heading reset, the yaw may not be exactly zero, especially when the x-axis is close to the vertical. This is caused by the definition of yaw in Euler angles, which becomes unstable when the pitch approaches 90 deg.
 
Inertial Sensor Modules
Algorithms, Data & Theory
