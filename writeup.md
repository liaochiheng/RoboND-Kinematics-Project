## Project: Kinematics Pick & Place

---


**Steps to complete the project:**  


1. Set up your ROS Workspace.
2. Download or clone the [project repository](https://github.com/udacity/RoboND-Kinematics-Project) into the ***src*** directory of your ROS Workspace.  
3. Experiment with the forward_kinematics environment and get familiar with the robot.
4. Launch in [demo mode](https://classroom.udacity.com/nanodegrees/nd209/parts/7b2fd2d7-e181-401e-977a-6158c77bf816/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/ae64bb91-e8c4-44c9-adbe-798e8f688193).
5. Perform Kinematic Analysis for the robot following the [project rubric](https://review.udacity.com/#!/rubrics/972/view).
6. Fill in the `IK_server.py` with your Inverse Kinematics code.


[//]: # (Image References)

[kr210_frames_DH]: ./misc_images/kr210_frames_DH.png
[kr210_frames_URDF]: ./misc_images/kr210_frames_URDF.png
[transform_eq]: ./misc_images/transform_eq.png
[cal_theta]: ./misc_images/cal_theta.png
[screenshot1]: ./misc_images/Screenshot_1.png
[screenshot2]: ./misc_images/Screenshot_2.png
[screenshot3]: ./misc_images/Screenshot_3.png
[screenshot4]: ./misc_images/Screenshot_4.png
[screenshot5]: ./misc_images/Screenshot_5.png

## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

1. Assign reference frames for DH parameters.
+ KR210 have 6 revolute joints, so it's straightforward to define Z axis for every joint, along each joint axis.
+ Make positive of Z intuitive, pointing up, in or right.
+ Drawing common normal between neighboring Z axis, this would be X axis.
+ Make positive of X intuitive, pointing right or up.
+ Since joint 4-6 compose a spherical wrist, it's a good choice to make origins of frame 4-6 at the same point.
+ For base_link, i choose the X0 axis parallel to X1, with the origin same to the world frame.
+ For end effector, the gripper, i choose the X_G parallel to X6, with Z_G same to Z_6.
+ Then that's the reference frames.

Here is a schematic for DH frames of kr210 joints.

![Frames of kr210 with DH][kr210_frames_DH]

2. Here is a schematic of frames, the black triangles are origins of URDF frames, and XYZ are same to base_link.

![Frames of kr210 with URDF][kr210_frames_URDF]

3. List parameters from URDF ['kr210.urdf.xacro'](./kuka_arm/urdf/kr210.urdf.xacro).

Joint Name | Parent Link | Child Link | x | y | z
--- | --- | --- | --- | --- | ---
joint_1 | base_link | link_1 | 0 | 0 | 0.33
joint_2 | link_1 | link_2 | 0.35 | 0 | 0.42
joint_3 | link_2 | link_3 | 0 | 0 | 1.25
joint_4 | link_3 | link_4 | 0.96 | 0 | -0.054
joint_5 | link_4 | link_5 | 0.54 | 0 | 0
joint_6 | link_5 | link_6 | 0.193 | 0 | 0
gripper_joint | link_6 | gripper_link | 0.11 | 0 | 0

4. Calculate DH parameters, based on the URDF parameters.

Links | alpha(i-1) | a(i-1) | d(i) | theta(i)
--- | --- | --- | --- | ---
0->1 | 0 | 0 | 0.75 | q1
1->2 | - pi/2 | 0.35 | 0 | -pi/2 + q2
2->3 | 0 | 1.25 | 0 | q3
3->4 | - pi/2 | -0.054 | 1.5 | q4
4->5 | pi/2 | 0 | 0 | q5
5->6 | - pi/2 | 0 | 0 | q6
6->EE | 0 | 0 | 0.303 | 0

+ d1 = 0.75 = 0.33(z offset from base_link to link_1) + 0.42(z offset from link_1 to link_2)
+ a1 = 0.35(x offset from link_1 to link_2)
+ q2 = -pi/2 + q2(consider initial -pi/2 offset between X1 and X2)
+ a2 = 1.25(z offset from link_2 to link_3)
+ a3 = -0.054(z offset from link_3 to link_4)
+ d4 = 1.5 = 0.96(x offset from link_3 to link_4) + 0.54(x offset from link_4 to link_5)
+ dEE = 0.303 = 0.193(x offset from link_5 to link_6) + 0.11(x offset from link_6 to gripper)

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

1. Create individual transformation matrices using DH parameter. This would be very straightforward. [IK_server.py line #30~55](./kuka_arm/scripts/IK_server.py)

+ I wrote a function to create a T(i-1 -> i):
```
def TF_Matrix( alpha, a, d, q ):
    TF = Matrix( [
            [                cos( q ),             - sin( q ),               0,                  a ],
            [ sin( q ) * cos( alpha ), cos( q ) * cos( alpha ), - sin( alpha ), - sin( alpha ) * d ],
            [ sin( q ) * sin( alpha ), cos( q ) * sin( alpha ),   cos( alpha ),   cos( alpha ) * d ],
            [                       0,                       0,              0,                  1 ]
        ] )
    return TF
```
+ Then create matrices:
```
	T0_1 = TF_Matrix( alpha0, a0, d1, q1 ).subs( DH_Table )
	T1_2 = TF_Matrix( alpha1, a1, d2, q2 ).subs( DH_Table )
	T2_3 = TF_Matrix( alpha2, a2, d3, q3 ).subs( DH_Table )
	...
```

2. Generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

+ The transform consist of two parts: Rotation part and Translation part.
+ Rotation part can be calculated like this: R0_G = R_z(yaw) * R_y(pitch) * R_z(roll), roll, pitch, yaw could be extracted from pose of the gripper. This is a extrinsic rotaion of x_y_z or intrinsic of z_x_y.
+ Translation part is more easier, it's simply the position of the gripper: [px, py, pz].
+ So the whole transform will be: (R_T = R0_G)
	![Transform equation][transform_eq]

#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

1. Define DH parameters and transform matrices.
+ See line #30~55 IK_server.py, and explained above.
2. Define R_06 with symbols 'r, p, y'.
+ See line #159, i wrote a function `ROT_06` to return a R_06 based on roll, pitch, yaw.
+ First calculate `R_EE = R_z * R_y * R_x`, this is simple.
+ Then there is a trick. R_EE is not R_06 because of the discrepancy between DH convention and Gazebo. So the final R_06 should be `R_06 = R_EE * R_Error`.
3. Derive wrist center
+ Extract EE position from pose. `EE = [px, py, pz]`
+ Extract roll/pitch/yaw from pose, then evaluate `R_06 = R_06.subs( { r: roll, p: pitch, y: yaw } )`
+ Then WC would be easy to calculate. `WC = EE - 0.303 * R_06[ :, 2 ]`. Here 0.303 is dEE in DH table.
4. Derive theta 1-3
+ Theta 1 is simple. `theta1 = atan2( WC[ 1 ], WC[ 0 ] )`
+ Theta 2 and 3 is a little tricky. It will be easier using the top view. See the schematic below.
![schematic of theta][cal_theta]
+ See the right triangle, it's easy to calculate `d_xy = sqrt(wcx**2 + wcy**2) - a1` and `d_z = wcz - d1`. Then the hypotenuse is `sqrt(d_xy**2 + d_z**2)`, and that is the `side_b` of the upper triangle. (IK_server.py line #96)
+ For the upper triangle, `side_a` and `side_c` is `d4` and `a2` respectively.
+ Then i got all the side length of the triangle. Using laws of cosine, i could calculate all the 3 angles. (IK_server.py line #100~102)
+ Now calculate theta2 = pi/2 - angle_a - the one angle of the right triangle.`theta2 = pi / 2 - angle_a - atan2( WC[ 2 ] - 0.75, sqrt( WC[ 0 ] * WC[ 0 ] + WC[ 1 ] * WC[ 1 ] ) - 0.35 )`. Notice the one angle of the right triangle is atan2(d_z, d_xy) which are mentioned above.
+ Notice the link_3 incline a little downwards with angle `alpha = 0.036 = atan2( a3, d4 )`.
+ So it's easy to know: `theta3 - alpha = angle_b - pi/2`, then `theta3 = alpha + angle_b - pi/2`. Notice the theta3 should be negative based on DH convention, then:`theta3 = pi / 2 - ( angle_b + 0.036 )`
5. Derive theta 4-6
+ Since i know theta 1-3, so R0_3 could be calculated through evaluation of T1_3.
```
R0_3 = T0_1[ 0 : 3, 0 : 3 ] * T1_2[ 0 : 3, 0 : 3 ] * T2_3[ 0 : 3, 0 : 3 ]
R0_3 = R0_3.evalf( subs = { q1: theta1, q2: theta2, q3: theta3 } )
```
+ Calculate R3_6. `R3_6 = R0_3.inv('LU') * R_06`, notice `R_06` is derived from roll/pitch/yaw of EE.
+ Derive the `R3_6` through `T3_6`:
```
################################
### Extract R3_6 from T3_4 * T4_5 * T5_6:
# R3_6 = Matrix([
#     [-sin(q4)*sin(q6) + cos(q4)*cos(q5)*cos(q6),
#         -sin(q4)*cos(q6) - sin(q6)*cos(q4)*cos(q5),
#             -sin(q5)*cos(q4)],
#     [sin(q5)*cos(q6), -sin(q5)*sin(q6), cos(q5)],
#     [-sin(q4)*cos(q5)*cos(q6) - sin(q6)*cos(q4),
#         sin(q4)*sin(q6)*cos(q5) - cos(q4)*cos(q6),
#             sin(q4)*sin(q5)],
# ])
################################
```
+ Then i could find couple of approaches to derive q4/q5/q6, here is one of them.
```
theta4 = atan2( R3_6[ 2, 2 ], - R3_6[ 0, 2 ] )
theta5 = atan2( sqrt( R3_6[ 0, 2 ] * R3_6[ 0, 2 ] + R3_6[ 2, 2 ] * R3_6[ 2, 2 ]), R3_6[ 1, 2 ] )
theta6 = atan2( - R3_6[ 1, 1 ], R3_6[ 1, 0 ] )
```
6. That's all for joint angles.


### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results.


I think i have discussed the code in the above section.


Here are some screenshots for a pick-place action:
![screenshot of pick and place][screenshot1]
![screenshot of pick and place][screenshot2]
![screenshot of pick and place][screenshot3]
![screenshot of pick and place][screenshot4]
![screenshot of pick and place][screenshot5]
