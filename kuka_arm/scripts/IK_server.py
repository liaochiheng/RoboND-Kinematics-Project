#!/usr/bin/env python

# Copyright (C) 2017 Udacity Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya

# import modules
import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *
import numpy as np


def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:
        ### Your FK code here
        # Create symbols
        q1, q2, q3, q4, q5, q6, q7 = symbols( 'q1:8' )
        d1, d2, d3, d4, d5, d6, d7 = symbols( 'd1:8' )
        a0, a1, a2, a3, a4, a5, a6 = symbols( 'a0:7' )
        alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols( 'alpha0:7' )

        # Create Modified DH parameters
        DH_Table = {
            alpha0:        0, a0:       0, d1:  0.75, q1:          q1,
            alpha1: - pi / 2, a1:    0.35, d2:     0, q2: q2 - pi / 2,
            alpha2:        0, a2:    1.25, d3:     0, q3:          q3,
            alpha3: - pi / 2, a3: - 0.054, d4:  1.50, q4:          q4,
            alpha4:   pi / 2, a4:       0, d5:     0, q5:          q5,
            alpha5: - pi / 2, a5:       0, d6:     0, q6:          q6,
            alpha6:        0, a6:       0, d7: 0.303, q7:           0
        }

        # Define Modified DH Transformation matrix
        # Create individual transformation matrices
        T0_1 = TF_Matrix( alpha0, a0, d1, q1 ).subs( DH_Table )
        T1_2 = TF_Matrix( alpha1, a1, d2, q2 ).subs( DH_Table )
        T2_3 = TF_Matrix( alpha2, a2, d3, q3 ).subs( DH_Table )
        # T3_4 = TF_Matrix( alpha3, a3, d4, q4 ).subs( DH_Table )
        # T4_5 = TF_Matrix( alpha4, a4, d5, q5 ).subs( DH_Table )
        # T5_6 = TF_Matrix( alpha5, a5, d6, q6 ).subs( DH_Table )
        # T6_EE = TF_Matrix( alpha6, a6, d7, q7 ).subs( DH_Table )

        # T0_EE = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_EE

        # Define Rotation matrix from link_0 to link_6 using roll-pitch-yaw
        r, p, y = symbols( 'r p y' )
        R_06 = ROT_06( r, p, y )

        # Initialize service response
        joint_trajectory_list = []
        for x in xrange(0, len(req.poses)):
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()

        # Extract end-effector position and orientation from request
        # px,py,pz = end-effector position
        # roll, pitch, yaw = end-effector orientation
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])

            rospy.loginfo( '============== Request ===============' )
            rospy.loginfo( 'position: %.2f, %.2f, %.2f', px, py, pz )
            rospy.loginfo( 'orientation: %.2f, %.2f, %.2f', roll, pitch, yaw )
            rospy.loginfo( '========== End of Request ============' )

            ### Your IK code here
            R_06 = R_06.subs( { r: roll, p: pitch, y: yaw } )
            # Position of end effector
            EE = Matrix( [
                [ px ],
                [ py ],
                [ pz ]
                ] )
            # Position of wrist center
            WC = EE - 0.303 * R_06[ :, 2 ]

            # Calculate joint angles
            theta1 = atan2( WC[ 1 ], WC[ 0 ] )

            # See the schematic in MD file
            side_a = 1.5
            side_b = sqrt( pow( sqrt( WC[ 0 ] * WC[ 0 ] + WC[ 1 ] * WC[ 1 ] ) - 0.35, 2 ) \
                + pow( WC[ 2 ] - 0.75, 2 ) )
            side_c = 1.25

            ### TODO: improvements:
            # 1. Instead of arcos or arctan, we use to use arctan2 which has better angle properties than arcos or even arctan (https://en.wikipedia.org/wiki/Atan2)
            # 2. Also to inverse the kinematics, is you use transpose instead of .inv("LU"), as it is highly more effective, you should reach better performance.
            # 3. Use numpy instead of sympy, to increase the speed of operation

            angle_a = acos( ( side_b * side_b + side_c * side_c - side_a * side_a ) / ( 2 * side_b * side_c ) )
            angle_b = acos( ( side_a * side_a + side_c * side_c - side_b * side_b ) / ( 2 * side_a * side_c ) )
            angle_c = acos( ( side_a * side_a + side_b * side_b - side_c * side_c ) / ( 2 * side_a * side_b ) )

            theta2 = pi / 2 - angle_a - atan2( WC[ 2 ] - 0.75, sqrt( WC[ 0 ] * WC[ 0 ] + WC[ 1 ] * WC[ 1 ] ) - 0.35 )
            theta3 = pi / 2 - ( angle_b + 0.036 ) # 0.036 = atan2( a3, d4 )

            # Calculate rotation part from 0 to 3
            R0_3 = T0_1[ 0 : 3, 0 : 3 ] * T1_2[ 0 : 3, 0 : 3 ] * T2_3[ 0 : 3, 0 : 3 ]
            R0_3 = R0_3.evalf( subs = { q1: theta1, q2: theta2, q3: theta3 } )

            # Calculate rotation part from 3 to 6
            R3_6 = R0_3.inv('LU') * R_06

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

            # Euler angles, see above R3_6 equations
            theta4 = atan2( R3_6[ 2, 2 ], - R3_6[ 0, 2 ] )
            theta5 = atan2( sqrt( R3_6[ 0, 2 ] * R3_6[ 0, 2 ] + R3_6[ 2, 2 ] * R3_6[ 2, 2 ]), R3_6[ 1, 2 ] )
            theta6 = atan2( - R3_6[ 1, 1 ], R3_6[ 1, 0 ] )

            ###

            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
            joint_trajectory_point.positions = [
                Limit_Theta( theta1, -185, 185 ),
                Limit_Theta( theta2, -45, 85 ),
                Limit_Theta( theta3, -210, 65 ),
                Limit_Theta( theta4, -350, 350 ),
                Limit_Theta( theta5, -125, 125 ),
                Limit_Theta( theta6, -350, 350 )
            ]
            joint_trajectory_list.append(joint_trajectory_point)

        rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
        return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()

def TF_Matrix( alpha, a, d, q ):
    TF = Matrix( [
            [                cos( q ),             - sin( q ),               0,                  a ],
            [ sin( q ) * cos( alpha ), cos( q ) * cos( alpha ), - sin( alpha ), - sin( alpha ) * d ],
            [ sin( q ) * sin( alpha ), cos( q ) * sin( alpha ),   cos( alpha ),   cos( alpha ) * d ],
            [                       0,                       0,              0,                  1 ]
        ] )
    return TF

def Limit_Theta( theta, lower, upper ):
    deg = 0.017453293 # degrees to radians
    old = theta
    while theta < lower * deg:
        theta = theta + pi
    while theta > upper * deg:
        theta = theta - pi
    _ = '-' if theta == old else '*'
    rospy.loginfo( "====> Theta[%s]: %.2f + [%.2f ~ %.2f] = %.2f", _, old / deg, lower, upper, theta / deg )
    return float( theta )

# Rotation matrix for roll-pitch-yaw
def ROT_06( r, p, y ):
    R_x = Matrix( [
        [ 1,        0,         0 ],
        [ 0, cos( r ), -sin( r ) ],
        [ 0, sin( r ),  cos( r ) ]
    ] )

    R_y = Matrix( [
        [ cos( p ),     0,  sin( p ) ],
        [        0,     1,         0 ],
        [-sin( p ),     0,  cos( p ) ]
    ] )

    R_z = Matrix( [
        [ cos( y ), -sin( y ),      0 ],
        [ sin( y ),  cos( y ),      0 ],
        [         0,        0,      1 ]
    ] )

    R_EE = R_z * R_y * R_x

    # Anti-compensate for the discrepancy, transform ROT_EE to ROT_0_6
    R_Error = R_z.subs( y, radians( 180 ) ) * R_y.subs( p, radians( -90 ) )

    return R_EE * R_Error


if __name__ == "__main__":
    IK_server()
