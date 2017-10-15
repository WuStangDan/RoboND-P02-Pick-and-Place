#!/usr/bin/env python

# Copyright (C) 2017 Electric Movement Inc.
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
from sympy import symbols, cos, sin, pi, simplify, sqrt, atan2, acos, asin
from sympy.matrices import Matrix


def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:
        
        ### Your FK code here
        # Create symbols
        q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')
        d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
        a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
        alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')
        # Denavit-Hartenberg parameter values.
        s = {alpha0: 0,     a0: 0,      d1: 0.75, 
             alpha1: -pi/2, a1: 0.35,   d2: 0,     q2: q2-pi/2,
             alpha2: 0,     a2: 1.25,   d3: 0,
             alpha3: -pi/2, a3: -0.054, d4: 1.5,
             alpha4: pi/2,  a4: 0,      d5: 0,
             alpha5: -pi/2, a5: 0,      d6: 0,
             alpha6: 0,     a6: 0,      d7: 0.303, q7: 0}
        # Define Modified DH Transformation matrix
        def rot_x(q):
            R_x = Matrix([[ 1,              0,        0],
                      [ 0,         cos(q),  -sin(q)],
                      [ 0,         sin(q),  cos(q)]])
            return R_x
        
        def rot_y(q):              
            R_y = Matrix([[ cos(q),        0,  sin(q)],
                      [      0,        1,       0],
                      [-sin(q),        0, cos(q)]])
            return R_y

        def rot_z(q):    
            R_z = Matrix([[ cos(q),  -sin(q),       0],
                      [ sin(q),   cos(q),       0],
                      [      0,        0,       1]])
            return R_z

        def homogeneous_transform(alpha, a, d, q):
            M = Matrix([[cos(q),            -sin(q),           0,           a],
                        [sin(q)*cos(alpha), cos(alpha)*cos(q), -sin(alpha), -sin(alpha)*d],
                        [sin(alpha)*sin(q), sin(alpha)*cos(q), cos(alpha),  cos(alpha)*d],
                        [0,                 0,                 0,           1]])
            return M
    
        # Create individual transformation matrices
        T0_1 = homogeneous_transform(alpha0, a0, d1, q1)
        T0_1 = T0_1.subs(s)

        T1_2 = homogeneous_transform(alpha1, a1, d2, q2)
        T1_2 = T1_2.subs(s)

        T2_3 = homogeneous_transform(alpha2, a2, d3, q3)
        T2_3 = T2_3.subs(s)

        # Extract rotation matrices from the transformation matrices
        R_cor = rot_z(pi) * rot_y(-pi/2)
        T0_3 = simplify(T0_1 * T1_2 * T2_3)
        T0_3_t = T0_3.transpose() 
        ###

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
     
            ### Your IK code here 
            
            # Compensate for rotation discrepancy between DH parameters and Gazebo
            Rrpy = rot_z(yaw) * rot_y(pitch) * rot_x(roll) * R_cor
            
            wx = px - s[d7] * Rrpy[0,2]
            wy = py - s[d7] * Rrpy[1,2]
            wz = pz - s[d7] * Rrpy[2,2]

            # Calculate joint angles using Geometric IK method
            theta1 = atan2(wy, wx)
            floor_dist = sqrt( (wy) **2 + wx**2) # distance to wc with z = 0.

            # Law of cosines.
            c = sqrt( (floor_dist-s[a1])**2 + (wz-s[d1])**2)
            a = sqrt(s[d4]**2 + s[a3]**2)
            b = s[a2]
            
            # Theta2 is 90 degrees minus the angle from joint2 x to wc plus angle closest to the base(alpha) 
            # of the triangle created from by joint2, joint3, and wc.
            # Theta3 is 90 degrees minus the angle of that same triangle that is closest to joint 3. Since 
            # joint3 to joint5 (wc) isn't straight (has small height offset which is a3) this must also be subtracted
            # from the 90 degrees.
            tri_alpha = acos( (b**2 + c**2 - a**2)/(2*b*c) )
            theta2 = pi/2 - tri_alpha - atan2(wz-s[d1], floor_dist-s[a1])
            tri_gamma = acos( (a**2 + b**2 - c**2)/(2*a*b) )
            theta3 = pi/2 - tri_gamma + atan2(s[a3], s[d4])

            Rrpy = Rrpy.row_insert(3, Matrix([[0, 0, 0]]))
            Rrpy = Rrpy.col_insert(3, Matrix([[0], [0], [0], [1]]))
            R3_6 = T0_3_t * Rrpy
            R3_6 = R3_6.evalf(subs={q1:theta1, q2:theta2, q3:theta3})

            theta4 = atan2(R3_6[2,2], -R3_6[0,2])
            theta5 = acos(R3_6[1,2])
            theta6 = atan2(-R3_6[1,1], R3_6[1,0])
            ###
        
            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
            joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
            joint_trajectory_list.append(joint_trajectory_point)

        rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
        return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()

if __name__ == "__main__":
    IK_server()
