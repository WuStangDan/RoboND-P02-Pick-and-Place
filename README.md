# Project: Kinematics Pick & Place

[//]: # (Image References)

[image0]: ./dh-origins.png
[image1]: ./dh-rules.png
[image2]: ./dh-table-small.png
[image3]: ./rviz.png
[image4]: ./terminal.png
[image5]: ./fig0-small.png
[image6]: ./fig2-small.png


### Kinematic Analysis
#### Denavit-Hartenberg Parameters
For the DH origins I selected the same locations as suggested in the lectures. The x, z axes locations can be seen in relation to all of the joints below.

![DH Figures][image0]

The with the origin locations the the DH table was filled in by following the rules listed below:
![DH Rules][image1]

<!---
Latex Code for Table and equation

\begin{document}

\begin{table}[t]
\begin{center}
\caption{Denavit-Hartenberg Parameters}
\vspace{0.2cm}
\begin{tabular}{c | c c c c }
Link & $\alpha_{(i-1)}$ & $a_{(i-1)$ & $d_i$ & $\theta_i$\\
\hline
1 & 0 & 0 & $L_1$ & $q_1$ \\
2 & $\frac{\pi}{2}$ & $L_2$ & 0 & $q_2-\frac{\pi}{2}$ \\
3 & 0 & $L_3$ & 0 & $q_3$ \\
4 & $-\frac{\pi}{2}$ & $L_4$ & $L_5$ & $q_4$ \\
5 & $\frac{\pi}{2}$ & 0 & 0 & $q_5$ \\
6 & $-\frac{\pi}{2}$ & 0 & 0 &$q_6$ \\
G & 0 & 0 & $L_6$ & 0
\end{tabular}
\end{center}
\end{table}

\begin{align*}
L_1 &= 0.33 + 0.42 = 0.75 \,\mathrm{m} \\
L_2 &= 0.35 \,\mathrm{m} \\
L_3 &= 1.25 \,\mathrm{m} \\
L_4 &= -0.054  \,\mathrm{m} \\
L_5 &= 0.96 + 0.54 = 1.5  \,\mathrm{m} \\
L_6 &= 0.193 + 0.11 = 0.303  \,\mathrm{m}
\end{align*}

\end{document}
--->

![DH Table][image2]

The L values were calculated based on the xyz values found in the urdf files. These values give the distances between the joints. Since the DH origins don't align with the joints, the L values are sometimes made up of multiple joint distances.

#### Individual Transformation Matrices

The general homogenous transform (for Link 1) can be seen below. The exact same form is used for all transforms up to the gripper or end effector. The only difference between all the matrices are the q, alpha, a, and d values which come from the DH table above.

	T0_1 = Matrix([[cos(q1),             -sin(q1),            0,            a0],
	               [sin(q1)*cos(alpha0), cos(alpha0)*cos(q1), -sin(alpha0), -sin(alpha0)*d1],
	               [sin(alpha0)*sin(q1), sin(alpha0)*cos(q1), cos(alpha0),  cos(alpha0)*d1],
	               [0,                   0,                   0,            1]])


However these transformations will result in a different gripper orientation when compared with the RViz robot which needs to be corrected. To correct for this a rotation in Y and Z is required on the final transform.

The code for that can be seen below.

	# Correction for gripper link final axis orientation.
	R_y = Matrix([[cos(-pi/2),  0, sin(-pi/2), 0],
		      [0,           1, 0,           0],
		      [-sin(-pi/2), 0, cos(-pi/2),  0],
		      [0, 0, 0, 1]])
	R_z = Matrix([[cos(pi), -sin(pi), 0,  0],
		      [sin(pi), cos(pi),  0,  0],
		      [0, 0, 1, 0],
		      [0, 0, 0, 1]])

	R_cor = simplify(R_z * R_y)
	T_total = simplify(T0_G * R_cor)

Since the total matrix is far too large to show, I've included an example that it yields the same results as RViz to verify it's correct. The matrix was tested at more points than is shown below.

	print("Total = ", T_total.evalf(subs={q1:-0.97, q2:-0.39, q3:0.72, q4:1.10, q5:-1.10, q6:0}))

![RViz][image3]

![Terminal Output][image4]

#### Inverse Kinematics 

At this point the X, Y, Z position of the gripper and it's orientation in roll, pitch, and yaw is known (calculated from quaternion). Inverse kinematics is used to reverse this information into angles for each joint that will yield this position of the gripper.

First, the location of the wrist center (joint 5) is calculated using the gripper position and orientation. The code for that caluclation is shown below where s[d7] is the d\_G in the original figure and R\_cor is the same rotation matrix for the gripper axis correction shown above.

	Rrpy = rot_z(yaw) * rot_y(pitch) * rot_x(roll) * R_cor
 	wx = px - s[d7] * Rrpy[0,2]
 	wy = py - s[d7] * Rrpy[1,2]
 	wz = pz - s[d7] * Rrpy[2,2]

With the wrist center position known, the angles for the first 3 joints can be calculated. The angle of joint one is easy as it just needs to rotate so that the robot arm is aligned with the X, Y position.

	theta1 = atan2(wy, wx)
	
Calculating the angles of joint 2 and joint 3 are a bit more difficult. To calculate angle 2, two triangles must be used. One triangle is a right triangle from joint 2, to the wrist center. The second traingle connects joint 2, joint 3, and the wrist center. Both of these triangles can be seen below.

![triangles][image5]

From the figure it can be seen that the angle of joint 2 is 90 degrees minus alpha and the arctan of the distance along the floor and the height of the wrist center relative to joint 2. Alpha can be calculated using the cosine laws since all the side lengths of the triangle are known.

For joint 3, gamma can also be calculated using the cosine laws. However, recall that when theta 3 equals zero, it doesn't mean that gamma will be 90 degrees. This is because of the small offset due to the height difference between joint 3 and joint 5. This tiny angle must be accounted for as shown in the figure below.

![Joint 3][image6]

The figure above shows the case when theta 3 = 0. When this is the case, gamma = 90 degrees - atan of a3 and d4. The code for this calculation is shown below.

	theta3 = pi/2 - tri_gamma + atan2(s[a3], s[d4])

Note that the sign infront of the arctangent is + because the function atan2 is used. Since a3 is a negative height, the result of atan2 will be negative and thus the correct sign is used.


To calculate the last 3 angles (theta 4, 5, 6), the individual transform matrices from link 1 to link 3 are multiplited together. This yields a single transform matrix from the base to joint 3, T0_3.

Recall that the matrix Rrpy used above is the same as the transform from base to the gripper since Rrpy was just the gripper position and orientation translated into the coordinates we used originally for our transformation matrices to go from the base link to the gripper.

Therefore:

	T0_G = Rrpy
	T3_G_calc = inv(T0_3) * Rrpy
	
This will calculate T3\_6 and fill in all the values of the matrix. Since we can calculate T3\_G symbolically, as shown below, we can set the cells with our theta variables equal to the calculated values of the fully calculated T3\_G_calc. I show a cleaner version of T3\_G below where only the locations required to calculate theta 4, 5, or 6 are shown.

	T3_G = simplify(T3_4 * T4_5 * T5_6 * T6_G )
	T3_G = Matrix([[-sin(q4)*sin(q6) + cos(q4)*cos(q5)*cos(q6), -sin(q4)*cos(q6) - sin(q6)*cos(q4)*cos(q5), -sin(q5)*cos(q4), -0.303*sin(q5)*cos(q4) - 0.054],
	    [sin(q5)*cos(q6), -sin(q5)*sin(q6), cos(q5), 0.303*cos(q5) + 1.5], 
	    [-sin(q4)*cos(q5)*cos(q6) - sin(q6)*cos(q4), sin(q4)*sin(q6)*cos(q5) - cos(q4)*cos(q6), sin(q4)*sin(q5), 0.303*sin(q4)*sin(q5)], 
	    [0, 0, 0, 1]])
	
	T3_G_clean = Matrix([[X, X, -sin(q5)*cos(q4), X],
	    [sin(q5)*cos(q6), -sin(q5)*sin(q6), cos(q5), X], 
	    [X, X, sin(q4)*sin(q5), X)], 
	    [0, 0, 0, 1]])


With variables for each joint angle (q4-6) the individual angles can be calculated using T3\_G\_calc as shown below.

	theta4 = atan2(R3_6[2,2], -R3_6[0,2])
	theta5 = acos(R3_6[1,2])
	theta6 = atan2(-R3_6[1,1], R3_6[1,0])


### Project Implementation

The code follows all the techniques outlined above. When running IK_debug.py all 3 test cases yeild an overall end effector offset of 0.00000000 units. All of the thetas also have errors in the 0.00X rad order of magnitude.

I ran 12 pick and place tests and it successfully grabbed 10 of them and placed them in the bin. The first video linked below shows this entire run. Both of the fails (and one success) happened when trying to grab the cylinder from the top right hand corner.

This passes the metric set out by the rubic. However I investigated further and found that when looking at the video it seemed that both failures happened because the gripper didn't close quick enough. I was already using a sleep duration of 2.0 seconds for the tests as suggested. However I increased the duration to 5.0 seconds and then recorded the second video. In the second video I was able to successfully grab a cylinder from the top right location. 

[10/12 Score Video](https://www.youtube.com/watch?v=YesG_mBrvQE) 

[Top Right Success](https://www.youtube.com/watch?v=YiineXpmEkA)

One clear area that could be improved upon is having more logic when setting the angles of joint 4 and 6. I believe a way to get rid of the back and forth spinning would be to retain some memory of previous angles set to ensure that any new angles are set to the closest 2pi value. This should eliminate the angle 4 joint going from -pi/2 to 6/4pi and doing a full spin around. 
