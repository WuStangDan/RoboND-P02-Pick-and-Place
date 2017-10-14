# Project: Kinematics Pick & Place

[//]: # (Image References)

[image0]: ./dh-origins.png
[image1]: ./dh-rules.png
[image2]: ./dh-table-small.png
[image3]: ./rviz.png
[image4]: ./terminal.png


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

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

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

#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

And here's where you can draw out and show your math for the derivation of your theta angles. 



### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 


Here I'll talk about the code, what techniques I used, what worked and why, where the implementation might fail and how I might improve it if I were going to pursue this project further.  


And just for fun, another example image:


