#!/usr/bin/env python3
import numpy as np
from scipy.linalg import expm
from lab3_header import *

"""
Use 'expm' for matrix exponential.
Angles are in radian, distance are in meters.
"""

def Get_MS():
	# =================== Your code starts here ====================#
	# Fill in the correct values for S1~6, as well as the M matrix
	M = np.array([
     [0, -1, 0, 0.39],
     [0, 0, -1, 0.405],
     [1, 0, 0, 0.215],
     [0,0, 0, 1]
		
	])

	w_list = [np.array([0,0,1]),
		np.array([0,1,0]),
		np.array([0,1,0]),
		np.array([0,1,0]),
		np.array([1,0,0]),
		np.array([0,1,0])
  	]

	q_list = [np.array([-0.15,0.15,0]),
		np.array([-0.15,0,0.162]),
		np.array([0.094,0,0.162]),
		np.array([0.307,0,0.162]),
		np.array([0,0.270,0.162]),
		np.array([0.390,0,0.162])
  	]
	
	S = np.zeros((6,6))
	for i in range(6):
		w = w_list[i]
		q = q_list[i]
		v = np.cross(-w, q)
		S[:, i] = np.concatenate([w, v])
	
	print(M, "\n")
	print(S, "\n")
	# ==============================================================#
	return M, S


"""
Function that calculates encoder numbers for each motor
"""

def lab_fk(theta1, theta2, theta3, theta4, theta5, theta6):

	# Initialize the return_value
	return_value = [None, None, None, None, None, None]

	# =========== Implement joint angle to encoder expressions here ===========
	print("Foward kinematics calculated:\n")

	# =================== Your code starts here ====================#

	thetalist = np.array([theta1, theta2, theta3, theta4, theta5, theta6])

	M,S = Get_MS()
 
	T = np.eye(4)
 
	for i in range(6):

		# Extract w,v
		w = S[:3, i]
		v = S[3:, i]
	
		# Build S matrix
		S_matrix = np.zeros((4, 4))
		S_matrix[0:3, 0:3] = np.array([
			[0, -w[2], w[1]],
			[w[2], 0, -w[0]],
			[-w[1], w[0], 0]
		])
		S_matrix[0:3, 3] = v
	
		# Matrix exponential
		T_i = expm(S_matrix * thetalist[i])
	
		T = T @ T_i
	
	T = T @ M

	#np.set_printoptions(suppress=True, precision=8)

	# ==============================================================#

	print(str(T) + "\n")

	return_value[0] = theta1 + PI
	return_value[1] = theta2
	return_value[2] = theta3
	return_value[3] = theta4 - (0.5*PI)
	return_value[4] = theta5
	return_value[5] = theta6

	return return_value
