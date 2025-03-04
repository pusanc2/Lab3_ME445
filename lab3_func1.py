#!/usr/bin/env python
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

	w_list = [[0,0,1],
	[0,1,0],
	[0,1,0],
	[0,1,0],
	[1,0,0],
	[0,1,0]]


	q_list = [[-0.15,0.15,0],
	[-0.15,0,0.162],
	[0.094,0,0.162],
	[0.307,0,0.162],
	[0,0.270,0.162],
	[0.390,0,0.162]]
	
	V = []
    for i in [0, 5]:
    	v_i = np.cross(-np.array(w_list[i]), np.array(q_list[i]))
    	V.append(v_i)
    
    S = []
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

	T = np.array(M)
    
    for i in range(len(thetalist) - 1, -1, -1):
        T = np.dot(MatrixExp6(VecTose3(np.array(Slist)[:, i] \
                                       * thetalist[i])), T)
    return T








	# ==============================================================#

	print(str(T) + "\n")

	return_value[0] = theta1 + PI
	return_value[1] = theta2
	return_value[2] = theta3
	return_value[3] = theta4 - (0.5*PI)
	return_value[4] = theta5
	return_value[5] = theta6

	return return_value
