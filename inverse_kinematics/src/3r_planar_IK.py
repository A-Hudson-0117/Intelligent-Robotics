#!/usr/bin/env python3

import math   as m
import numpy  as np

def main():
	# Length of each links for the planar 3R robot
	l1 = 5
	l2 = 7
	l3 = 0

	# Given the following transformation from 0 to P:
	T0_P = np.mat([	   [0.8435, 0.5372, 0, 9.0],
				   [-0.5372, 0.8435, 0, 6.0],
				   [0, 0, 1, 0],
				   [0, 0, 0, 1]])

	# Find at least 2 solutions sets [theta_1, theta_2, theta_3] for the robot 
	# manipulator position.
	# Use IK formulas learned from the lecture.

	# Get the x, y coordinate from T0_P:
	x, y = T0_P[0,3], T0_P[1, 3] 
	print('x, y coord: {},{}'.format(x,y))

	cos_phi, sin_phi = T0_P[0,0], T0_P[1,0] 
	print('sin_phi = {} \ncos_phi = {}'.format(sin_phi,cos_phi))

	# If surprised by the order here, it is recommended to review the lecture.
	# You are welcome to add as many intermediate steps as you judge necessary.

	#_*-*_*-*_*-*_*-*_*-*_*-*_*-*_*-*_*-*_*-*_*-*_*-*_*-*_*-*_*-*_*-*_*-*_*-*_*-*_*-*_#

	# Here starts the calculations for the first set of solutions:
	print("\nSolution 1: \n")

	# Theta 2
	cos_theta_2 = ( m.pow(x, 2) + m.pow(y,2) - m.pow(l1,2) - m.pow(l2,2) ) / ( 2 * l1 * l2 )
	print("Cosinus Theta 2: {}".format(cos_theta_2))

	sin_theta_2 = 1 * m.sqrt( 1.0 - m.pow(cos_theta_2,2) )
	print("Sinus Theta 2: {}".format(sin_theta_2))

	theta_2 = m.atan2(sin_theta_2, cos_theta_2)

	# Theta 1
	k1 = l1+ l2 * cos_theta_2
	k2 = l2 * sin_theta_2

	theta_1 = m.atan2(y, x) - m.atan2(k2, k1)

	# Theta 3
	phi = m.atan2(sin_phi, cos_phi)
	print("Phi: {}".format(m.degrees(phi)))

	theta_3 = phi - theta_1 - theta_2

	# 1st set of solution with angles converted to degrees
	solution_1 = [m.degrees(theta_1), m.degrees(theta_2), m.degrees(theta_3)]
	print("Theta 1, 2, and 3: {}".format(solution_1))

	#_*-*_*-*_*-*_*-*_*-*_*-*_*-*_*-*_*-*_*-*_*-*_*-*_*-*_*-*_*-*_*-*_*-*_*-*_*-*_*-*_#

	# Here starts the calculations for the second set of solutions:
	print("\nSolution 2: \n")

	# Theta 2
	cos_theta_2 = ( m.pow(x, 2) + m.pow(y,2) - m.pow(l1,2) - m.pow(l2,2) ) / ( 2 * l1 * l2 )
	print("Cosinus Theta 2: {}".format(cos_theta_2))

	sin_theta_2 = -1 * m.sqrt( 1.0 - m.pow(cos_theta_2,2) )
	print("Sinus Theta 2: {}".format(sin_theta_2))

	theta_2 = m.atan2(sin_theta_2, cos_theta_2)

	# Theta 1
	k1 = l1+ l2 * cos_theta_2
	k2 = l2 * sin_theta_2

	theta_1 = m.atan2(y, x) - m.atan2(k2, k1)

	# Theta 3
	phi = m.atan2(sin_phi, cos_phi)
	print("Phi: {}".format(m.degrees(phi)))

	theta_3 = phi - theta_1 - theta_2

	# 2nd set of solution with angles converted to degrees
	solution_2 = [m.degrees(theta_1), m.degrees(theta_2), m.degrees(theta_3)]
	print("Theta 1, 2, and 3: {}".format(solution_2))

if __name__ == '__main__':
	main()
