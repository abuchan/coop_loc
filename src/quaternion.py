#-----------------------------------------------------------------------
# Brian Nemsick
# brian.nemsick@eecs.berkeley.edu
# November 2015
# quaternion.py
#-----------------------------------------------------------------------
# Purpose: Implement basic quaternion functions
# Note: JPL Convention
#-----------------------------------------------------------------------
import numpy
from numpy import matrix,hstack,vstack,sign
from numpy.linalg import norm
#-----------------------------------------------------------------------
# function: quat_norm(q)
# Purpose: Normalize a quaternion
#-----------------------------------------------------------------------
def quat_norm(q):
	return q/norm(q)
#-----------------------------------------------------------------------
# function: quat_inv(q)
# Purpose: Calculate inverse of a quaternion
#-----------------------------------------------------------------------
def quat_inv(q):
	return matrix([
	[-q[0,0]],
	[-q[1,0]],
	[-q[2,0]],
	[ q[3,0]]
	])
#-----------------------------------------------------------------------
# function: quat_mult(q,p)
# Purpose: Calculate the product of two quaternions
#-----------------------------------------------------------------------				  
def quat_mult(q,p, norm = True):
	if norm:
		return quat_norm(matrix([
		[ q[3,0]*p[0,0] + q[2,0]*p[1,0] - q[1,0]*p[2,0] + q[0,0]*p[3,0]],
		[-q[2,0]*p[0,0] + q[3,0]*p[1,0] + q[0,0]*p[2,0] + q[1,0]*p[3,0]],
		[ q[1,0]*p[0,0] - q[0,0]*p[1,0] + q[3,0]*p[2,0] + q[2,0]*p[3,0]],
		[-q[0,0]*p[0,0] - q[1,0]*p[1,0] - q[2,0]*p[2,0] + q[3,0]*p[3,0]],
		]))
	else:
		return matrix([
		[ q[3,0]*p[0,0] + q[2,0]*p[1,0] - q[1,0]*p[2,0] + q[0,0]*p[3,0]],
		[-q[2,0]*p[0,0] + q[3,0]*p[1,0] + q[0,0]*p[2,0] + q[1,0]*p[3,0]],
		[ q[1,0]*p[0,0] - q[0,0]*p[1,0] + q[3,0]*p[2,0] + q[2,0]*p[3,0]],
		[-q[0,0]*p[0,0] - q[1,0]*p[1,0] - q[2,0]*p[2,0] + q[3,0]*p[3,0]],
		])

#-----------------------------------------------------------------------
# function: quat_skew(q)
# Purpose: Calculate the skew-symmetric matrix operator of a quaternion
#-----------------------------------------------------------------------	
def quat_skew(q):
	return matrix([
	[0,-q[2,0],q[1,0]],
	[q[2,0],0,-q[0,0]],
	[-q[1,0],q[0,0],0],
	])
#-----------------------------------------------------------------------
# function: quat_cross(q,p)
# Purpose: Calculate the cross product of two quaternions
#-----------------------------------------------------------------------	
def quat_cross(q,p):
	return matrix([
	[q[1,0]*p[2,0]-q[2,0]*p[1,0]],
	[q[2,0]*p[0,0]-q[0,0]*p[2,0]],
	[q[0,0]*p[1,0]-q[1,0]*p[0,0]],
	])
#-----------------------------------------------------------------------
# function: quat_pi(q)
# Purpose: Remove q_w from a quaternion
#-----------------------------------------------------------------------	
def quat_pi(q, norm = True):
	if norm:
		q = quat_norm(q)
		return 2.0*sign(q[3,0])*q[[0,1,2],0]
	else:
		return sign(q[3,0])*q[[0,1,2],0]
#-----------------------------------------------------------------------
# function: quat_pi_inv(q)
# Purpose: Adds q_w to a quaternion
#-----------------------------------------------------------------------	
def quat_pi_inv(q, norm = True):
	if norm:
		return quat_norm(vstack((1.0/2.0*q,matrix('1'))))
	else:
		return vstack((q,matrix('1')))
#-----------------------------------------------------------------------
# function: quat_Omega(w)
# Purpose: Calculate the Omega function
#-----------------------------------------------------------------------	
def quat_Omega(w):
	return vstack((hstack((-quat_skew(w),w)),
				   hstack((-w.transpose(),matrix([0])))))
#-----------------------------------------------------------------------
# function: quat2rot(q)
# Purpose: Calculate the rotation matrix associated with a quaternion
#-----------------------------------------------------------------------	
def quat2rot(q):
	q = quat_norm(q)
	return matrix([
	[q[0,0]**2-q[1,0]**2-q[2,0]**2+q[3,0]**2,2*(q[0,0]*q[1,0]+q[2,0]*q[3,0]),2*(q[0,0]*q[2,0]-q[1,0]*q[3,0])],
	[2*(q[0,0]*q[1,0]-q[2,0]*q[3,0]),-q[0,0]**2+q[1,0]**2-q[2,0]**2+q[3,0]**2,2*(q[1,0]*q[2,0]+q[0,0]*q[3,0])],
	[2*(q[0,0]*q[2,0]+q[1,0]*q[3,0]),2*(q[1,0]*q[2,0]-q[0,0]*q[3,0]),-q[0,0]**2-q[1,0]**2+q[2,0]**2+q[3,0]**2],
	])
#-----------------------------------------------------------------------
# function: quat_RK4(w_1,w_2,dt):
# Purpose: calculate the body quaternion from numerical intergration of
# w(k),w(k+1) using Runge-Kutta 4th order method
# Notes: See RK4 
#        w_1 = w(k), w_2 = w(k+1)
#-----------------------------------------------------------------------
def quat_RK4(w_1,w_2,dt):
	q_0 = matrix('0;0;0;1')
	k_1 = quat_Omega(w_1)*q_0/2.0
	k_2 = quat_Omega((w_1+w_2)/2.0)*(q_0 + dt/2.0 * k_1)/2.0
	k_3 = quat_Omega((w_1+w_2)/2.0)*(q_0 + dt/2.0 * k_2)/2.0
	k_4 = quat_Omega(w_2)*(q_0+dt*k_3)/2.0
	return quat_norm(q_0 + dt/6.0*(k_1+2.0*k_2+2.0*k_3+k_4))



