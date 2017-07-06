#-----------------------------------------------------------------------
# Brian Nemsick
# brian.nemsick@eecs.berkeley.edu
# May 2016
# coop_slam.py
#-----------------------------------------------------------------------

from numpy import array, matrix, vstack, hstack, copy
from numpy.matlib import eye, zeros
from numpy.linalg import inv, norm
from quaternion import *
from ind import *

class EKF(object):
	
	def __init__(self,n,s2_g,s2_a,s2_wg,s2_wa,q_O_C,p_C):

		#---------------------------------------------------------------
		# Initilization: IMU Noise and State Transition Matrix
		#---------------------------------------------------------------
		self.Q_c = self.calc_Q_c(n,s2_g,s2_a,s2_wg,s2_wa)
		self.phi = eye(PL*n)

		#---------------------------------------------------------------
		# Observer Body -> Camera Transform
		#---------------------------------------------------------------
		self.q_O_C = q_O_C
		self.p_C = p_C
		self.s2_g = s2_g
		self.s2_a = s2_a

	#-------------------------------------------------------------------
	# function: propagate_IMU(self,n,x,P,u,c,dt,gd)
	# Purpose: Propagate the state of the team with IMU Measurements
	# Assumed: 1 Observer robot, N-1 Picket robots
	#-------------------------------------------------------------------
	# Notes: 
	#       x_i = [q,p,v,b_a,b_g]'
	#           q = quaternion (QUAT)
	#           p = position (POS)
	#           v = velocity (VEL)
	#           b_g = bias Gyroscope (BG)
	#           b_a = bias Accelerometer (BA)
	#       w_m = measured angular velocity from the Gyroscope
	#       a_m = measured acceleration from the Accelerometer
	#       w_1 = w(k), w_2 = w(k+1)
	#       a_1 = a(k), a_2 = a(k+1)
	#       u(n) = [w,a]'
	#       dt = time step
	#       c(n) == 1 (moving), c(n) == 0 (stationary)
	#       gd = Gaussian Diffusion
	#--------------------------------------------------------------------

	def propagate_IMU(self,n,x,p,u,c,dt,gd):
		
		# Initialize  
		x_hat = copy(x)
		phi = copy(self.phi)
		G_c_1,G_c_2 = self.init_G_c(n,c)
		Q_c = self.Q_c
		
		# For each robot
		for ii in range (0,n):
			
			# Index change for ii-th robot
			jj = ii*SL # x_hat
			kk = ii*PL # P
			ll = ii*QL # G_c || Q_c
				
			# Is the robot moving?
			if c[ii,0]:

				#--------------------------------------------------------
				# Propagate State: x_hat (incrementally)
				#--------------------------------------------------------
				
				# Gyroscope: Quaternion change RK4
				w_1 = u[UG+UL*ii,0]-x[BG+jj,0] # subtract out the biases
				w_2 = u[UG+UL*ii,1]-x[BG+jj,0] # subtract out the biases

				q_B_B = quat_RK4(w_1,w_2,dt)
				
				# Calculate rotation matrices
				R_1 = quat2rot(x[QUAT+jj,0])
				R_2 = quat2rot(x_hat[QUAT+jj,0])
				R_1_T = R_1.T
				R_2_T = R_2.T
				
				# Calculate position, velocity change (Trapezoidal Integration)
				a_1 = u[UA+UL*ii,0]-x[BA+jj,0] # subtract out the biases
				a_2 = u[UA+UL*ii,1]-x[BA+jj,0] # subtract out the biases

				# Outlier Rejection on IMU
				if max(vstack((w_1,w_2))) > 3.0*self.s2_g or max(vstack((a_1+g,a_2+g))) > 3.0*self.s2_a:
					gd[ii,0] = True

				s = dt/2.0*(quat2rot(q_B_B).T*a_2+a_1)
				y = dt/2.0*s

				if gd[ii,0]:
					pass
				else:
					x_hat[QUAT+jj,0] =quat_mult(q_B_B,x[QUAT+jj,0])
					x_hat[VEL+jj,0] = x[VEL+jj,0]+R_1_T*s+g*dt
					x_hat[POS+jj,0] = x[POS+jj,0]+dt*x[VEL+jj,0]+R_1_T*y+g/2.0*dt**2


				# Calculate phi (Trapezoidal Integration)
				phi[PPOS+kk,PVEL.T+kk] = dt*eye(3) # phi_pv
				phi[PPOS+kk,PTHETA.T+kk] = -quat_skew(R_1_T*y); # phi_pq
				phi[PVEL+kk,PTHETA.T+kk] = -quat_skew(R_1_T*s); # phi_vq	
				phi[PTHETA+kk,PBG.T+kk] = -dt/2.0*(R_1_T+R_2_T); # phi_qbg
				phi[PVEL+kk,PBG.T+kk] = (dt**2)/4.0*quat_skew(R_2_T*(a_2)-g)*(R_1_T+R_2_T); # phi_vbg
				phi[PPOS+kk,PBG.T+kk] = dt/2.0*phi[PVEL+kk,PBG.T+kk]; # phi_pbg
				phi[PVEL+kk,PBA.T+kk] = phi[PTHETA+kk,PBG.T+kk]; # phi_vba
				phi[PPOS+kk,PBA.T+kk] = dt/2.0*phi[PVEL+kk,PBA.T+kk]; # phi_pba
				
				# Index the G_c rotation
				G_c_1[PVEL+kk,NA.T+ll] = -R_1_T
				G_c_2[PVEL+kk,NA.T+ll] = -R_2_T
				
			else:
				# Zero out velocity
				x_hat[VEL+jj,0] = zeros((3,1))
			
		# Calculate Q_d
		N_c_1 = G_c_1*Q_c*G_c_1.T
		N_c_2 = G_c_2*Q_c*G_c_2.T
		Q_d = dt/2.0*(phi*N_c_2*phi.T + N_c_1)
		
		# Calculate sigma_hat
		p_hat = phi*p*phi.T + Q_d
	
		# Maintain numerical precision & covariance matrix symmetry
		p_hat =(p_hat+p_hat.T)/2.0
		return(x_hat,p_hat)
	
	#--------------------------------------------------------------------
	# function: update_camera(self,n,x,x_prev,P,z,v,c,R)
	# Purpose: Update the state of the team with Camera Measurements
	#--------------------------------------------------------------------
	# Notes: 
	#       v(n) == 1 (Pose Estimate), v(n) == 0 (No Pose Estimate)
	#       c(n) == 1 (Robot Moving), c(n) == 0 (Robot Not Moving)
	#--------------------------------------------------------------------			
	def update_camera(self,n,x,p,z,v,c,R):
		
		# Pre initialize
		x_hat = copy(x)

		# Calculate observation matrix
		H = self.calc_H(n,x,v)
		
		# Kalman gain
		K = p*H.T*inv(H*p*H.T+R);
		
		# Sigma Update
		p_hat = (eye((SL-1)*n) - K*H)*p 
		
		# Residual
		r = zeros((RL*(n-1),1))
		q_G_O = x[QUAT,0]
		R_G_O = quat2rot(q_G_O)
		R_O_C = quat2rot(self.q_O_C)
		R_G_C = R_O_C*R_G_O
		p_O = x[POS,0]

		# Pickets
		for ii in range(1,n):
			jj = ii*SL # x_hat
			if v[ii-1]:
				p_B = x[POS+ii*SL,0]
				q_G_B = x[QUAT+jj,0]

				z_quat_i = quat_norm(z[QUAT+ZL*(ii-1),0])
				z_quat_i_hat = quat_mult(q_G_B,quat_inv(quat_mult(self.q_O_C,q_G_O)))
				z_pos_i = z[POS+ZL*(ii-1),0]
				z_pos_i_hat = R_G_C*(p_B-p_O-self.p_C)

				if norm(z_pos_i) > 3:
					print "Warning: Beyond max distance!"
					print norm(z_pos_i)
					return (x,p)

				r[POBS + RL*(ii-1),0] = vstack((quat_pi(quat_mult(quat_inv(z_quat_i_hat),z_quat_i)),
												z_pos_i-z_pos_i_hat))
		
		# Correction
		Kr = K*r
		
		# X_hat update
		for ii in range(0,n):
			jj = ii*SL # x_hat
			kk = ii*PL # P

			x_hat[QUAT+jj,0] = quat_mult(x[QUAT+jj,0],quat_pi_inv(Kr[PTHETA+kk,0]))
			x_hat[NONQUAT+jj,0] = x_hat[NONQUAT+jj,0] + Kr[PNONQUAT+kk,0]

			# Stationary Target == no outlier rejection
			#if not c[ii,0]:
			#	pass
			# Moving Target == outlier rejection
			#else:
			#	if sum(abs((x_hat[POS+jj,0] - x_prev[POS+jj,0]))) > 3.0:
			#		x_hat = copy(x)
			#		p_hat = copy(p)

		return (x_hat,p_hat)
		
	#--------------------------------------------------------------------
	# function: init_G_c(n,c)
	# Purpose: Initialize the block structure of G_c matrix (n-robots) 
	# for the fixed terms.
	#--------------------------------------------------------------------
	# Example:
	# G_c(n = 2) = | 1 0 0 0 | 0 ...     |
	#              | 0 0 0 0 | 0 ...     |
	#              | 0 R 0 0 | 0 ...     |
	#		       | 0 0 1 0 | 0 ...     |
	#		       | 0 0 0 1 | 0 ...     |
	#              -----------------------
	#		       | 0 ...     | 1 0 0 0 |
	#		       | 0 ...     | 0 0 0 0 |
	#		       | 0 ...     | 0 R 0 0 |
	#		       | 0 ...     | 0 0 1 0 |
	#		       | 0 ...     | 0 0 0 1 |
	# Where :
	#	1 = 1 (3x3), 0 = 0 (3x3)
	#   R = -R_T (3x3) (calculated later)
	#--------------------------------------------------------------------
	def init_G_c(self,n,c):
		
		# Create the empty matrix
		g_c = zeros((PL*n,(SL-4)*n))
		
		# Create the  1 terms
		for ii in range(0,n):
			
			if c[ii,0]:
			
				# Index change for ii-th robot
				kk = ii*PL # P
				ll = ii*QL # G_c || Q_c
				
				g_c[PTHETA+kk,NG.T+ll] = eye(3)
				g_c[PBG+kk,NWG.T+ll] = eye(3)
				g_c[PBA+kk,NWA.T+ll] = eye(3)
			
		return (g_c,copy(g_c))
		
	#--------------------------------------------------------------------
	# function: calc_Q_c(n)
	# Purpose: Initialize the block structure of Q_c matrix (n-robots) 
	# for the fixed terms.
	# Note: The IMU noise characteristics are assumed to be ~ for all
	# robots.
	#--------------------------------------------------------------------
	# Example:
	# Q_c(i = 2) = | s2_g     0     0    0 | 0 ...                 |
	#              |    0  s2_a     0    0 | 0 ...                 |
	#              |    0     0 s2_wg    0 | 0 ...                 |
	#		       |    0     0     0 s2_wa| 0 ...                 |
	#              ------------------------------------------------------
	#		       | 0 ...                 | s2_g     0     0    0 |
	#		       | 0 ...                 |    0  s2_a     0    0 |
	#		       | 0 ...                 |    0     0 s2_wg    0 |
	#		       | 0 ...                 |    0     0     0 s2_wa|
	# Where :
	#	0    = (3x3)
	#   s2_X = (3x3)
	#--------------------------------------------------------------------
	def calc_Q_c(self,n,s2_g,s2_a,s2_wg,s2_wa):
		
		# Create the empty matrix
		Q_c = zeros((QL*n,QL*n))
		
		for ii in range(0,n):

			# Index change for ii-th robot
			ll = ii*QL # G_c || Q_c
			
			Q_c[NG+ll ,NG.T+ll ] = s2_g*eye(3)
			Q_c[NA+ll ,NA.T+ll ] = s2_a*eye(3)
			Q_c[NWG+ll,NWG.T+ll] = s2_wg*eye(3)
			Q_c[NWA+ll,NWA.T+ll] = s2_wa*eye(3)
		
		return Q_c
	
	#--------------------------------------------------------------------
	# function: calc_H(n,x,c)
	# Purpose: Calculate the block structure of H
	#--------------------------------------------------------------------
	# Example:
	# H_i    = |                 -R_G_C       0 ... R_G_C     0 ...|       
	#          |  R_G_C|(p_B-p_O-p_C)x| - R_G_C ...     0 R_G_C ...|
	# Where :
	#	0    = (3x3)
	#--------------------------------------------------------------------

	def calc_H(self,n,x,v):
		
		# Create the empty matrix
		H = zeros((RL*(n-1),PL*n))
		
		# Create the H matrix where measurements received
		R_G_O = quat2rot(x[QUAT,0])
		R_O_C = quat2rot(self.q_O_C)
		R_G_C = R_O_C*R_G_O
		p_O = x[POS,0]

		for ii in range(0,n-1):
			if (v[ii,0]):

				p_B = x[POS+(ii+1)*SL,0]

				# Quaternion measurement sub-Jacobian
				H[PTHETA+RL*ii,PTHETA.T] = -R_G_C
				H[PTHETA+RL*ii,PTHETA.T+(ii+1)*PL] = R_G_C
				
				# Position measurement sub-Jacobian
				H[PPOS+RL*ii,PTHETA.T] = R_G_C*quat_skew(p_B-p_O-self.p_C)
				H[PPOS+RL*ii,PPOS.T] = -R_G_C
				H[PPOS+RL*ii,PPOS.T+(ii+1)*PL] = R_G_C
				
		return H