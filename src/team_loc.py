#!/usr/bin/python

#-----------------------------------------------------------------------
# Brian Nemsick
# brian.nemsick@eecs.berkeley.edu
# June 2015
# team_loc.py
#-----------------------------------------------------------------------
# Purpose: Manage sensors for the EKF
#-----------------------------------------------------------------------

import sys
import rospy
import threading
import csv

import coop_loc_EKF
import sensor_meas

import tf

from coop_loc.msg import IMUwithMoving
from geometry_msgs.msg import PoseWithCovarianceStamped

from ind import *
from quaternion import *

import numpy as np
from numpy import matrix, diag, argsort, asarray, vstack, inf, copy
from numpy.matlib import eye, zeros, ones, identity,diag

class team_loc():

	def __init__(self):

		# Team Setup
		self.n = rospy.get_param("n")
		self.t = 0
		self.t_stop_bias = rospy.get_param("t_stop_bias")
		self.robot_prefix = rospy.get_param("robot_prefix")
		self.robot_suffix = rospy.get_param("robot_suffix")
						
		# Camera Transform
		self.p_C = matrix(rospy.get_param("p_C")).T
		self.q_O_C = quat_norm(matrix(rospy.get_param("q_O_C")).T)

		# IMU Bias
		s2_g  = rospy.get_param("s2_g")
		s2_a  = rospy.get_param("s2_a")
		s2_wg = rospy.get_param("s2_wg")
		s2_wa = rospy.get_param("s2_wa")
		self.gd = matrix(rospy.get_param("gd")).T

		# IMU Measurent Buffer (Trapezoidal Integration)
		self.u = zeros((self.n*UL,2))
		self.c_imu = zeros((self.n,2))
		self.c_imu_t = zeros((self.n,1)) - inf
		if (rospy.has_param('c_lock')):
			self.c_lock = rospy.get_param('c_lock')
		else:
			self.c_lock = False

		# Sensor Measurement Buffer
		self.sensor_manager = sensor_meas.sensor_meas()

		# EKF Handle
		self.EKF = coop_loc_EKF.EKF(self.n,s2_g,s2_a,s2_wg,s2_wa,self.q_O_C,self.p_C)

		# Initial Conditions
		init = False
		while (not init):
			if (rospy.has_param('initial_state') & rospy.has_param('initial_covariance') & rospy.has_param('initial_time') &rospy.has_param('t_bias')):
				self.x = matrix(rospy.get_param('initial_state'))
				self.x_prev = copy(self.x)
				self.p = matrix(rospy.get_param('initial_covariance'))
				self.t_bias = matrix(rospy.get_param('t_bias'))

				rospy.delete_param('initial_state')
				rospy.delete_param('initial_covariance')
				rospy.delete_param('initial_time')
				rospy.delete_param('t_bias')
				init = True

		for ii in range(self.n):
			
			def imu_anonymous_callback(self,jj):
				return lambda msg: self.imu_callback(jj,msg)

			rospy.Subscriber(self.robot_prefix+self.robot_suffix[ii]+"/imu", 
							 IMUwithMoving, imu_anonymous_callback(self,ii))

			def camera_anonymous_callback(self,jj):
				return lambda msg: self.camera_callback(jj,msg)

			if (ii > 0):
				rospy.Subscriber(self.robot_prefix+self.robot_suffix[ii]+"/relative_pose",
								 PoseWithCovarianceStamped,camera_anonymous_callback(self,ii))

		# TF Publisher
		self.tf_pub = tf.TransformBroadcaster()

		self.logger = rospy.get_param("logger")

		if self.logger:
			#CSV Writer for logging
			self.log_path = rospy.get_param("log_path")
			np.savetxt(self.log_path + "t.csv", np.array(zeros(1)), delimiter=",")
			np.savetxt(self.log_path + "x.csv", np.array(zeros(self.n*SL)), delimiter=",")
			np.savetxt(self.log_path + "p.csv", np.array(zeros(self.n*SL)), delimiter=",")
			np.savetxt(self.log_path + "c.csv", np.array(zeros(self.n)), delimiter=",")

		# Threading
		self.lock = threading.Condition()

	def imu_callback(self,ii,msg):

		msg.header.frame_id = str(ii)
		self.sensor_manager.add_meas(msg,ii,IMU,self.t_bias[ii,0])

	def camera_callback(self,ii,msg):

		msg.header.frame_id = str(ii)
		self.sensor_manager.add_meas(msg,ii,POSE,self.t_bias[0,0])

	def publish_state(self,x,c):
		
		for ii in range(0,self.n):
			if c[ii,0]:
				if ii == 0:
					self.tf_pub.sendTransform((x[4+ii*SL,0], x[5+ii*SL,0], x[6+ii*SL,0]),
                		(x[0+ii*SL,0], x[1+ii*SL,0], x[2+ii*SL,0], x[3+ii*SL,0]),
                		rospy.Time.now(), "observer", "world")
				else:
					self.tf_pub.sendTransform((x[4+ii*SL,0], x[5+ii*SL,0], x[6+ii*SL,0]),
                		(x[0+ii*SL,0], x[1+ii*SL,0], x[2+ii*SL,0], x[3+ii*SL,0]),
                		rospy.Time.now(), ("p-"+str(ii)), "world")
					#print '['+str(x[0+ii*SL,0]) +',' + str(x[1+ii*SL,0])+ ',' + str(x[1+ii*SL,0]) + ',' + str(x[3+ii*SL,0]) + ']'

	def publish_relative_pose(self,z,c):

		for ii in range (0,self.n-1):
			if c[ii,0]:
				q_C_B = z[QUAT+(ii)*ZL,0]
				p_C_B = z[POS+(ii)*ZL,0]

	    		# Convert to Observer Frame
				q_O_B = quat_mult(q_C_B,self.q_O_C)
				p_O_B   = quat2rot(quat_inv(self.q_O_C))*p_C_B + self.p_C

				self.tf_pub.sendTransform((p_O_B[0,0], p_O_B[1,0], p_O_B[2,0]),
					(q_O_B[0,0], q_O_B[1,0], q_O_B[2,0], q_O_B[3,0]),
					rospy.Time.now(), ("z-"+str(ii+1)), "observer")

	def state_estimation(self):

		self.lock.acquire()

		sensor_meas, sensor_type, sensor_time = self.sensor_manager.get_meas()
		self.sensor_manager.flush_meas()

		self.lock.release()

		# No Measurements
		if len(sensor_meas) == 0:
			return

		for ii in range(0,len(sensor_meas)):

			# Time Step
			dt = sensor_time[ii] - self.t
			self.t = sensor_time[ii]

			if (dt > 1):
				print 'Time ' + str(self.t) + ', DT ' + str(dt)
				print int(sensor_meas[ii].header.frame_id)

			# IMU
			if (sensor_type[ii] == IMU):

				# Process Measurement
				imu_msg = sensor_meas[ii]
				jj = int(imu_msg.header.frame_id)
				accel = matrix([imu_msg.linear_acceleration.x,imu_msg.linear_acceleration.y,imu_msg.linear_acceleration.z]).T
				gyro  = matrix([imu_msg.angular_velocity.x,imu_msg.angular_velocity.y,imu_msg.angular_velocity.z]).T

				# Moving
				if imu_msg.ismoving:

					# Trapezoidal integration
					if self.c_imu[jj,1]:
						self.u[U+jj*UL,0] = self.u[U+jj*UL,1]    # Shift previous measurement
						self.u[U+jj*UL,1] = vstack((gyro,accel)) # Add new measurement
						self.c_imu[jj,0] = self.c_imu[jj,1]      # Shift previous ismoving
						self.c_imu[jj,1] = 1                     # Add new ismoving
						self.c_imu_t[jj,0] = sensor_time[ii]     # Last time the robot is moving

					# Euler integration
					else:
						self.u[U+jj*UL,1] = vstack((gyro,accel)) # Add new measurement
						self.u[U+jj*UL,0] = self.u[U+jj*UL,1]    # Copy new measurement
						self.c_imu[jj,1] = 1                     # Add new ismoving
						self.c_imu[jj,0] = self.c_imu[jj,1]      # Copy new ismoving
						self.c_imu_t[jj,0] = sensor_time[ii]   # Last time the robot is moving
				# Not moving
				else:
					self.u[U+jj*UL,0] = self.u[U+jj*UL,1]    # Shift previous measurement
					self.u[U+jj*UL,1] = vstack((gyro,accel)) # Add new measurement
					self.c_imu[jj,0] = self.c_imu[jj,1]      # Shift previous ismoving
					self.c_imu[jj,1] = 0                     # Add new ismoving

				# Stopping time and movement restriction lock
				c_imu = np.logical_or(self.c_imu[:,1],self.t-self.c_imu_t < self.t_stop_bias)
				if c_imu[0,0] and self.c_lock:
					c_imu[1:self.n,0] = False

				# EKF propagate
				if dt > 0:
					[self.x,self.p] = self.EKF.propagate_IMU(self.n,self.x,self.p,self.u,c_imu,dt,self.gd)

			# Relative Pose
			elif (sensor_type[ii] == POSE):
				
				# Process Measurement
				pose_msg = sensor_meas[ii]
				jj = int(pose_msg.header.frame_id)-1
				quat = pose_msg.pose.pose.orientation
				pos  = pose_msg.pose.pose.position
				cov  = diag(diag(matrix(pose_msg.pose.covariance).reshape(6,6)))

				# Form Z, R, v_cam
				z = eye((self.n-1)*ZL)
				z[Z+jj*ZL,0] = vstack((quat.x,quat.y,quat.z,quat.w,pos.x,pos.y,pos.z))

				R = 1e-25*eye((self.n-1)*RL) #1e-25 prevents singularity from floating point arithmetic

				# Stopping time and movement restriction lock
				c_imu = np.logical_or(self.c_imu[:,1],self.t-self.c_imu_t < .2)
				if c_imu[0,0] and self.c_lock:
					c_imu[1:self.n,0] = False


				if c_imu[0,0]:
					R[ZCOV+jj*RL,ZCOV.T +jj*RL] = cov/10
				else:
					R[ZCOV+jj*RL,ZCOV.T +jj*RL] = cov*10

				v_cam = zeros((self.n-1,1))
				v_cam[jj,0] = True


				# EKF propagate & update

				# Time difference = propagate & update
				if dt > 0:
					[self.x,self.p] = self.EKF.propagate_IMU(self.n,self.x,self.p,self.u,c_imu,dt,self.gd)
					[self.x,self.p] = self.EKF.update_camera(self.n,self.x,self.p,z,v_cam,c_imu,R)
				# No time difference = update
				elif dt == 0:
					[self.x,self.p] = self.EKF.update_camera(self.n,self.x,self.p,z,v_cam,c_imu,R)

				if self.logger:
					with open(self.log_path + "t.csv",'a') as f_handle:
						np.savetxt(f_handle, [self.t], delimiter=",")
					with open(self.log_path + "p.csv",'a') as f_handle:
						np.savetxt(f_handle, [diag(self.p)], delimiter=",")
					with open(self.log_path + "x.csv",'a') as f_handle:
						np.savetxt(f_handle, np.transpose(np.array(self.x)), delimiter=",")
					with open(self.log_path + "c.csv",'a') as f_handle:
						np.savetxt(f_handle, np.transpose(np.array(self.c_imu[:,1])), delimiter=",")

				#self.publish_relative_pose(z,v_cam)
				self.publish_state(self.x,vstack((matrix('1'),v_cam)))

def main(args):
    rospy.init_node('team_loc()', anonymous=True)
    team = team_loc()
    rate = rospy.Rate(2) #2 Hz
    while not rospy.is_shutdown():
    	team.state_estimation()
    	rate.sleep()
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)



