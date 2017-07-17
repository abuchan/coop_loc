#!/usr/bin/python

#-----------------------------------------------------------------------
# Brian Nemsick
# brian.nemsick@eecs.berkeley.edu
# June 2015
# team_init.py
#-----------------------------------------------------------------------
# Purpose: Basic initialization procedure for the EKF
#-----------------------------------------------------------------------

import sys
import rospy
import threading

from coop_loc.msg import IMUwithMoving
from geometry_msgs.msg import PoseWithCovarianceStamped

from ind import *
from quaternion import *

from numpy import matrix, diag
from numpy.matlib import eye, zeros

class team_init():

  def __init__(self):

    # Team Setup
    self.n = rospy.get_param("n")
    self.robot_prefix = rospy.get_param("robot_prefix")
    self.robot_suffix = rospy.get_param("robot_suffix")

    # Time
    self.init_time = rospy.get_param("~init_time")
    self.start_time = 0

    # robot_00 initial position
    self.r0_x0 = matrix(rospy.get_param("r0_x0")).T

    # Camera Transform
    self.p_C = matrix(rospy.get_param("p_C")).T
    self.q_O_C = quat_norm(matrix(rospy.get_param("q_O_C")).T)

    # IMU Bias
    s2_g  = rospy.get_param("s2_g")
    s2_a  = rospy.get_param("s2_a")
    IMU_hz = rospy.get_param("imu_hz")
    self.IMU_samples = zeros((self.n,1), dtype = int)

    # Initial EKF Values - Observer is origin

    self.x = zeros((SL*self.n,1),dtype = float)
    self.p = 1e-25*eye(PL*self.n,dtype = float) #1e-25 prevents singularity from floating point arithmetic
    self.t_bias = zeros((self.n,1))

    for ii in range(self.n):
      self.x[QUAT+ii*SL,0] = matrix('0.0;0.0;0.0;1.0')
      
      # High uncertainty if no pose estimate
      if (ii != 0):
        self.p[PPOSE + ii*PL, PPOSE.T + ii*PL] = 100.0* eye((RL))
      
      self.p[PBG+ii*PL,PBG.T+ii*PL] = s2_g/(IMU_hz*self.init_time) * eye(3)
      self.p[PBA+ii*PL,PBA.T+ii*PL] = s2_a/(IMU_hz*self.init_time) * eye(3)

    # Threading
    self.lock = threading.Condition()

    # Subscribers
    for ii in range(self.n):
      
      def imu_anonymous_callback(self,jj):
        return lambda msg: self.imu_callback(jj,msg)

      rospy.Subscriber(self.robot_prefix+self.robot_suffix[ii]+"/imu_moving", IMUwithMoving, imu_anonymous_callback(self,ii))

      def camera_anonymous_callback(self,jj):
        return lambda msg: self.camera_callback(jj,msg)

      rospy.Subscriber(self.robot_prefix+self.robot_suffix[ii]+"/camera_0/estimated_pose",PoseWithCovarianceStamped,camera_anonymous_callback(self,ii))

  def imu_callback(self,ii,msg):
#print "Got IMU from robot %d" % ii
    # Process Message
    t = msg.header.stamp.to_sec()

    accel = matrix([msg.linear_acceleration.x,msg.linear_acceleration.y,msg.linear_acceleration.z]).T + g
    gyro  = matrix([msg.angular_velocity.x,msg.angular_velocity.y,msg.angular_velocity.z]).T
    self.IMU_samples[ii,0] += 1
    
    # Update Bias Estimate (avg)    
    self.x[BG +ii*SL,0] = (gyro  + (self.IMU_samples[ii,0]-1)*self.x[BG +ii*SL,0])/self.IMU_samples[ii,0]
    self.x[BA +ii*SL,0] = (accel + (self.IMU_samples[ii,0]-1)*self.x[BA +ii*SL,0])/self.IMU_samples[ii,0]
    self.t_bias[ii,0] = max(self.t_bias[ii,0],msg.header.stamp.to_sec())

    self.check_time(t-self.t_bias[ii,0])

  def camera_callback(self,ii,msg):
    #print 'Got pose estimate %s from camera %d' % (msg.header.frame_id, ii) 
    obs_ii = int(msg.header.frame_id[-1])

    # Process Message
    t = msg.header.stamp.to_sec()
    quat = msg.pose.pose.orientation
    pos  = msg.pose.pose.position
    cov  = matrix(msg.pose.covariance).reshape(6,6)

    q_C_B = matrix([quat.x,quat.y,quat.z,quat.w]).T
    p_C_B = matrix([pos.x, pos.y, pos.z]).T

    # Convert to Observer Frame (origin)
    q_O_B = quat_mult(q_C_B,self.q_O_C)
    p_O_B   = quat2rot(quat_inv(self.q_O_C))*p_C_B + self.p_C

    # Update Pose Estimate
    self.x[QUAT + obs_ii*SL,0] = q_O_B
    self.x[POS + obs_ii*SL,0] = p_O_B
    self.p[PPOSE + obs_ii*PL, PPOSE.T+ obs_ii*PL] = cov

    self.check_time(t)

  def check_time(self,t):

    self.lock.acquire()

    if (self.start_time == 0):
      self.start_time = t
    elif (t - self.start_time > self.init_time):

      rospy.set_param('initial_state',self.x.tolist())
      rospy.set_param('initial_covariance',self.p.tolist())
      rospy.set_param('initial_time', t)
      rospy.set_param('t_bias',self.t_bias.tolist())

      self.lock.release()
      
      rospy.signal_shutdown("*** Team Initialization Complete ***")

    self.lock.release()

def main(args):
    rospy.init_node('team_init()', anonymous=True)
    team = team_init()
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)



