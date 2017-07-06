#!/usr/bin/python

import sys
import rospy
import csv

import numpy as np

from geometry_msgs.msg import TransformStamped

class vicon2csv():

	def __init__(self):
		
		self.log_path = rospy.get_param("log_path")

		np.savetxt(self.log_path + "obs.csv", np.array(np.zeros(7)), delimiter=",")
		np.savetxt(self.log_path + "p1.csv", np.array(np.zeros(7)), delimiter=",")
		np.savetxt(self.log_path + "p2.csv", np.array(np.zeros(7)), delimiter=",")

		self.obs = rospy.Subscriber("/vicon/observer/observer_segment",TransformStamped, self.obs_callback)
		self.p1 = rospy.Subscriber("/vicon/one/one_segment",TransformStamped, self.p1_callback)
		self.p2 = rospy.Subscriber("/vicon/two/two_segment",TransformStamped, self.p2_callback)

		print self.log_path + "obs.csv"

	def obs_callback(self,msg):
		msg_csv = np.array([msg.transform.rotation.x,msg.transform.rotation.y,msg.transform.rotation.z,msg.transform.rotation.w, msg.transform.translation.x,msg.transform.translation.y,msg.transform.translation.z])
		with open(self.log_path + "obs.csv",'a') as f_handle:
			np.savetxt(f_handle, [msg_csv], delimiter=",")
	def p1_callback(self,msg):
		msg_csv = np.array([msg.transform.rotation.x,msg.transform.rotation.y,msg.transform.rotation.z,msg.transform.rotation.w, msg.transform.translation.x,msg.transform.translation.y,msg.transform.translation.z])
		with open(self.log_path + "p1.csv",'a') as f_handle:
			np.savetxt(f_handle, [msg_csv], delimiter=",")
	def p2_callback(self,msg):
		msg_csv = np.array([msg.transform.rotation.x,msg.transform.rotation.y,msg.transform.rotation.z,msg.transform.rotation.w, msg.transform.translation.x,msg.transform.translation.y,msg.transform.translation.z])
		with open(self.log_path + "p2.csv",'a') as f_handle:
			np.savetxt(f_handle, [msg_csv], delimiter=",")
def main(args):
	rospy.init_node('vicon2csv()', anonymous=True)
	handle = vicon2csv()
	rospy.spin()

if __name__ == '__main__':
    main(sys.argv)



