#!/usr/bin/python

import sys
import rospy
import csv

import tf

from geometry_msgs.msg import TransformStamped


class visualize_vicon():

	def __init__(self):
		self.tf_pub = tf.TransformBroadcaster()

		rospy.Subscriber("/vicon/one/one_segment", TransformStamped, self.callback)

	def callback(self, data):
		self.tf_pub.sendTransform((data.transform.translation.x, data.transform.translation.y, data.transform.translation.z),
                		(data.transform.rotation.x, data.transform.rotation.y, data.transform.rotation.z, data.transform.rotation.w), rospy.Time.now(), "observer", "world")

def main(args):
    rospy.init_node('visualize_vicon', anonymous=True)
    v = visualize_vicon()
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)

