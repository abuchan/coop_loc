#!/usr/bin/python

import rospy

from coop_loc.msg import IMUwithMoving
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist

class AddMovingIMU():
  def __init__(self, name='add_moving_imu'):
    rospy.init_node(name)
    
    rospy.Subscriber('imu_0', Imu, self.imu_callback, queue_size=1)
    rospy.Subscriber('command', Twist, self.command_callback, queue_size=1)
    
    self.imu_moving_pub = rospy.Publisher('imu_moving', IMUwithMoving, queue_size=1)
    self.moving = False

  def imu_callback(self, msg):
    imu_moving_msg = IMUwithMoving(
      msg.header, msg.angular_velocity, msg.linear_acceleration, self.moving
    )
    self.imu_moving_pub.publish(imu_moving_msg)

  def command_callback(self, msg):
    self.moving = abs(msg.linear.x) > 0.001 or abs(msg.angular.z) > 0.001

if __name__ == '__main__':
  amu = AddMovingIMU()
  rospy.spin()
